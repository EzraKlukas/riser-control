// Link this executable to the mock IgH functions below. It never opens a master
// device.
#include "../src/bottom_csv_system.cpp"
#include <array>
#include <unordered_map>

struct ec_master {};
struct ec_domain {
  std::array<uint8_t, 256> bytes{};
};
struct ec_slave_config {
  unsigned base = 0;
  std::unordered_map<unsigned, unsigned> offsets;
};

namespace {
ec_master master;
ec_domain domain;
std::array<ec_slave_config, 4> slaves;
unsigned count = 0;
bool complete = true;
} // namespace

ec_master_t *ecrt_request_master(unsigned index) {
  return index == 0 ? &master : nullptr;
}
void ecrt_release_master(ec_master_t *) {}
int ecrt_master(ec_master_t *, ec_master_info_t *info) {
  info->link_up = 1;
  info->slave_count = count;
  return 0;
}
int ecrt_master_get_slave(ec_master_t *, uint16_t index,
                          ec_slave_info_t *info) {
  if (index >= count)
    return -1;
  info->vendor_id = 0x83;
  info->product_code = 4;
  info->revision_number = 0x20001;
  info->serial_number = 100 + index;
  return 0;
}
ec_domain_t *ecrt_master_create_domain(ec_master_t *) { return &domain; }
ec_slave_config_t *ecrt_master_slave_config(ec_master_t *, uint16_t alias,
                                            uint16_t index, uint32_t vendor,
                                            uint32_t product) {
  if (alias || index >= count || vendor != 0x83 || product != 4)
    return nullptr;
  slaves[index].base = index * 64;
  return &slaves[index];
}
int ecrt_slave_config_pdos(ec_slave_config_t *sc, unsigned,
                           const ec_sync_info_t syncs[]) {
  unsigned next = sc->base;
  for (unsigned sm : {2U, 3U}) {
    const auto &map = syncs[sm].pdos[0];
    for (unsigned i = 0; i < map.n_entries; ++i) {
      const auto &entry = map.entries[i];
      sc->offsets[entry.index] = next;
      next += entry.bit_length / 8;
    }
  }
  return next == sc->base + 26 && syncs[2].watchdog_mode == EC_WD_ENABLE ? 0
                                                                         : -1;
}
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t *sc, uint16_t index,
                                    uint8_t sub, ec_domain_t *, unsigned *) {
  return sub || !sc->offsets.count(index)
             ? -1
             : static_cast<int>(sc->offsets.at(index));
}
int ecrt_slave_config_dc(ec_slave_config_t *, uint16_t assign, uint32_t cycle,
                         int32_t shift, uint32_t, int32_t) {
  return assign == 0x0300 && cycle == 1000000 && shift == 650000 ? 0 : -1;
}
int ecrt_master_activate(ec_master_t *) { return 0; }
uint8_t *ecrt_domain_data(const ec_domain_t *d) {
  return const_cast<uint8_t *>(d->bytes.data());
}
int ecrt_master_application_time(ec_master_t *, uint64_t) { return 0; }
int ecrt_master_receive(ec_master_t *) { return 0; }
int ecrt_domain_process(ec_domain_t *) { return 0; }
int ecrt_domain_state(const ec_domain_t *, ec_domain_state_t *state) {
  state->wc_state = complete ? EC_WC_COMPLETE : EC_WC_INCOMPLETE;
  return 0;
}
int ecrt_slave_config_state(const ec_slave_config_t *,
                            ec_slave_config_state_t *state) {
  state->operational = 1;
  return 0;
}
int ecrt_master_sync_reference_clock(ec_master_t *) { return 0; }
int ecrt_master_sync_slave_clocks(ec_master_t *) { return 0; }
int ecrt_domain_queue(ec_domain_t *) { return 0; }
int ecrt_master_send(ec_master_t *) { return 0; }

int exercise(unsigned enabled, bool inhibited) {
  using Callback = hardware_interface::CallbackReturn;
  using Result = hardware_interface::return_type;
  hardware_interface::HardwareComponentInterfaceParams params;
  auto &info = params.hardware_info;
  info.name = "G5Test";
  info.type = "system";
  info.hardware_parameters = {
      {"position_units_per_m", "1000"},
      {"velocity_units_per_m_s", "1000"},
      {"max_velocity_m_s", "1"},
      {"cycle_ns", "1000000"},
      {"master_index", "0"},
      {"product_code", "4"},
      {"sync0_shift_ns", "650000"},
      {"commissioning_disabled", inhibited ? "1" : "0"}};
  std::vector<std::string> keys;
  for (unsigned i = 0; i < enabled; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = std::array<std::string, 4>{"bottom_motor", "top_motor",
                                            "torso_motor", "hip_motor"}[i];
    keys.push_back(joint.name + "/velocity");
    hardware_interface::InterfaceInfo iface;
    iface.name = "velocity";
    joint.command_interfaces.push_back(iface);
    riser_hardware::G5Axis axis;
    std::vector<hardware_interface::StateInterface> states;
    axis.export_states(states);
    for (const auto &state : states) {
      iface.name = state.get_interface_name();
      joint.state_interfaces.push_back(iface);
    }
    info.joints.push_back(joint);
    for (const auto &kv : std::vector<std::pair<std::string, std::string>>{
             {"alias", "0"},
             {"position", std::to_string(i)},
             {"serial", std::to_string(100 + i)},
             {"rated_torque_nm", "1.3"}})
      info.hardware_parameters[joint.name + "_" + kv.first] = kv.second;
  }
  riser_hardware::BottomCsvSystem hw;
  rclcpp_lifecycle::State state;
  if (hw.on_init(params) != Callback::SUCCESS)
    return 1;
  count = enabled - 1;
  if (hw.on_configure(state) == Callback::SUCCESS)
    return 2;
  count = enabled;
  if (hw.on_configure(state) != Callback::SUCCESS ||
      hw.on_activate(state) != Callback::SUCCESS)
    return 3;
  const auto expected = inhibited ? Result::ERROR : Result::OK;
  if (hw.prepare_command_mode_switch(keys, {}) != expected ||
      hw.perform_command_mode_switch(keys, {}) != expected)
    return 4;
  auto commands = hw.export_command_interfaces();
  const auto period = rclcpp::Duration::from_seconds(.001);
  for (unsigned i = 0; i < enabled; ++i) {
    auto &offset = slaves[i].offsets;
    EC_WRITE_U16(domain.bytes.data() + offset.at(0x6041), 0x27);
    EC_WRITE_S8(domain.bytes.data() + offset.at(0x6061), 9);
    if (!commands[i].set_value(i % 2 ? -.5 : .5))
      return 5;
  }
  complete = true;
  hw.read(rclcpp::Time(0), period);
  hw.write(rclcpp::Time(0), period);
  for (unsigned i = 0; i < enabled; ++i) {
    auto &offset = slaves[i].offsets;
    if (EC_READ_U16(domain.bytes.data() + offset.at(0x6040)) !=
            (inhibited ? 0 : 15) ||
        EC_READ_S32(domain.bytes.data() + offset.at(0x60ff)) !=
            (inhibited ? 0 : (i % 2 ? -500 : 500)))
      return 6;
  }
  complete = false;
  hw.read(rclcpp::Time(0), period);
  hw.write(rclcpp::Time(0), period);
  for (unsigned i = 0; i < enabled; ++i)
    if (EC_READ_S32(domain.bytes.data() + slaves[i].offsets.at(0x60ff)) != 0)
      return 7;
  if (hw.on_deactivate(state) != Callback::SUCCESS ||
      hw.on_cleanup(state) != Callback::SUCCESS)
    return 8;
  return 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  for (unsigned n : {1U, 4U})
    for (bool inhibited : {true, false}) {
      const int result = exercise(n, inhibited);
      if (result)
        return result;
    }
  riser_hardware::Elm3604 elm;
  elm.domain = &domain;
  elm.slave = &slaves[0];
  elm.data = domain.bytes.data();
  elm.status = {0, 8, 16};
  elm.sample = {4, 12, 20};
  complete = true;
  for (unsigned i = 0; i < elm.sample.size(); ++i) {
    EC_WRITE_U16(domain.bytes.data() + elm.status[i], 1);
    EC_WRITE_S32(domain.bytes.data() + elm.sample[i], -10 + static_cast<int>(i));
  }
  elm.read(true);
  for (const auto offset : elm.status)
    EC_WRITE_U16(domain.bytes.data() + offset, 0x4001);
  elm.read(true);
  if (elm.valid != 1 || elm.left != std::array<double, 6>{0, -8, -9, -10, 0, 0})
    return 9;
  elm.read(true);
  if (elm.valid != 0)
    return 10;
  rclcpp::shutdown();
}
