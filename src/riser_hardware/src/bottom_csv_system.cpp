#include "elm3604.hpp"
#include "footplate_pitch.hpp"
#include "g5_axis.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <ecrt.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <riser_hardware/csv_logic.hpp>
#include <riser_hardware/g5_topology.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace riser_hardware {
namespace {
constexpr uint32_t kOmronVendor = 0x00000083;
constexpr uint32_t kRotaryG5Product = 0x00000004; // R88D-KN04L-ECT
constexpr char kFootplate[] = "foot_plate_pitch";
enum BottomCommand : unsigned { Linear = 1, PitchPosition = 2, PitchVelocity = 4 };

unsigned bottom_commands(const std::vector<std::string> &keys) {
  unsigned modes = 0;
  for (const auto &key : keys) {
    if (key == "bottom_motor/velocity")
      modes |= Linear;
    else if (key == "foot_plate_pitch/position")
      modes |= PitchPosition;
    else if (key == "foot_plate_pitch/velocity")
      modes |= PitchVelocity;
  }
  return modes;
}

} // namespace

class BottomCsvSystem final : public hardware_interface::SystemInterface {
  ec_master_t *master_ = nullptr;
  ec_domain_t *domain_ = nullptr;
  uint8_t *data_ = nullptr;
  std::vector<G5Axis> axes_;
  FootplatePitch pitch_;
  bool has_pitch_ = false;
  std::atomic<unsigned> bottom_claim_{0};
  Elm3604 elm_;
  bool force_plates_ = false;
  int coupler_position_ = -1, terminal_position_ = -1;
  std::vector<double> excluded_discovered_;
  double position_scale_ = 0, velocity_scale_ = 0, speed_limit_ = 0;
  uint32_t cycle_ns_ = 1000000;
  bool active_ = false;
  bool commissioning_disabled_ = true;
  std::chrono::steady_clock::time_point last_{};
  double number(const std::string &key) {
    const auto &s = info_.hardware_parameters.at(key);
    size_t end = 0;
    double v = std::stod(s, &end);
    if (end != s.size() || !std::isfinite(v))
      throw std::invalid_argument(key);
    return v;
  }
  uint32_t integer(const std::string &key) {
    const auto &s = info_.hardware_parameters.at(key);
    size_t end = 0;
    if (s.empty() || s.front() < '0' || s.front() > '9')
      throw std::invalid_argument(key);
    const auto v = std::stoul(s, &end, 0);
    if (end != s.size() || v > UINT32_MAX)
      throw std::invalid_argument(key);
    return static_cast<uint32_t>(v);
  }
  void check_topology() {
    ec_master_info_t bus{};
    if (ecrt_master(master_, &bus))
      throw std::runtime_error("Cannot read EtherCAT master inventory");
    if (bus.scan_busy || !bus.link_up || bus.slave_count > 6)
      throw std::runtime_error("EtherCAT topology unavailable: link " +
                               std::string(bus.link_up ? "up" : "down") +
                               ", slaves " + std::to_string(bus.slave_count) +
                               ", scan busy " + std::to_string(bus.scan_busy));
    std::vector<G5Identity> detected;
    std::set<uint32_t> peripherals;
    int coupler = -1, terminal = -1;
    for (unsigned i = 0; i < bus.slave_count; ++i) {
      ec_slave_info_t found{};
      if (ecrt_master_get_slave(master_, i, &found))
        throw std::runtime_error("Cannot read slave identity at " +
                                 std::to_string(i));
      if (found.vendor_id == Elm3604::vendor &&
          found.product_code == Elm3604::coupler && coupler < 0) {
        coupler = i;
        peripherals.insert(i);
      }
      if (found.vendor_id == Elm3604::vendor &&
          found.product_code == Elm3604::product &&
          found.revision_number == Elm3604::revision && terminal < 0) {
        terminal = i;
        peripherals.insert(i);
      }
      detected.push_back({found.vendor_id, found.product_code,
                          found.revision_number, found.alias,
                          found.serial_number});
    }
    std::vector<AxisAddress> addresses;
    for (const std::string name :
         {"bottom_motor", "top_motor", "torso_motor", "hip_motor"}) {
      if (!info_.hardware_parameters.count(name + "_position"))
        continue;
      const bool enabled =
          std::any_of(axes_.begin(), axes_.end(),
                      [&](const auto &a) { return a.name == name; });
      addresses.push_back({name, integer(name + "_position"),
                           integer(name + "_alias"), integer(name + "_serial"),
                           enabled});
    }
    validate_topology(addresses, detected, peripherals);
    if (force_plates_ &&
        (coupler < 0 || terminal != coupler + 1 ||
         (elm_.domain && (coupler != coupler_position_ ||
                          terminal != terminal_position_))))
      throw std::runtime_error(
          "Expected the EK1100 followed by the ELM3604-0002");
    coupler_position_ = coupler;
    terminal_position_ = terminal;
    for (size_t i = 0; i < info_.sensors.size(); ++i) {
      const auto &sensor = info_.sensors[i].name;
      if (is_force_plate(sensor))
        continue;
      const auto name = sensor.substr(0, sensor.size() - 9) + "_motor";
      const auto found =
          std::find_if(addresses.begin(), addresses.end(),
                       [&](const auto &a) { return a.name == name; });
      excluded_discovered_[i] =
          found != addresses.end() && found->ring < detected.size() &&
          !peripherals.count(found->ring);
    }
  }
  static bool is_force_plate(const std::string &name) {
    return name == "left_force_plate" || name == "right_force_plate";
  }

  void release() {
    if (master_)
      ecrt_release_master(master_);
    master_ = nullptr;
    domain_ = nullptr;
    elm_ = {};
    pitch_ = {};
    bottom_claim_ = 0;
    for (auto &a : axes_) {
      a.discovered = 0;
      a.feedback_valid = 0;
      a.ready_ = 0;
    }
    std::fill(excluded_discovered_.begin(), excluded_discovered_.end(), NAN);
    data_ = nullptr;
  }

  void send(bool stopping) {
    const int reference = ecrt_master_sync_reference_clock(master_);
    const int slaves = ecrt_master_sync_slave_clocks(master_);
    const unsigned mode = bottom_claim_;
    for (auto &a : axes_) {
      if (reference || slaves)
        a.guard_.trip();
      if (!stopping && mode > Linear && a.name == "bottom_motor")
        a.command_ =
            pitch_.carriage_command(mode == PitchPosition, a.position_, speed_limit_);
      a.stage(data_, commissioning_disabled_, stopping, speed_limit_,
              velocity_scale_);
    }
    const int queued = ecrt_domain_queue(domain_);
    if (force_plates_ && ecrt_domain_queue(elm_.domain))
      elm_.clear();
    const int sent = ecrt_master_send(master_);
    for (auto &a : axes_) {
      if (queued || sent) {
        a.guard_.trip();
        a.controlword = a.brake_request = NAN;
      } else {
        a.controlword = EC_READ_U16(data_ + a.cw_);
        // Host servo enable request. G5 sequences the assigned brake output
        // internally.
        a.brake_request = a.controlword == 0x000f;
      }
    }
  }

public:
  ~BottomCsvSystem() override { release(); }
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override {
    const auto &info = params.hardware_info;
    if (SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS ||
        info.joints.empty() || info.joints.size() > 5)
      return hardware_interface::CallbackReturn::ERROR;
    excluded_discovered_.assign(info.sensors.size(), NAN);
    for (const auto &sensor : info.sensors) {
      if (is_force_plate(sensor.name)) {
        force_plates_ = true;
        continue;
      }
      if (sensor.name.size() < 10 ||
          sensor.name.substr(sensor.name.size() - 9) != "_excluded" ||
          sensor.state_interfaces.size() != 1 ||
          sensor.state_interfaces[0].name != "discovered")
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::set<std::string> joint_names;
    const std::set<std::string> supported{"bottom_motor", "top_motor",
                                          "torso_motor", "hip_motor"};
    for (const auto &j : info.joints) {
      if (!joint_names.insert(j.name).second)
        return hardware_interface::CallbackReturn::ERROR;
      if (j.name == kFootplate) {
        has_pitch_ = true;
        continue;
      }
      if (!supported.count(j.name))
        return hardware_interface::CallbackReturn::ERROR;
      auto &a = axes_.emplace_back();
      a.name = j.name;
      a.key = a.name + "/velocity";
      std::vector<hardware_interface::StateInterface> expected;
      a.export_states(expected);
      if (j.command_interfaces.size() != 1 ||
          j.command_interfaces[0].name != "velocity" ||
          expected.size() != j.state_interfaces.size())
        return hardware_interface::CallbackReturn::ERROR;
      for (const auto &state : expected)
        if (std::none_of(j.state_interfaces.begin(), j.state_interfaces.end(),
                         [&](const auto &v) {
                           return v.name == state.get_interface_name();
                         }))
          return hardware_interface::CallbackReturn::ERROR;
    }
    if (axes_.empty() || (has_pitch_ && !joint_names.count("bottom_motor")))
      return hardware_interface::CallbackReturn::ERROR;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (auto &a : axes_)
      a.export_states(result);
    if (has_pitch_) {
      result.emplace_back(kFootplate, "position", &pitch_.position);
      result.emplace_back(kFootplate, "velocity", &pitch_.velocity);
      result.emplace_back(kFootplate, "valid", &pitch_.valid);
    }
    if (force_plates_)
      elm_.export_states(result);
    for (size_t i = 0; i < info_.sensors.size(); ++i)
      if (!is_force_plate(info_.sensors[i].name))
        result.emplace_back(info_.sensors[i].name, "discovered",
                            &excluded_discovered_[i]);
    return result;
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto &a : axes_)
      result.emplace_back(a.name, "velocity", &a.command_);
    if (has_pitch_) {
      result.emplace_back(kFootplate, "position", &pitch_.position_command);
      result.emplace_back(kFootplate, "velocity", &pitch_.velocity_command);
    }
    return result;
  }
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    release();
    try {
      position_scale_ = number("position_units_per_m");
      velocity_scale_ = number("velocity_units_per_m_s");
      speed_limit_ = number("max_velocity_m_s");
      cycle_ns_ = integer("cycle_ns");
      const auto shift = info_.hardware_parameters.count("sync0_shift_ns")
                             ? integer("sync0_shift_ns")
                             : 0;
      const auto disabled =
          info_.hardware_parameters.count("commissioning_disabled")
              ? integer("commissioning_disabled")
              : 1;
      const auto product = integer("product_code");

      if (!valid_scale(position_scale_) || !valid_scale(velocity_scale_) ||
          !valid_scale(speed_limit_) || product != kRotaryG5Product ||
          cycle_ns_ < 250000 || cycle_ns_ > 4000000 || shift >= cycle_ns_ ||
          disabled > 1 ||
          !safe_velocity(speed_limit_, speed_limit_, velocity_scale_))
        throw std::invalid_argument("identity, scaling, timing or limits");
      commissioning_disabled_ = disabled != 0;
      const auto &parameters = info_.hardware_parameters;
      if (has_pitch_ && parameters.count("footplate_zero_m") &&
          !pitch_.configure(
              {number("footplate_lower_rad"), number("footplate_upper_rad")},
              number("footplate_rate_rad_s"), number("footplate_zero_m"),
              number("footplate_min_m"), number("footplate_max_m"),
              number("footplate_direction"), number("footplate_position_gain_s_inv")))
        throw std::invalid_argument("Invalid footplate calibration");
      master_ = ecrt_request_master(integer("master_index"));
      if (!master_)
        throw std::runtime_error("request master failed");
      check_topology();
      domain_ = ecrt_master_create_domain(master_);
      if (!domain_)
        throw std::runtime_error("Cannot create PDO domain");
      for (auto &a : axes_) {
        const auto ring = integer(a.name + "_position");
        a.rated_torque = number(a.name + "_rated_torque_nm");
        if (!valid_scale(a.rated_torque))
          throw std::runtime_error("Missing motor rating: " + a.name);
        a.discovered = 1;
        a.slave_ =
            ecrt_master_slave_config(master_, 0, ring, kOmronVendor, product);
        if (!a.slave_)
          throw std::runtime_error("Slave configuration failed: " + a.name);
        a.configure_pdos(domain_, cycle_ns_, shift);
      }
      if (force_plates_ &&
          (cycle_ns_ != 1000000 ||
           !ecrt_master_slave_config(master_, 0, coupler_position_,
                                     Elm3604::vendor, Elm3604::coupler) ||
           !elm_.configure(master_, terminal_position_,
                           info_.hardware_parameters.at("elm_left_matrix"),
                           info_.hardware_parameters.at("elm_gain_excitation"))))
        throw std::runtime_error("Cannot configure the ELM3604 input domain");
      if (ecrt_master_activate(master_) || !(data_ = ecrt_domain_data(domain_)))
        throw std::runtime_error("master activation failed");
      if (force_plates_ && !(elm_.data = ecrt_domain_data(elm_.domain)))
        throw std::runtime_error("Missing ELM3604 process data");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("bottom_csv"), "%s", e.what());
      release();
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    if (!data_)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      check_topology();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("bottom_csv"), "%s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (auto &a : axes_) {
      a.command_ = NAN;
      a.ready_ = a.feedback_valid = 0;
      a.guard_.reset();
      a.stats_ = {};
      a.claimed_ = false;
    }
    bottom_claim_ = 0;
    active_ = true;
    last_ = std::chrono::steady_clock::now();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    if (data_)
      send(true);
    elm_.clear();
    pitch_.read(false, 0, 0);
    bottom_claim_ = 0;
    for (auto &a : axes_) {
      if (commissioning_disabled_ && active_)
        a.summary();
      a.claimed_ = false;
      a.command_ = NAN;
      a.feedback_valid = a.ready_ = 0;
    }
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override {
    release();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) override {
    if (active_)
      on_deactivate(state);
    release();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &state) override {
    return on_shutdown(state);
  }
  hardware_interface::return_type
  prepare_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    const unsigned incoming = bottom_commands(start);
    const unsigned remaining = bottom_claim_ & ~bottom_commands(stop);
    if ((incoming & (incoming - 1)) || (incoming && remaining) ||
        (incoming > Linear && (!has_pitch_ || !pitch_.direction)) ||
        (incoming == PitchPosition && pitch_.position_gain <= 0))
      return hardware_interface::return_type::ERROR;
    if (commissioning_disabled_ && incoming)
      return hardware_interface::return_type::ERROR;
    for (const auto &a : axes_)
      if (commissioning_disabled_ &&
          std::find(start.begin(), start.end(), a.key) != start.end())
        return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    auto result = prepare_command_mode_switch(start, stop);
    if (result != hardware_interface::return_type::OK)
      return result;
    const unsigned incoming = bottom_commands(start);
    const unsigned outgoing = bottom_commands(stop);
    for (auto &a : axes_) {
      if (a.name == "bottom_motor" && (incoming || outgoing)) {
        if (outgoing) {
          bottom_claim_ &= ~outgoing;
          a.claimed_ = bottom_claim_ != 0;
        }
        if (incoming) {
          if (incoming > Linear && (!pitch_.valid || !a.ready_))
            return hardware_interface::return_type::ERROR;
          pitch_.position_command = pitch_.position;
          pitch_.velocity_command = 0;
          bottom_claim_ = incoming;
          a.claimed_ = true;
          a.limit_blocked = false;
          a.command_ = incoming == Linear ? NAN : 0;
        }
        continue;
      }
      if (std::find(stop.begin(), stop.end(), a.key) != stop.end())
        a.claimed_ = false;
      if (std::find(start.begin(), start.end(), a.key) != start.end()) {
        a.claimed_ = true;
        a.limit_blocked = false;
        a.command_ = NAN;
      }
    }
    return result;
  }
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override {
    if (!active_)
      return hardware_interface::return_type::OK;
    const auto now = std::chrono::steady_clock::now();
    // Anchor DC application time before controller and broadcaster execution.
    const auto app_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count();
    const int time_result =
        ecrt_master_application_time(master_, static_cast<uint64_t>(app_time));
    const auto interval = now - last_;
    const double dt = std::chrono::duration<double>(interval).count();
    last_ = now;
    const int receive_result = ecrt_master_receive(master_);
    const int process_result = ecrt_domain_process(domain_);
    ec_domain_state_t domain{};
    const int domain_result = ecrt_domain_state(domain_, &domain);
    if (force_plates_)
      elm_.read(!time_result && !receive_result);
    for (auto &a : axes_) {
      ec_slave_config_state_t slave{};
      const int slave_result = ecrt_slave_config_state(a.slave_, &slave);
      const bool io_ok = !time_result && !receive_result && !process_result &&
                         !domain_result && !slave_result;
      a.read(data_, io_ok, slave, domain, interval, dt, cycle_ns_,
             commissioning_disabled_, position_scale_, velocity_scale_,
             speed_limit_);
      if (has_pitch_ && a.name == "bottom_motor")
        pitch_.read(a.feedback_valid, a.position_, a.velocity_);
    }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override {
    if (!active_)
      return hardware_interface::return_type::OK;
    send(false);
    return hardware_interface::return_type::OK;
  }
};
} // namespace riser_hardware
PLUGINLIB_EXPORT_CLASS(riser_hardware::BottomCsvSystem,
                       hardware_interface::SystemInterface)
