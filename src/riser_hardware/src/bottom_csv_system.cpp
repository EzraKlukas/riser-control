#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <ecrt.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <riser_hardware/csv_logic.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace riser_hardware {
namespace {
constexpr uint32_t kOmronVendor = 0x00000083;
constexpr uint32_t kRotaryG5Product = 0x00000004; // R88D-KN04L-ECT
constexpr uint32_t kVariablePdoRevision = 0x00020001;
} // namespace

class BottomCsvSystem final : public hardware_interface::SystemInterface {
  ec_master_t *master_ = nullptr;
  ec_domain_t *domain_ = nullptr;
  ec_slave_config_t *slave_ = nullptr;
  uint8_t *data_ = nullptr;
  unsigned cw_ = 0, target_ = 0, mode_ = 0, sw_ = 0, pos_ = 0, vel_ = 0,
           display_ = 0;
  double position_ = NAN, velocity_ = NAN, command_ = NAN, ready_ = 0;
  double position_scale_ = 0, velocity_scale_ = 0, speed_limit_ = 0,
         travel_limit_ = 0;
  CsvGuard guard_;
  std::string command_key_;
  uint32_t cycle_ns_ = 1000000;
  bool active_ = false, claimed_ = false;
  bool commissioning_disabled_ = false;
  struct CommissioningStats {
    uint64_t op_cycles = 0, incomplete = 0, enabled = 0, faulted = 0;
    uint64_t op_exits = 0, preop_cycles = 0;
    uint64_t incomplete_wkc[4]{};
    uint64_t first_incomplete = 0, last_incomplete = 0, longest_burst = 0;
    uint64_t burst = 0;
    int32_t min_position = INT32_MAX, max_position = INT32_MIN;
    int64_t max_interval_ns = 0;
    bool was_operational = false;
  } stats_;
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
  void release() {
    if (master_)
      ecrt_release_master(master_);
    master_ = nullptr;
    domain_ = nullptr;
    slave_ = nullptr;
    data_ = nullptr;
  }
  void send(uint16_t word, int32_t target) {
    const int reference_result = ecrt_master_sync_reference_clock(master_);
    const int slave_result = ecrt_master_sync_slave_clocks(master_);
    if (reference_result || slave_result) {
      guard_.trip();
      word = kShutdownWord;
      target = 0;
    }
    EC_WRITE_U16(data_ + cw_, commissioning_disabled_ ? 0 : word);
    EC_WRITE_S32(data_ + target_, commissioning_disabled_ ? 0 : target);
    EC_WRITE_S8(data_ + mode_, 9);
    const int queue_result = ecrt_domain_queue(domain_);
    const int send_result = ecrt_master_send(master_);
    if (queue_result || send_result)
      guard_.trip();
  }

public:
  ~BottomCsvSystem() override { release(); }
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override {
    const auto &info = params.hardware_info;
    if (SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS ||
        info.joints.size() != 1)
      return hardware_interface::CallbackReturn::ERROR;
    const auto &j = info.joints.front();
    if (j.command_interfaces.size() != 1 ||
        j.command_interfaces[0].name != "velocity" ||
        j.state_interfaces.size() != 3 ||
        j.state_interfaces[0].name != "position" ||
        j.state_interfaces[1].name != "velocity" ||
        j.state_interfaces[2].name != "ready")
      return hardware_interface::CallbackReturn::ERROR;
    command_key_ = j.name + "/velocity";
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    const auto &name = info_.joints[0].name;
    std::vector<hardware_interface::StateInterface> result;
    result.emplace_back(name, "position", &position_);
    result.emplace_back(name, "velocity", &velocity_);
    result.emplace_back(name, "ready", &ready_);
    return result;
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    result.emplace_back(info_.joints[0].name, "velocity", &command_);
    return result;
  }
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    release();
    try {
      position_scale_ = number("position_units_per_rad");
      velocity_scale_ = number("velocity_units_per_rad_s");
      speed_limit_ = number("max_velocity_rad_s");
      travel_limit_ = number("max_travel_rad");
      cycle_ns_ = integer("cycle_ns");
      const auto shift = info_.hardware_parameters.count("sync0_shift_ns")
                             ? integer("sync0_shift_ns")
                             : 0;
      const auto disabled =
          info_.hardware_parameters.count("commissioning_disabled")
              ? integer("commissioning_disabled")
              : 0;
      const auto product = integer("product_code");
      const auto alias = integer("alias"), position = integer("position");
      if (!valid_scale(position_scale_) || !valid_scale(velocity_scale_) ||
          !valid_scale(speed_limit_) || !valid_scale(travel_limit_) ||
          product != kRotaryG5Product || alias > UINT16_MAX ||
          position > UINT16_MAX || cycle_ns_ < 250000 || cycle_ns_ > 4000000 ||
          shift >= cycle_ns_ || disabled > 1 ||
          !safe_velocity(speed_limit_, speed_limit_, velocity_scale_))
        throw std::invalid_argument("identity, scaling, timing or limits");
      commissioning_disabled_ = disabled != 0;
      master_ = ecrt_request_master(integer("master_index"));
      if (!master_)
        throw std::runtime_error("request master failed");
      // IgH's lookup uses the absolute ring position. A nonzero configured
      // alias anchors the position at the first slave with that effective
      // alias.
      uint32_t ring_position = position;
      ec_slave_info_t detected{};
      if (alias != 0) {
        uint32_t anchor = 0;
        for (; anchor <= UINT16_MAX; ++anchor) {
          if (ecrt_master_get_slave(master_, anchor, &detected))
            throw std::runtime_error("EtherCAT alias not found in bus scan");
          if (detected.alias == alias)
            break;
        }
        ring_position += anchor;
      }
      if (ring_position > UINT16_MAX ||
          ecrt_master_get_slave(master_, ring_position, &detected))
        throw std::runtime_error("EtherCAT slave not found in bus scan");
      if (detected.vendor_id != kOmronVendor ||
          detected.product_code != product ||
          detected.revision_number != kVariablePdoRevision) {
        RCLCPP_ERROR(rclcpp::get_logger("bottom_csv"),
                     "Slave %u: vendor 0x%08x, product 0x%08x, revision 0x%08x",
                     ring_position, detected.vendor_id, detected.product_code,
                     detected.revision_number);
        throw std::runtime_error(
            "CSV remap requires Omron R88D-KN04L-ECT revision 0x00020001");
      }
      domain_ = ecrt_master_create_domain(master_);
      slave_ = ecrt_master_slave_config(master_, alias, position, kOmronVendor,
                                        product);
      if (!domain_ || !slave_)
        throw std::runtime_error("domain/slave failed");
      // Custom CSV maps using the rotary ESI's variable mapping objects.
      ec_pdo_entry_info_t rx[] = {
          {0x6040, 0, 16}, {0x60ff, 0, 32}, {0x6060, 0, 8}};
      ec_pdo_entry_info_t tx[] = {
          {0x6041, 0, 16}, {0x6064, 0, 32}, {0x606c, 0, 32}, {0x6061, 0, 8}};
      ec_pdo_info_t pdos[] = {{0x1600, 3, rx}, {0x1a00, 4, tx}};
      ec_sync_info_t syncs[] = {
          {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
          {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
          {2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
          {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
          {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
      if (ecrt_slave_config_pdos(slave_, EC_END, syncs))
        throw std::runtime_error("PDO configuration failed");
      auto reg = [&](uint16_t index, unsigned &offset) {
        const int result =
            ecrt_slave_config_reg_pdo_entry(slave_, index, 0, domain_, nullptr);
        if (result < 0)
          throw std::runtime_error("PDO registration failed");
        offset = static_cast<unsigned>(result);
      };
      reg(0x6040, cw_);
      reg(0x60ff, target_);
      reg(0x6060, mode_);
      reg(0x6041, sw_);
      reg(0x6064, pos_);
      reg(0x606c, vel_);
      reg(0x6061, display_);
      if (ecrt_slave_config_dc(slave_, 0x0300, cycle_ns_, shift, 0, 0))
        throw std::runtime_error("DC configuration failed");
      if (ecrt_master_activate(master_) || !(data_ = ecrt_domain_data(domain_)))
        throw std::runtime_error("master activation failed");
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
    command_ = NAN;
    ready_ = 0;
    guard_.reset();
    stats_ = {};
    active_ = true;
    claimed_ = false;
    last_ = std::chrono::steady_clock::now();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    if (data_)
      send(kShutdownWord, 0);
    if (commissioning_disabled_ && active_)
      RCLCPP_INFO(
          rclcpp::get_logger("bottom_csv"),
          "Disabled PDO summary: OP=%" PRIu64 " incomplete=%" PRIu64
          " enabled=%" PRIu64 " faulted=%" PRIu64 " OP_exits=%" PRIu64
          " PREOP_tail=%" PRIu64 " position=[%d,%d] max_interval_ns=%" PRId64,
          stats_.op_cycles, stats_.incomplete, stats_.enabled, stats_.faulted,
          stats_.op_exits, stats_.preop_cycles, stats_.min_position,
          stats_.max_position, stats_.max_interval_ns);
    if (commissioning_disabled_ && active_)
      RCLCPP_INFO(
          rclcpp::get_logger("bottom_csv"),
          "Incomplete OP detail: WKC 0=%" PRIu64 " 1=%" PRIu64
          " 2=%" PRIu64 " other=%" PRIu64 " first=%" PRIu64
          " last=%" PRIu64 " longest_burst=%" PRIu64,
          stats_.incomplete_wkc[0], stats_.incomplete_wkc[1],
          stats_.incomplete_wkc[2], stats_.incomplete_wkc[3],
          stats_.first_incomplete, stats_.last_incomplete, stats_.longest_burst);
    active_ = false;
    claimed_ = false;
    command_ = NAN;
    ready_ = 0;
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
                              const std::vector<std::string> &) override {
    if (commissioning_disabled_ &&
        std::find(start.begin(), start.end(), command_key_) != start.end())
      return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    if (commissioning_disabled_ &&
        std::find(start.begin(), start.end(), command_key_) != start.end())
      return hardware_interface::return_type::ERROR;
    for (const auto &s : stop)
      if (s == command_key_) {
        claimed_ = false;
        command_ = NAN;
      }
    for (const auto &s : start)
      if (s == command_key_) {
        claimed_ = true;
        // The successfully activated controller must initialize its command.
        command_ = NAN;
      }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override {
    if (!active_)
      return hardware_interface::return_type::OK;
    const auto now = std::chrono::steady_clock::now();
    // Anchor DC application time before controller and broadcaster execution.
    const auto app_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    const int time_result =
        ecrt_master_application_time(master_, static_cast<uint64_t>(app_time));
    const auto interval = now - last_;
    const double dt = std::chrono::duration<double>(interval).count();
    last_ = now;
    const int receive_result = ecrt_master_receive(master_);
    const int process_result = ecrt_domain_process(domain_);
    ec_domain_state_t domain{};
    ec_slave_config_state_t slave{};
    const int domain_result = ecrt_domain_state(domain_, &domain);
    const int slave_result = ecrt_slave_config_state(slave_, &slave);
    const bool io_ok = !time_result && !receive_result && !process_result &&
                       !domain_result && !slave_result;
    if (!io_ok)
      guard_.trip();
    const bool healthy = io_ok && domain.wc_state == EC_WC_COMPLETE &&
                         slave.operational;
    if (healthy) {
      position_ = EC_READ_S32(data_ + pos_) / position_scale_;
      velocity_ = EC_READ_S32(data_ + vel_) / velocity_scale_;
    }
    const Feedback feedback{healthy, position_, velocity_,
                            EC_READ_U16(data_ + sw_),
                            EC_READ_S8(data_ + display_)};
    if (commissioning_disabled_) {
      if (slave.operational) {
        ++stats_.op_cycles;
        if (!healthy) {
          if (stats_.incomplete++ == 0)
            stats_.first_incomplete = stats_.op_cycles;
          stats_.last_incomplete = stats_.op_cycles;
          ++stats_.incomplete_wkc[std::min(domain.working_counter, 3U)];
          stats_.longest_burst = std::max(stats_.longest_burst, ++stats_.burst);
        } else {
          stats_.burst = 0;
        }
        stats_.max_interval_ns = std::max(
            stats_.max_interval_ns,
            std::chrono::duration_cast<std::chrono::nanoseconds>(interval)
                .count());
        if (healthy) {
          stats_.enabled += (feedback.status & 0x0004) != 0;
          stats_.faulted += (feedback.status & 0x0008) != 0;
          const auto position = EC_READ_S32(data_ + pos_);
          stats_.min_position = std::min(stats_.min_position, position);
          stats_.max_position = std::max(stats_.max_position, position);
        }
      } else if (stats_.was_operational) {
        ++stats_.op_exits;
        stats_.burst = 0;
      }
      if (stats_.op_cycles && slave.al_state == 2)
        ++stats_.preop_cycles;
      stats_.was_operational = slave.operational;
    }
    ready_ = guard_.observe(feedback, dt, cycle_ns_ * 1e-9, speed_limit_,
                            travel_limit_, claimed_);
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override {
    if (!active_)
      return hardware_interface::return_type::OK;
    if (claimed_ && !safe_velocity(command_, speed_limit_, velocity_scale_))
      guard_.trip();
    const bool enable = claimed_ && ready_ >= 1 && !guard_.fault();
    const auto status = EC_READ_U16(data_ + sw_);
    const uint16_t word = control_word(status, enable, guard_.fault());
    const int32_t target =
        enable && ready_ == 2
            ? static_cast<int32_t>(std::llround(command_ * velocity_scale_))
            : 0;
    send(word, target);
    return hardware_interface::return_type::OK;
  }
};
} // namespace riser_hardware
PLUGINLIB_EXPORT_CLASS(riser_hardware::BottomCsvSystem,
                       hardware_interface::SystemInterface)
