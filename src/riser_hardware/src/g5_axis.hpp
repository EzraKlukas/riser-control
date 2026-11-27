#pragma once
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <ecrt.h>
#include <hardware_interface/handle.hpp>
#include <rclcpp/logging.hpp>
#include <riser_hardware/csv_logic.hpp>
#include <string>
#include <vector>

namespace riser_hardware {
struct G5Axis {
  ec_slave_config_t *slave_ = nullptr;
  std::string name, key, midpoint_key;
  double rated_torque = NAN;
  unsigned torque_offset{}, fault_offset{};
  unsigned cw_ = 0, target_ = 0, mode_ = 0, dio_ = 0, sw_ = 0, pos_ = 0,
           vel_ = 0, display_ = 0;
  double position_ = NAN, velocity_ = NAN, command_ = NAN, ready_ = 0,
         reverse_limit_ = NAN, forward_limit_ = NAN;
  double calibration_midpoint = NAN, calibration_half_travel = 0;
  bool calibrating = false;
  double digital_inputs = NAN, feedback_valid = 0, statusword = NAN,
         controlword = NAN, operating_mode = NAN, fault_code = NAN,
         torque_raw = NAN, torque_nm = NAN, brake_request = NAN,
         unknown_output = NAN, unknown_brake = NAN, discovered = 0,
         host_fault = 0;
  CsvGuard guard_;
  bool claimed_ = false, limit_blocked = false;
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

  void configure_pdos(ec_domain_t *domain_, uint32_t cycle_ns, uint32_t shift) {
    // Custom CSV maps using the rotary ESI's variable mapping objects.
    ec_pdo_entry_info_t rx[] = {
        {0x6040, 0, 16}, {0x60ff, 0, 32}, {0x6060, 0, 8}};
    ec_pdo_entry_info_t tx[] = {
        {0x6041, 0, 16}, {0x6064, 0, 32}, {0x606c, 0, 32}, {0x6061, 0, 8},
        {0x60fd, 0, 32}, {0x6077, 0, 16}, {0x603f, 0, 16}};
    ec_pdo_info_t pdos[] = {{0x1600, 3, rx}, {0x1a00, 7, tx}};
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
    reg(0x6077, torque_offset);
    reg(0x603f, fault_offset);
    reg(0x6040, cw_);
    reg(0x60ff, target_);
    reg(0x6060, mode_);
    reg(0x60fd, dio_);
    reg(0x6041, sw_);
    reg(0x6064, pos_);
    reg(0x606c, vel_);
    reg(0x6061, display_);
    if (ecrt_slave_config_dc(slave_, 0x0300, cycle_ns, shift, 0, 0))
      throw std::runtime_error("DC configuration failed");
  }
  void read(uint8_t *data_, bool io_ok, const ec_slave_config_state_t &slave,
            const ec_domain_state_t &domain,
            std::chrono::steady_clock::duration interval, double dt,
            uint32_t cycle_ns_, bool commissioning_disabled_,
            double position_scale_, double velocity_scale_,
            double speed_limit_) {
    if (!io_ok)
      guard_.trip();
    const bool healthy =
        io_ok && domain.wc_state == EC_WC_COMPLETE && slave.operational;
    feedback_valid = healthy;
    if (healthy) {
      position_ = EC_READ_S32(data_ + pos_) / position_scale_;
      velocity_ = EC_READ_S32(data_ + vel_) / velocity_scale_;
      const uint32_t inputs = EC_READ_U32(data_ + dio_);
      reverse_limit_ = (inputs & (1u << 0)) != 0; // NOT, CN1 pin 8
      forward_limit_ = (inputs & (1u << 1)) != 0; // POT, CN1 pin 7
      digital_inputs = inputs;
      torque_raw = EC_READ_S16(data_ + torque_offset);
      torque_nm = torque_raw * rated_torque / 1000.;
      fault_code = EC_READ_U16(data_ + fault_offset);
      statusword = EC_READ_U16(data_ + sw_);
      operating_mode = EC_READ_S8(data_ + display_);
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
    ready_ =
        guard_.observe(feedback, dt, cycle_ns_ * 1e-9, speed_limit_, claimed_);
    host_fault = guard_.reason();
  }

  void stage(uint8_t *data_, bool disabled, bool stopping, double speed_limit,
             double scale) {
    if (claimed_ && !safe_velocity(command_, speed_limit, scale))
      guard_.trip();
    const bool enabled =
        !disabled && !stopping && claimed_ && ready_ >= 1 && !guard_.fault();
    const uint16_t word = disabled ? 0
                                   : control_word(EC_READ_U16(data_ + sw_),
                                                  enabled, guard_.fault());
    if (command_ == 0)
      limit_blocked = false;
    if ((command_ > 0 && forward_limit_ != 0) ||
        (command_ < 0 && reverse_limit_ != 0))
      limit_blocked = true;
    const int32_t target =
        enabled && ready_ == 2 && !limit_blocked
            ? static_cast<int32_t>(std::llround(command_ * scale))
            : 0;
    EC_WRITE_U16(data_ + cw_, word);
    EC_WRITE_S32(data_ + target_, target);
    EC_WRITE_S8(data_ + mode_, 9);
  }
  void export_states(std::vector<hardware_interface::StateInterface> &result) {
    result.emplace_back(name, "position", &position_);
    result.emplace_back(name, "velocity", &velocity_);
    result.emplace_back(name, "ready", &ready_);
    result.emplace_back(name, "reverse_limit", &reverse_limit_);
    result.emplace_back(name, "forward_limit", &forward_limit_);
    result.emplace_back(name, "digital_inputs", &digital_inputs);
    result.emplace_back(name, "feedback_valid", &feedback_valid);
    result.emplace_back(name, "statusword", &statusword);
    result.emplace_back(name, "controlword", &controlword);
    result.emplace_back(name, "operating_mode", &operating_mode);
    result.emplace_back(name, "fault_code", &fault_code);
    result.emplace_back(name, "torque_raw", &torque_raw);
    result.emplace_back(name, "torque_nm", &torque_nm);
    result.emplace_back(name, "brake_release_request", &brake_request);
    result.emplace_back(name, "brake_output_feedback", &unknown_output);
    result.emplace_back(name, "physical_brake", &unknown_brake);
    result.emplace_back(name, "discovered", &discovered);
    result.emplace_back(name, "host_fault", &host_fault);
  }
  void summary() {
    RCLCPP_INFO(
        rclcpp::get_logger("bottom_csv"),
        "Disabled PDO summary: OP=%" PRIu64 " incomplete=%" PRIu64
        " enabled=%" PRIu64 " faulted=%" PRIu64 " OP_exits=%" PRIu64
        " PREOP_tail=%" PRIu64 " position=[%d,%d] max_interval_ns=%" PRId64,
        stats_.op_cycles, stats_.incomplete, stats_.enabled, stats_.faulted,
        stats_.op_exits, stats_.preop_cycles, stats_.min_position,
        stats_.max_position, stats_.max_interval_ns);
    RCLCPP_INFO(rclcpp::get_logger("bottom_csv"),
                "Incomplete OP detail: WKC 0=%" PRIu64 " 1=%" PRIu64
                " 2=%" PRIu64 " other=%" PRIu64 " first=%" PRIu64
                " last=%" PRIu64 " longest_burst=%" PRIu64,
                stats_.incomplete_wkc[0], stats_.incomplete_wkc[1],
                stats_.incomplete_wkc[2], stats_.incomplete_wkc[3],
                stats_.first_incomplete, stats_.last_incomplete,
                stats_.longest_burst);
  }
};
} // namespace riser_hardware
