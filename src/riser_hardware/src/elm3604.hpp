#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <ecrt.h>
#include <hardware_interface/handle.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace riser_hardware {
struct Elm3604 {
  static constexpr uint32_t vendor = 2, coupler = 0x044c2c52;
  static constexpr uint32_t product = 0x50219349, revision = 0x00130002;
  static constexpr std::array<const char *, 6> fields{
      "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
  static constexpr std::array<unsigned, 3> channel_field{3, 2, 1};

  ec_domain_t *domain = nullptr;
  ec_slave_config_t *slave = nullptr;
  const uint8_t *data = nullptr;
  std::array<unsigned, 3> status{}, sample{};
  std::array<int, 3> previous_counter{-1, -1, -1};
  std::array<double, 3> uv_per_v_per_count{};
  std::array<std::array<double, 6>, 6> matrix{};
  std::array<double, 6> left{};
  double zero = 0, valid = 0, left_mask = 0b001110;

  bool configure(ec_master_t *master, unsigned position,
                 const std::string &coefficients,
                 const std::string &amplifier) {
    std::istringstream input(coefficients);
    for (auto &row : matrix)
      for (double &value : row)
        if (!(input >> value) || !std::isfinite(value))
          return false;
    std::istringstream gains(amplifier);
    for (double &scale : uv_per_v_per_count) {
      double gain_times_excitation;
      if (!(gains >> gain_times_excitation) ||
          !std::isfinite(gain_times_excitation) || gain_times_excitation <= 0)
        return false;
      scale = 1.28 / gain_times_excitation; // ±10 V Extended Range, µV/count.
    }

    domain = ecrt_master_create_domain(master);
    slave = ecrt_master_slave_config(master, 0, position, vendor, product);
    if (!domain || !slave ||
        ecrt_slave_config_dc(slave, 0x0700, 1000000, 0, 20000, 0))
      return false;

    // Keep the fixed SM3 maps. Read bipolar amplifier outputs with IEPE bias off.
    constexpr uint16_t bipolar_10v = 97;
    for (unsigned i = 0; i < channel_field.size(); ++i) {
      const uint16_t settings = 0x8000 + 16 * i;
      if (ecrt_slave_config_sdo8(slave, settings, 0x07, 0) ||
          ecrt_slave_config_sdo16(slave, settings, 0x01, bipolar_10v) ||
          ecrt_slave_config_sdo16(slave, settings, 0x03, 0) ||
          ecrt_slave_config_sdo16(slave, settings, 0x2e, 0))
        return false;
      const uint16_t object = 0x6000 + 16 * i;
      const int header =
          ecrt_slave_config_reg_pdo_entry(slave, object, 1, domain, nullptr);
      const int value = ecrt_slave_config_reg_pdo_entry(slave, object + 1, 1,
                                                        domain, nullptr);
      if (header < 0 || value != header + 4)
        return false;
      status[i] = header;
      sample[i] = value;
    }
    return true;
  }

  void clear() {
    left.fill(0);
    previous_counter.fill(-1);
    valid = 0;
  }

  void read(bool bus_ok) {
    ec_domain_state_t working_counter{};
    ec_slave_config_state_t state{};
    if (ecrt_domain_process(domain) || !bus_ok ||
        ecrt_domain_state(domain, &working_counter) ||
        working_counter.wc_state != EC_WC_COMPLETE ||
        ecrt_slave_config_state(slave, &state) || !state.operational) {
      clear();
      return;
    }
    bool fresh = true;
    std::array<double, 6> signals{};
    for (unsigned i = 0; i < channel_field.size(); ++i) {
      const uint16_t header = EC_READ_U16(data + status[i]);
      const int counter = header >> 14;
      // Bit 12 announces a new diagnostic message, independently of PDO validity.
      fresh &= previous_counter[i] >= 0 && counter != previous_counter[i] &&
               (header & 0xff) == 1 && !(header & 0x2700);
      previous_counter[i] = counter;
      signals[channel_field[i]] =
          EC_READ_S32(data + sample[i]) * uv_per_v_per_count[i];
    }
    for (unsigned row = 0; row < left.size(); ++row) {
      left[row] = 0;
      for (unsigned col = 0; col < signals.size(); ++col)
        left[row] += matrix[row][col] * signals[col];
    }
    valid = fresh;
  }

  void export_states(std::vector<hardware_interface::StateInterface> &states) {
    for (const std::string side : {"left_force_plate", "right_force_plate"}) {
      const bool is_left = side == "left_force_plate";
      for (unsigned i = 0; i < fields.size(); ++i)
        states.emplace_back(side, fields[i], is_left ? &left[i] : &zero);
      states.emplace_back(side, "live_mask", is_left ? &left_mask : &zero);
      states.emplace_back(side, "valid", &valid);
    }
  }
};
} // namespace riser_hardware
