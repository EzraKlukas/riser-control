#pragma once

#include <array>
#include <cmath>
#include <ecrt.h>
#include <hardware_interface/handle.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace riser_hardware {
class Elm3604 {
  struct Channel {
    unsigned status_offset = 0, sample_offset = 0;
    int counter = -1;
    double raw = NAN, status = NAN, value = NAN;
  };
  std::array<Channel, 3> channels_{};
  std::array<double, 3> scale_{}, zero_{};
  ec_slave_config_t *slave_ = nullptr;
  ec_domain_t *domain_ = nullptr;
  uint8_t *data_ = nullptr;
  double valid_ = 0;

  static void check(int result, const char *operation) {
    if (result < 0)
      throw std::runtime_error(std::string("ELM3604: ") + operation);
  }

public:
  static constexpr unsigned vendor = 2;
  static constexpr unsigned product = 0x50219349;
  static constexpr unsigned revision = 0x00130002;
  static constexpr unsigned coupler_product = 0x044c2c52;
  static constexpr std::array<const char *, 10> states{
      "torque.x", "force.z", "force.x",   "valid",     "x1_raw",
      "x2_raw",   "x3_raw",  "x1_status", "x2_status", "x3_status"};
  std::string name;

  void configure(ec_master_t *master, unsigned position,
                 const std::array<double, 3> &scale,
                 const std::array<double, 3> &zero) {
    scale_ = scale;
    zero_ = zero;
    slave_ = ecrt_master_slave_config(master, 0, position, vendor, product);
    domain_ = ecrt_master_create_domain(master);
    if (!slave_ || !domain_)
      throw std::runtime_error("Cannot allocate ELM3604 configuration/domain");

    // Fixed Beckhoff maps. Change the SM3 assignment without rewriting entries.
    check(ecrt_slave_config_pdo_assign_clear(slave_, 3),
          "clear SM3 assignment");
    constexpr std::array<unsigned, 3> status_pdo{0x1a00, 0x1a21, 0x1a42};
    for (unsigned i = 0; i < channels_.size(); ++i) {
      const uint16_t settings = 0x8000 + 16 * i;
      // Reference electrical mode: 0–10 V DC, IEPE off, unfiltered.
      check(ecrt_slave_config_sdo8(slave_, settings, 0x07, 0), "disable IEPE");
      for (const auto &[subindex, value] :
           std::array<std::pair<uint8_t, uint16_t>, 6>{{{0x01, 108},
                                                        {0x03, 0},
                                                        {0x16, 0},
                                                        {0x18, 1},
                                                        {0x19, 0},
                                                        {0x2e, 0}}})
        check(ecrt_slave_config_sdo16(slave_, settings, subindex, value),
              "queue analog channel settings");
      check(ecrt_slave_config_pdo_assign_add(slave_, 3, status_pdo[i]),
            "assign status PDO");
      check(ecrt_slave_config_pdo_assign_add(slave_, 3, status_pdo[i] + 1),
            "assign sample PDO");
    }
    // The first registration sizes IgH's FMMU for the whole sync manager.
    for (unsigned i = 0; i < channels_.size(); ++i) {
      const uint16_t object = 0x6000 + 16 * i;
      const int status =
          ecrt_slave_config_reg_pdo_entry(slave_, object, 1, domain_, nullptr);
      const int sample = ecrt_slave_config_reg_pdo_entry(slave_, object + 1, 1,
                                                         domain_, nullptr);
      check(status, "register status PDO");
      check(sample, "register sample PDO");
      channels_[i].status_offset = status;
      channels_[i].sample_offset = sample;
    }
    check(ecrt_slave_config_dc(slave_, 0x0700, 1000000, 0, 20000, 0),
          "configure distributed clocks");
  }

  void bind() {
    data_ = ecrt_domain_data(domain_);
    if (!data_)
      throw std::runtime_error("ELM3604 has no process data");
  }

  void invalidate() {
    valid_ = 0;
    for (auto &channel : channels_) {
      channel.counter = -1;
      channel.raw = channel.status = channel.value = NAN;
    }
  }

  void release() {
    invalidate();
    slave_ = nullptr;
    domain_ = nullptr;
    data_ = nullptr;
  }

  void read(bool bus_ok) {
    ec_domain_state_t domain{};
    ec_slave_config_state_t slave{};
    if (ecrt_domain_process(domain_) || ecrt_domain_state(domain_, &domain) ||
        ecrt_slave_config_state(slave_, &slave) || !bus_ok ||
        domain.wc_state != EC_WC_COMPLETE || !slave.operational) {
      invalidate();
      return;
    }
    bool good = true;
    for (unsigned i = 0; i < channels_.size(); ++i) {
      auto &channel = channels_[i];
      const uint8_t *status = data_ + channel.status_offset;
      const uint8_t flags = EC_READ_U8(status + 1);
      const int counter = flags >> 6;
      const bool advancing =
          channel.counter != -1 && counter != channel.counter;
      channel.counter = counter;
      channel.status = EC_READ_U16(status);
      channel.raw = EC_READ_S32(data_ + channel.sample_offset);
      // Samples, Error, Under/Overrange, Diag and TxPDO State.
      const bool channel_ok =
          advancing && EC_READ_U8(status) && !(flags & 0x37);
      channel.value = channel_ok && scale_[i] != 0
                          ? (channel.raw - zero_[i]) * scale_[i]
                          : NAN;
      good &= std::isfinite(channel.value);
    }
    valid_ = good;
  }

  void queue() {
    if (ecrt_domain_queue(domain_))
      invalidate();
  }

  void export_states(std::vector<hardware_interface::StateInterface> &result) {
    for (unsigned i = 0; i < channels_.size(); ++i) {
      result.emplace_back(name, states[i], &channels_[i].value);
      result.emplace_back(name, states[4 + i], &channels_[i].raw);
      result.emplace_back(name, states[7 + i], &channels_[i].status);
    }
    result.emplace_back(name, "valid", &valid_);
  }
};
} // namespace riser_hardware
