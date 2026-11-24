#include "../src/g5_axis.hpp"
#include <array>

int main() {
  using namespace riser_hardware;
  std::array<uint8_t, 128> domain_bytes{};
  for (unsigned base : {0U, 32U, 64U, 96U}) {
    G5Axis a;
    a.cw_ = base;
    a.target_ = base + 2;
    a.mode_ = base + 6;
    a.sw_ = base + 7;
    a.pos_ = base + 9;
    a.vel_ = base + 13;
    a.display_ = base + 17;
    a.dio_ = base + 18;
    a.torque_offset = base + 22;
    a.fault_offset = base + 24;
    a.rated_torque = 1.3;
    auto *data = domain_bytes.data();
    EC_WRITE_U16(data + a.sw_, 0x27);
    EC_WRITE_S8(data + a.display_, 9);
    EC_WRITE_S32(data + a.pos_, -100);
    EC_WRITE_S16(data + a.torque_offset, -500);
    ec_slave_config_state_t slave{};
    slave.operational = 1;
    ec_domain_state_t domain{};
    domain.wc_state = EC_WC_COMPLETE;
    auto read = [&](bool healthy = true) {
      a.read(data, healthy, slave, domain, std::chrono::milliseconds(1), .001,
             1000000, false, 100, 100, 1, 2);
    };
    a.claimed_ = true;
    for (uint32_t mask = 0; mask < 4; ++mask) {
      EC_WRITE_U32(data + a.dio_, mask | (1u << 26));
      read();
      if (!a.feedback_valid || a.position_ != -1 || a.torque_nm != -.65 ||
          a.digital_inputs != (mask | (1u << 26)) ||
          a.reverse_limit_ != (mask & 1) ||
          a.forward_limit_ != ((mask >> 1) & 1) || !std::isnan(a.unknown_brake))
        return 1;
      for (int sign : {-1, 1}) {
        a.command_ = 0;
        a.stage(data, false, false, 1, 100);
        a.command_ = sign * .5;
        a.stage(data, false, false, 1, 100);
        const auto expected = mask & (sign < 0 ? 1 : 2) ? 0 : sign * 50;
        if (EC_READ_S32(data + a.target_) != expected ||
            EC_READ_U16(data + a.cw_) != 15)
          return 2;
      }
    }
    a.stage(data, true, false, 1, 100);
    if (EC_READ_U16(data + a.cw_) || EC_READ_S32(data + a.target_))
      return 3;
    EC_WRITE_U16(data + a.sw_, 8);
    EC_WRITE_U16(data + a.fault_offset, 0x2310);
    read();
    if (!a.host_fault || a.fault_code != 0x2310 || a.ready_ != 0)
      return 4;
    read(false);
    a.stage(data, false, false, 1, 100);
    if (a.feedback_valid || EC_READ_U16(data + a.cw_) != kShutdownWord ||
        EC_READ_S32(data + a.target_))
      return 5;
  }
}
