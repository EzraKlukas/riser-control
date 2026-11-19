#include "riser_human_balance/ankle_balance.hpp"

int main() {
  using namespace riser_human_balance;
  if (ankle_torque_nm({0., 0.}, 1., 18.) != 0. ||
      ankle_torque_nm({0., 0.}, 0.2, 18.) != 18. ||
      ankle_torque_nm({0.01, 0.}, 1., 0.) >= 0. ||
      ankle_torque_nm({0., -0.01}, 1., 0.) <= 0. ||
      ankle_torque_nm({1., 0.}, 1., 0.) != -120.)
    return 1;

  if (strap_torque_nm({0.01, 0.02}, {0.01, 0.02}) != 0. ||
      strap_torque_nm({0.02, 0.02}, {0.01, 0.02}) >= 0. ||
      strap_torque_nm({0.01, -0.02}, {0.01, 0.02}) <= 0.)
    return 1;
}
