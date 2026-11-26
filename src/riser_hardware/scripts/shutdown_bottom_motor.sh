#!/usr/bin/env bash
set -euo pipefail

# Bottom-axis setup: master 0, G5 at ring position 0 and ROS BottomMotor.
# Run in the same sourced ROS environment/domain as the live controller manager.
ec=${ETHERCAT_CLI:-/opt/etherlab/bin/ethercat}
if (( $# > 1 )) || [[ ${1:-} != "" && ${1:-} != --stop-profile ]]; then
  echo "Usage: bash shutdown_bottom_motor.sh [--stop-profile]" >&2
  exit 2
fi

g5=("$ec" -m 0 -p 0)
slave=$("${g5[@]}" slaves -v)
if ! grep -Fxq '=== Master 0, Slave 0 ===' <<<"$slave" ||
   ! grep -Eq 'Product code:[[:space:]]+0x00000004' <<<"$slave" ||
   ! grep -Eq 'Vendor Id:[[:space:]]+0x00000083' <<<"$slave" ||
   ! grep -Eq 'Revision number:[[:space:]]+0x00020001' <<<"$slave"; then
  echo "Expected the bottom Omron G5 at master 0, position 0." >&2
  exit 1
fi

ros_control() { timeout 10s ros2 control "$@"; }
if [[ ${1:-} == --stop-profile ]]; then
  ros_control set_controller_state motor_profile inactive
fi
interfaces=$(ros_control list_hardware_interfaces)
if ! grep -Eq 'bottom_motor/velocity.*\[unclaimed\]' <<<"$interfaces" ||
   grep -Eq '_motor/velocity.*\[claimed\]' <<<"$interfaces"; then
  echo "Release all motor velocity interfaces before requesting PREOP." >&2
  exit 1
fi

# Do not deactivate hardware until it has serviced the PREOP transition.
"${g5[@]}" states PREOP
consecutive=0
for ((attempt=0; attempt<100; ++attempt)); do
  state=$("${g5[@]}" slaves | awk 'NR == 1 { print $3 }')
  if [[ $state == PREOP ]]; then
    consecutive=$((consecutive + 1))
    if (( consecutive >= 4 )); then break; fi
  else
    consecutive=0
  fi
  sleep 0.02
done
if (( consecutive < 4 )); then
  echo "PREOP did not settle. The ROS hardware loop was left running." >&2
  exit 1
fi

ros_control set_hardware_component_state BottomMotor inactive
ros_control set_hardware_component_state BottomMotor unconfigured
master=$("$ec" -m 0 master)
grep -q 'Active: no' <<<"$master"
"${g5[@]}" slaves
echo "Drive error (603F), detail (4001), statusword (6041):"
error=$("${g5[@]}" upload -t uint16 0x603f 0)
detail=$("${g5[@]}" upload -t uint16 0x4001 0)
status=$("${g5[@]}" upload -t uint16 0x6041 0)
printf '%s\n' "$error" "$detail" "$status"
[[ ${error%% *} == 0x0000 && ${detail%% *} == 0x0000 ]]
