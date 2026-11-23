#!/usr/bin/env bash
set -euo pipefail

# Dedicated bottom-axis setup: master 0, one G5 at ring position 0 and ROS BottomMotor.
# Run in the same sourced ROS environment/domain as the live controller manager.
ec=${ETHERCAT_CLI:-/opt/etherlab/bin/ethercat}
if (( $# > 1 )) || [[ ${1:-} != "" && ${1:-} != --stop-profile ]]; then
  echo "Usage: bash shutdown_bottom_motor.sh [--stop-profile]" >&2
  exit 2
fi

slave=$("$ec" slaves -v)
if [[ $(awk '/^=== Master / { count++ } END { print count+0 }' <<<"$slave") != 1 ]] ||
   ! grep -Fxq '=== Master 0, Slave 0 ===' <<<"$slave" ||
   ! grep -Eq 'Product code:[[:space:]]+0x00000004' <<<"$slave" ||
   ! grep -Eq 'Vendor Id:[[:space:]]+0x00000083' <<<"$slave" ||
   ! grep -Eq 'Revision number:[[:space:]]+0x00020001' <<<"$slave"; then
  echo "Expected the dedicated one-slave Omron G5 master." >&2
  exit 1
fi

ros_control() { timeout 10s ros2 control "$@"; }
if [[ ${1:-} == --stop-profile ]]; then
  ros_control set_controller_state motor_profile inactive
fi
interfaces=$(ros_control list_hardware_interfaces)
if ! grep -Eq 'bottom_motor/velocity.*\[unclaimed\]' <<<"$interfaces"; then
  echo "Release the bottom motor velocity interface before requesting PREOP." >&2
  exit 1
fi

# Do not deactivate hardware until it has serviced the PREOP transition.
"$ec" states -p 0 PREOP
consecutive=0
for ((attempt=0; attempt<100; ++attempt)); do
  state=$("$ec" slaves -p 0 | awk 'NR == 1 { print $3 }')
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
master=$("$ec" master)
grep -q 'Active: no' <<<"$master"
"$ec" slaves -p 0
echo "Drive error (603F), detail (4001), statusword (6041):"
error=$("$ec" upload -p 0 -t uint16 0x603f 0)
detail=$("$ec" upload -p 0 -t uint16 0x4001 0)
status=$("$ec" upload -p 0 -t uint16 0x6041 0)
printf '%s\n' "$error" "$detail" "$status"
[[ ${error%% *} == 0x0000 && ${detail%% *} == 0x0000 ]]
