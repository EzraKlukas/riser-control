#!/usr/bin/env bash
set -euo pipefail

# Run in the same sourced ROS environment/domain as the live controller manager.
ec=${ETHERCAT_CLI:-/opt/etherlab/bin/ethercat}
stop_profile=false
if [[ ${1:-} == --stop-profile ]]; then
  stop_profile=true
  shift
fi
master_index=${1:-0}
position=${2:-0}
serial=${3:-0}
if (( $# > 3 )) || [[ ! $master_index =~ ^[0-9]+$ || ! $position =~ ^[0-9]+$ ||
                        ! $serial =~ ^[0-9]+$ ]]; then
  echo "Usage: bash shutdown_bottom_motor.sh [--stop-profile] [master_index] [ring_position] [decimal_serial]" >&2
  exit 2
fi

bus=("$ec" -m "$master_index")
slave=("${bus[@]}" -p "$position")
identity=$("${slave[@]}" slaves -v)
if ! grep -Fxq "=== Master $master_index, Slave $position ===" <<<"$identity" ||
   ! grep -Eq 'Product code:[[:space:]]+0x00000004$' <<<"$identity" ||
   ! grep -Eq 'Vendor Id:[[:space:]]+0x00000083$' <<<"$identity" ||
   ! grep -Eq 'Revision number:[[:space:]]+0x00020001$' <<<"$identity"; then
  echo "Expected the Omron G5 at the selected position." >&2
  exit 1
fi
if (( serial )); then
  actual_serial=$(awk '/Serial number:/ {print $NF}' <<<"$identity")
  if [[ ! $actual_serial =~ ^0x[0-9a-fA-F]+$ ]] || (( actual_serial != serial )); then
    echo "Selected drive serial does not match the commissioned bottom axis." >&2
    exit 1
  fi
elif [[ $position != 0 || $("${bus[@]}" slaves | wc -l) != 1 ]]; then
  echo "Supply the commissioned drive serial for a shared ring." >&2
  exit 1
fi

ros_control() { timeout 10s ros2 control "$@"; }
if $stop_profile; then
  ros_control set_controller_state motor_profile inactive
fi
interfaces=$(ros_control list_hardware_interfaces)
if [[ $(awk '/_motor\/velocity.*\[/ {count++} END {print count + 0}' <<<"$interfaces") != 1 ]] ||
   ! grep -Eq 'bottom_motor/velocity.*\[unclaimed\]' <<<"$interfaces"; then
  echo "Release the bottom motor velocity interface before requesting PREOP." >&2
  exit 1
fi

# Do not deactivate hardware until it has serviced the PREOP transition.
"${slave[@]}" states PREOP
consecutive=0
for ((attempt=0; attempt<100; ++attempt)); do
  state=$("${slave[@]}" slaves | awk 'NR == 1 { print $3 }')
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
master=$("${bus[@]}" master)
grep -q 'Active: no' <<<"$master"
"${slave[@]}" slaves
echo "Drive error (603F), detail (4001), statusword (6041):"
error=$("${slave[@]}" upload -t uint16 0x603f 0)
detail=$("${slave[@]}" upload -t uint16 0x4001 0)
status=$("${slave[@]}" upload -t uint16 0x6041 0)
printf '%s\n' "$error" "$detail" "$status"
[[ ${error%% *} == 0x0000 && ${detail%% *} == 0x0000 ]]
