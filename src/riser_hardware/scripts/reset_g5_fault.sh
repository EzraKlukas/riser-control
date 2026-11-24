#!/usr/bin/env bash
set -euo pipefail

ethercat=${ETHERCAT_CLI:-/opt/etherlab/bin/ethercat}
master=${1:-0}
position=${2:-0}
serial=${3:-0}
if [[ ! $serial =~ ^[0-9]+$ ]]; then echo "Expected a decimal drive serial" >&2; exit 2; fi
if (( $# > 3 )) || [[ ! $master =~ ^[0-9]+$ || ! $position =~ ^[0-9]+$ ]]; then
  echo "Usage: bash reset_g5_fault.sh [master_index] [ring_position] [serial]" >&2
  exit 2
fi

bus=("$ethercat" -m "$master")
slave=("${bus[@]}" -p "$position")
read_object() {
  local result
  result=$("${slave[@]}" upload -t "$1" "$2" "$3") || return
  printf '%s\n' "${result%% *}"
}
write_controlword() {
  "${slave[@]}" download -t uint16 0x6040 0 "$1"
}

# Run with the cyclic application stopped and exclusive control of this master.
master_state=$("${bus[@]}" master)
slave_state=$("${slave[@]}" slaves)
if ! grep -q 'Phase: Idle' <<<"$master_state" ||
   ! grep -q 'Active: no' <<<"$master_state" ||
   ! grep -q '[[:space:]]PREOP[[:space:]]' <<<"$slave_state"; then
  echo "Fault reset requires an idle master and the selected slave in PREOP." >&2
  exit 1
fi
if [[ $(read_object uint32 0x1018 1) != 0x00000083 ||
      $(read_object uint32 0x1018 2) != 0x00000004 ||
      $(read_object uint32 0x1018 3) != 0x00020001 ]]; then
  echo "Expected Omron R88D-KN04L-ECT revision 0x00020001." >&2
  exit 1
fi

if (( serial )); then
  actual_serial=$(read_object uint32 0x1018 4)
  (( actual_serial == serial )) || { echo "Drive serial mismatch" >&2; exit 1; }
elif [[ $position != 0 || $("${bus[@]}" slaves | wc -l) != 1 ]]; then
  echo "Supply the commissioned drive serial for a multi-drive ring" >&2
  exit 1
fi
status=$(read_object uint16 0x6041 0)
if (( (status & 0x0004) != 0 )); then
  echo "Refusing reset while operation is enabled." >&2
  exit 1
fi
if (( (status & 0x0008) == 0 )); then
  echo "No CiA 402 fault latched."
  exit 0
fi
if (( (status & 0x004f) != 0x0008 )); then
  echo "Wait for the drive to finish its fault reaction before resetting." >&2
  exit 1
fi

before=$(read_object uint16 0x603f 0)
detail=$(read_object uint16 0x4001 0)
printf 'Before reset: error %s detail %s status %s\n' "$before" "$detail" "$status"
write_controlword 0
trap 'write_controlword 0' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Bit 7 only. Voltage, switch-on, and operation-enable bits stay clear.
write_controlword 0x0080
sleep 0.05
write_controlword 0
trap - EXIT INT TERM

after=$(read_object uint16 0x603f 0)
status=$(read_object uint16 0x6041 0)
printf 'Drive error %s -> %s, statusword %s\n' "$before" "$after" "$status"
if (( (status & 0x000c) != 0 )); then
  echo "The drive did not return to the disabled, fault-free state." >&2
  exit 1
fi
