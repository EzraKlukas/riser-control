# riserms bottom-axis EtherCAT commissioning

For current drive settings, motion-inhibited first contact, and the powered-test prerequisites, use the [G5 motion bringup checklist](g5-motion-bringup.md).


Current host maintenance and first contact are documented in
[riserms realtime and EtherCAT setup](riserms-realtime-setup.md).
This report records the earlier integration and generic-kernel baseline.

## Provenance and checkpoint

2026-11-20 UTC. riserms is a Minisforum MS-01 with an Intel Core
i9-13900H, 20 logical CPUs, Ubuntu 24.04.5 LTS x86_64, and Secure Boot
enabled and a LUKS-encrypted root. The drive is disconnected and all test
launches are stopped. The operator plans to attach Ubuntu Pro at the next
session. Running RT verification, MOK enrollment, native module build and
installation, fieldbus bind, and the fresh SSH and persistence checks are
pending. Both NICs still use Linux `igc`. Host evidence is archived in
`~/riserms-commissioning-20261120/` for the reboot.

- RISER main base: `86219c3c29542bfec1d7bbda26c62f452b1edec7`.
  Integration is in the current working tree.
- Jetson reference: read-only `~/riser-ethercat-jetson` archive of
  `feature/humble-ethercat-csv` based on
  `62e0cbc8d6318850ac50dbf97cc458e839a0913a`, with the archived EtherCAT
  changes uncommitted. The handoff, bringup logbook, tutorial, captured clock
  walkthrough, Sysmac HTML snapshot and rotary ESI are in that archive.
  Sysmac snapshot SHA-256
  `f9b9ba7e77933b4c196b5f13a89ab1ffce8dfae20495e67926870fbea03392aa`.
  The raw log directory `~/riser-humble-ethercat/evidence/g5-dc-startup-20261118/`
  and the earlier Jetson `/tmp` logs are on the Jetson.
- Stock IgH 1.6.13 tag object `8fd340e5c3dea8120c84256439813b27bf42c13d`,
  commit `61cc654f5b721ddd54df0f58bdd34106d91c5359`. Pristine source reviewed
  at `/tmp/riser-igh-1.6.13`. The Jetson patch targets IgH 1.5.3 commit
  `6cdb21c6f76980d65334a14dd224f8e090863424`.
- Initial host kernel and headers: `6.8.0-151-generic` package
  `6.8.0-151.151`, `PREEMPT_DYNAMIC`. This is the known-good fallback.
  The initial GRUB menu was hidden with a zero-second timeout.

| Function | PCI | Controller | Permanent MAC | Linux interface |
| --- | --- | --- | --- | --- |
| Fieldbus | `0000:57:00.0` | I226-V `8086:125c` | `38:05:25:37:9f:32` | `enp87s0` |
| SSH | `0000:59:00.0` | I226-LM `8086:125b` | `38:05:25:37:9f:33` | `enp89s0` |

Both initially used `igc`. The live SSH route from the host address
`10.34.191.186` to client `10.34.191.172` selected `enp89s0`.
Before binding, recheck with `ip route get "${SSH_CONNECTION%% *}"` and
`ethtool -P` on each port.

## Console and Secure Boot checkpoint

Ubuntu Pro was initially unattached. Use Canonical's generic GA realtime
stack. The Noble public archive also advertised a signed
`linux-image-6.8.1-1015-realtime` and corresponding headers at package version
`6.8.1-1015.16`. The extracted headers match that release and have
`CONFIG_PREEMPT_RT=y` and `CONFIG_HZ=1000`. Record the actual packages
selected by Pro before the reboot. [Canonical realtime
installation](https://documentation.ubuntu.com/pro-client/en/latest/howtoguides/enable_realtime_kernel/)

```bash
sudo apt update
sudo apt install ubuntu-pro-client
sudo pro attach
sudo pro enable realtime-kernel
sudo apt-mark manual linux-image-6.8.0-151-generic linux-modules-6.8.0-151-generic linux-modules-extra-6.8.0-151-generic
dpkg-query -W 'linux-image-*' 'linux-headers-*' | grep -E '6\.8|realtime'
ls -lh /boot/{vmlinuz,initrd.img}-*
sudo grep -E 'submenu |menuentry ' /boot/grub/grub.cfg
```

Make the GRUB menu visible and preserve the generic kernel as the default
using the menu IDs printed above. The operator can select the GA realtime
entry for the first boot and unlock LUKS at the console.

```bash
sudo tee /etc/default/grub.d/90-riser-console.cfg >/dev/null <<'GRUB'
GRUB_TIMEOUT_STYLE=menu
GRUB_TIMEOUT=10
GRUB_DEFAULT=saved
GRUB
sudo update-grub
sudo grep -E 'submenu |menuentry ' /boot/grub/grub.cfg
# Substitute the exact submenu and generic kernel entry IDs from the output.
advanced_id='<advanced-submenu-id>'
fallback_id='<generic-6.8.0-151-entry-id>'
sudo grub-set-default "$advanced_id>$fallback_id"
sudo grub-editenv list
```

With Secure Boot enabled, enroll the existing Ubuntu module signing key or
create one before reboot. Check the pending enrollment and complete it in
MokManager using the locally chosen one-time password. Keep the private key
with root.

```bash
mokutil --sb-state
if ! sudo test -f /var/lib/shim-signed/mok/MOK.der; then
  sudo update-secureboot-policy --new-key
fi
sudo mokutil --test-key /var/lib/shim-signed/mok/MOK.der
# If the key is not already enrolled:
sudo update-secureboot-policy --enroll-key
mokutil --list-new
```

After the operator boots the GA RT entry and unlocks LUKS, verify the
running kernel before configuring the native modules.

```bash
kernel=$(uname -r)
printf '%s\n' "$kernel"
[[ $kernel == 6.8.*-realtime ]]
test "$(cat /sys/kernel/realtime)" = 1
grep -E '^(CONFIG_PREEMPT_RT|CONFIG_HZ|CONFIG_MODULE_SIG)=' "/boot/config-$kernel"
grep -qx 'CONFIG_PREEMPT_RT=y' "/boot/config-$kernel"
dpkg-query -W "linux-image-$kernel" "linux-headers-$kernel"
test "$(cat "/lib/modules/$kernel/build/include/config/kernel.release")" = "$kernel"
sudo mokutil --test-key /var/lib/shim-signed/mok/MOK.der
```

For a boot rollback, select `6.8.0-151-generic` in the visible Advanced
options submenu and confirm SSH after LUKS unlock. Leave the fallback
installed. Make the RT entry the saved default only after a successful
maintenance test.

## Stock IgH build and transient attachment

Run this on the verified GA PREEMPT_RT kernel. Use the native 6.8 `ec_igc`.
Capture configure and compiler output. Ignore the upstream
`ethercatctl` start/stop path for this two-port host because its native
driver setup attempts to unload the shared `igc` module.

```bash
set -e
kernel=$(uname -r)
[[ $kernel == 6.8.*-realtime ]]
test "$(cat /sys/kernel/realtime)" = 1
test "$(cat "/lib/modules/$kernel/build/include/config/kernel.release")" = "$kernel"
git clone --branch 1.6.13 --depth 1 https://gitlab.com/etherlab.org/ethercat.git ~/igh-1.6.13
cd ~/igh-1.6.13
test "$(git rev-parse HEAD)" = 61cc654f5b721ddd54df0f58bdd34106d91c5359
git status --porcelain
./bootstrap
./configure --prefix=/opt/etherlab --with-systemdsystemunitdir=no \
  --with-linux-dir="/lib/modules/$kernel/build" --disable-generic --enable-igc --with-igc-kernel=6.8
set -o pipefail
make -j4 2>&1 | tee userspace-build.log
make -j4 modules 2>&1 | tee modules-build.log
sudo make install
for module in master/ec_master.ko devices/igc/ec_igc.ko; do
  sudo "/lib/modules/$kernel/build/scripts/sign-file" sha256 \
    /var/lib/shim-signed/mok/MOK.priv /var/lib/shim-signed/mok/MOK.der "$module"
  sudo install -D -m 0644 "$module" "/lib/modules/$kernel/updates/riser-igh-stock/${module##*/}"
done
sudo depmod -a "$kernel"
modinfo -n ec_master
modinfo -n ec_igc
modinfo -F vermagic ec_igc
modinfo -F signer ec_igc
/opt/etherlab/bin/ethercat version
```

With the bus disconnected, keep the existing SSH session open. Confirm that
`enp87s0` has no address or route and that the SSH route and the
`0000:59:00.0` driver are unchanged. Load the native module with both
ports still claimed by Linux, then switch only the fieldbus PCI device.

```bash
ip route get "${SSH_CONNECTION%% *}"
ip -br address show dev enp87s0
ethtool -P enp87s0
ethtool -P enp89s0
sudo bash <<'BIND'
set -euo pipefail
fieldbus=0000:57:00.0
ssh=0000:59:00.0
pci=/sys/bus/pci
test "$(readlink -f "$pci/devices/$ssh/driver")" = "$pci/drivers/igc"
test "$(readlink -f "$pci/devices/$fieldbus/driver")" = "$pci/drivers/igc"
test "$(ethtool -P enp87s0 | awk '{print $3}')" = 38:05:25:37:9f:32
modprobe ec_master main_devices=38:05:25:37:9f:32
modprobe ec_igc
ip link set enp87s0 down
printf '%s\n' ec_igc > "$pci/devices/$fieldbus/driver_override"
printf '%s\n' "$fieldbus" > "$pci/drivers/igc/unbind"
printf '%s\n' "$fieldbus" > "$pci/drivers/ec_igc/bind"
test "$(readlink -f "$pci/devices/$fieldbus/driver")" = "$pci/drivers/ec_igc"
test "$(readlink -f "$pci/devices/$ssh/driver")" = "$pci/drivers/igc"
BIND
lsmod | grep -E '^(igc|ec_master|ec_igc)\b'
lspci -nnk -s 57:00.0
lspci -nnk -s 59:00.0
for module in ec_master ec_igc; do
  modinfo -n "$module"
  modinfo -F srcversion "$module"
  cat "/sys/module/$module/srcversion"
done
sudo /opt/etherlab/bin/ethercat master
sudo /opt/etherlab/bin/ethercat slaves -v
sudo journalctl -k -b --no-pager | grep -Ei 'EtherCAT|ec_igc'
```

Expect master 0 idle and inactive, with the main device MAC attached,
link down and zero slaves. Compare loaded and on-disk source versions.
From the SSH client, open a fresh connection with
`ssh -o ControlMaster=no -o ControlPath=none smplab@10.34.191.186` and
verify `ip route get "${SSH_CONNECTION%% *}"` inside it.

Only after the fresh login and attachment checks pass, persist the guarded
PCI-specific sequence in a root-owned oneshot unit after
`systemd-udev-settle.service` and `network-online.target`. Keep the unit
disabled until its manual start/stop and SSH checks pass. Avoid starting ROS
from this unit. If userspace access is needed, scope access to
`/dev/EtherCAT0` for the bench user, for example temporarily with
`sudo apt install acl` followed by `sudo setfacl -m u:smplab:rw /dev/EtherCAT0`. For FIFO 80 and memory
locking in a fresh SSH session, use PAM limits scoped to `smplab`.

```bash
sudo tee /etc/security/limits.d/90-riser-commissioning.conf >/dev/null <<'LIMITS'
smplab - rtprio 80
smplab - memlock unlimited
LIMITS
# In a fresh SSH session:
ulimit -r
ulimit -l
chrt -f 80 true
```

To roll back the transient bind while the bus is idle and ROS is stopped,
unset the override and bind that same PCI address back to Linux. Disable
any dedicated persistence unit first. The override itself clears on reboot.

```bash
sudo bash <<'ROLLBACK'
set -euo pipefail
fieldbus=0000:57:00.0
pci=/sys/bus/pci
if [[ $(readlink -f "$pci/devices/$fieldbus/driver") == "$pci/drivers/ec_igc" ]]; then
  printf '%s\n' "$fieldbus" > "$pci/drivers/ec_igc/unbind"
fi
printf '\n' > "$pci/devices/$fieldbus/driver_override"
if [[ ! -L "$pci/devices/$fieldbus/driver" ]]; then
  printf '%s\n' "$fieldbus" > "$pci/drivers/igc/bind"
fi
test "$(readlink -f "$pci/devices/$fieldbus/driver")" = "$pci/drivers/igc"
modprobe -r ec_igc ec_master
ROLLBACK
ip route get "${SSH_CONNECTION%% *}"
```

To remove the IgH selection, move the complete
`updates/riser-igh-stock` directory outside the running module tree and run
`sudo depmod -a "$(uname -r)"`. Remove any dedicated limits or udev rule
that was added. Keep a signed copy of stock modules and the source revision
for comparisons.

## Jazzy build and disabled behavior

The operator installed Ubuntu build prerequisites, `rt-tests` 2.5-1 and
ROS Jazzy with the Noble `ros2-apt-source` 1.3.0 package. Installed
controller manager, controller interface, and hardware interface are
4.50.0. Joint state broadcaster is 4.43.1, robot state publisher 3.3.4,
and rclcpp 28.1.23. `rosdep check` passed.

For the host-side build, the stock source was configured with
`--disable-kernel --prefix=/tmp/riser-igh-userspace --with-systemdsystemunitdir=no`.
The staged CLI reports 1.6.13. Ten Jazzy packages built with Release,
`RISER_WITH_IGH=ON` and the staged header and `libethercat.so` paths.
The plugin resolves `libethercat.so.1` there. The final affected C++
rebuild and selected test suite passed (25 tests). The initial full
build emitted CMake diagnostics for unused IgH arguments on unrelated
packages. The existing boundary's Jazzy initializer deprecation was fixed.
Logs are `/tmp/riser-jazzy-build.log`, `/tmp/riser-jazzy-rebuild-final.log`
and `/tmp/riser-jazzy-retest-final.log`. Copies are in the evidence
directory's `build/` subdirectory.

`--show-args` confirmed that commissioning is disabled by default. The
disconnected smoke test loaded the intended plugin and failed at
`/dev/EtherCAT0` acquisition before PDO setup, as expected before
native module installation. The controller manager rejected initial
activation and the launch stopped the publisher. Log:
`/tmp/riser-bottom-disconnected-final.log`.

Build the production overlay against the final stock library after its
installation. The optional plugin is off in default builds. The mock,
Gazebo and full-robot EtherCAT boundary entry points are independent.

```bash
cd ~/riser-control
source /opt/ros/jazzy/setup.bash
rosdep check --rosdistro jazzy --from-paths src --ignore-src
colcon build --packages-up-to riser_motion riser_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DRISER_WITH_IGH=ON \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so
source install/setup.bash
colcon test --packages-select riser_hardware riser_motion riser_bringup
colcon test-result --verbose
ros2 launch riser_motion bottom_motor.launch.py --show-args
```

The archived Sysmac snapshot uses 1:1 gearing and semi-closed motor
feedback. Position and velocity use the 20-bit encoder and the carriage
pitch in `axes.yaml` to report m and m/s. Raw signed values and the archived
RxPDO `0x1600` and TxPDO
`0x1A00` mappings are preserved. Output remains zero through configure,
activation, every disabled PDO send, deactivation and shutdown.
Both command-switch phases reject a velocity claim in disabled mode.
The launch omits the motion controller in this mode. The state-only
broadcaster does not claim commands. The explicit powered setting only
loads the profile inactive. The first operation-enable request on that
path would require a later controller claim and healthy feedback. Fault
handling latches with zero target and the G5 Shutdown controlword on the
powered path. It performs no automatic fault reset. The ordered PREOP
sequence below is the normal bench exit.

## Host measurements

CPU 3 was selected for comparison with the Jetson procedure. It shares a
physical core with CPU 2. No CPU isolation or governor tuning was applied.
The generic kernel was in `powersave` mode, with realtime runtime budget
950,000 µs per second. At 1 kHz, record both live thread scheduling and
memory locking. Repeat after the GA RT reboot under equivalent conditions.

```bash
sudo cyclictest --default-system --policy=fifo --priority=80 --mlockall \
  --affinity=3 --threads=1 --interval=1000 --duration=60s --quiet \
  --histogram=10000 --json=/tmp/riser-cyclictest-rt.json | tee /tmp/riser-cyclictest-rt.log
# In a separate terminal during the measurement:
pid=$(pgrep -n -x cyclictest)
for task in /proc/"$pid"/task/*; do
  chrt -p "${task##*/}"
  grep -E '^(Name|Cpus_allowed_list|VmLck):' "$task/status"
done
```

The operator ran the same cyclictest options on the generic kernel
with a `generic` log suffix. The recorded run was 05:05:27–05:06:27 UTC,
with 60,000 samples, 1 µs minimum and 9.47 µs mean. The adjacent live
capture at 05:04:30 showed the worker at FIFO 80 on CPU 3 and 68,500 kB
locked. The main thread was SCHED_OTHER, priority 0. The live capture
predates the saved histogram run. Load at capture was
1.06 / 0.64 / 0.36. No histogram overflows or wakes 1 ms late. This
measurement records scheduler wakeup latency.

| Generic cyclictest wakeup (µs) | p50 | p99 | p99.9 | p99.99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| FIFO 80, CPU 3 | 9 | 13 | 21 | 37 | 40 |

Evidence is `/tmp/riser-cyclictest-generic{,-scheduling}.log` and
`/tmp/riser-cyclictest-generic.json`. An unprivileged cyclictest attempt
failed its scheduling privilege check. A separate unprivileged absolute
sleep probe measured 60,000 wakes, SCHED_OTHER/0, CPU 3, 18,780 kB locked,
p99.9 wakeup lateness 74.550 µs, maximum 550.866 µs and zero wakes 1 ms
late. See `/tmp/riser-host-baseline/generic-wakeup.log`.

The mock used the existing headless entry point and the normal position
controller and joint state broadcaster. A copy of
`src/riser_bringup/config/controllers.yaml` set manager affinity to CPU 3
and requested priority 80. No trajectory was sent. The available
SSH limit was realtime priority 0, so the actual update thread remained
SCHED_OTHER/0 in both completed runs. Reproduce the locked run with
`lock_memory: true` in that YAML, using:

```bash
MALLOC_ARENA_MAX=2 ROS_DOMAIN_ID=67 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 launch riser_bringup control.launch.py backend:=mock headless:=true \
  plot:=false wrench_input:=false controllers_file:="$HOME/riserms-commissioning-20261120/mock-locked/controllers.yaml"
```

| Mock run | Actual locked memory | Observed timing |
| --- | ---: | --- |
| `lock_memory: false`, 05:08 UTC, initial load 0.19 / 0.37 / 0.31 | 0 kB | Controller manager periodicity 943.675–1031.619 Hz at the 60 s snapshot, corresponding to 969.35–1059.69 µs intervals. |
| `lock_memory: true`, `MALLOC_ARENA_MAX=2`, 05:15 UTC, initial load 0.24 / 0.29 / 0.29 | 453,900 kB | 61,890 hardware-read samples by the statistics snapshot. Controller manager periodicity 648.736–2190.115 Hz, corresponding to 456.60–1541.46 µs intervals. |

A separate 20 s observation of the unlocked run collected 19,002
successive joint-state source stamp intervals. Median 1000.025 µs,
p99 1040.212 µs, p99.9 1047.579 µs, p99.99 1056.585 µs,
maximum 1058.052 µs, and zero sample gaps at or above 1.5 ms.
Subscriber losses could contribute to this observation. No runtime
overrun warnings were logged in either completed mock. The diagnostics
reported periodicity jitter in both. No exact overrun counter was
collected. Two earlier lock-enabled attempts without the arena cap
failed in ROS during startup. The successful lock-enabled manager
finished after SIGINT. Logs are in `/tmp/riser-mock-other` and
`/tmp/riser-mock-locked`. These are measurements of the host mock.

## First connected bench run

After host verification, connect the dedicated port to the G5. With ROS
stopped, the first command is:

```bash
sudo /opt/etherlab/bin/ethercat master
```

Require the fieldbus MAC attached, link up, idle/inactive and one slave.
Then verify topology, identity, error/detail, raw feedback and gearing in
PREOP. Do readbacks between runs. Take the first and last position readings
in the same power session because the encoder reference can change.

```bash
EC=/opt/etherlab/bin/ethercat
sudo "$EC" slaves -v
sudo "$EC" upload -p 0 -t uint16 0x603f 0
sudo "$EC" upload -p 0 -t uint16 0x4001 0
sudo "$EC" upload -p 0 -t uint16 0x6041 0
sudo "$EC" upload -p 0 -t int32 0x6064 0
sudo "$EC" upload -p 0 -t int32 0x606c 0
sudo "$EC" upload -p 0 -t uint32 0x6091 1
sudo "$EC" upload -p 0 -t uint32 0x6091 2
sudo "$EC" upload -p 0 -t int16 0x605b 0
sudo "$EC" pdos -p 0
```

Expect alias 0, position 0, PREOP, vendor `0x00000083`, product
`0x00000004`, revision `0x00020001`, zero error/detail, stationary
velocity, gear ratio 1:1 and shutdown option `-1`. Confirm the motor and
compare the live snapshot with the Jetson handoff.

Run three fresh trials on stock IgH, at least two with no ROS controllers
and one with feedback. Use a fresh launch and idle/PREOP start for each
trial. Use the same domain in both terminals. Record a new log directory
for each run.

```bash
# Terminal A, from ~/riser-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST MALLOC_ARENA_MAX=2
RUN=$(mktemp -d "$PWD/../riser-g5-stock.XXXXXX")
printf '%s\n' "$RUN"
date -Is > "$RUN/start-time"
python3 - "$RUN/controllers.yaml" <<'PY'
import sys
from pathlib import Path
import yaml

config = yaml.safe_load(Path("src/riser_motion/config/controllers.yaml").read_text())
config["controller_manager"]["ros__parameters"].update(
    cpu_affinity=3, lock_memory=True, thread_priority=80)
Path(sys.argv[1]).write_text(yaml.safe_dump(config, sort_keys=False))
PY
ros2 launch riser_motion bottom_motor.launch.py \
  product_code:=0x00000004 commissioning_disabled:=1 sync0_shift_ns:=650000 \
  controllers_file:="$RUN/controllers.yaml" \
  2>&1 | tee "$RUN/launch.log"
```

```bash
# Terminal B, substitute the log directory printed in terminal A.
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
RUN=/home/smplab/riser-g5-stock.REPLACE
EC=/opt/etherlab/bin/ethercat
"$EC" slaves | tee "$RUN/slaves.log"
"$EC" domains | tee "$RUN/domains.log"
ros2 control list_controllers | tee "$RUN/controllers.log"
ros2 control list_hardware_interfaces | tee "$RUN/interfaces.log"
pid=$(pgrep -n -x ros2_control_no)
for task in /proc/"$pid"/task/*; do
  chrt -p "${task##*/}"
  grep -E '^(Name|Cpus_allowed_list|VmLck):' "$task/status"
done > "$RUN/scheduling.log"
```

Require OP, unclaimed velocity, and one update thread with FIFO 80 and
affinity CPU 3. Confirm a positive process `VmLck`. In the no-controller
trials leave the controller list empty. For the feedback trial, start the
state-only broadcaster after OP and observe feedback for a minute.

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager --param-file "$RUN/controllers.yaml"
timeout 10s ros2 topic echo --once /joint_states > "$RUN/joint-states.log"
timeout 10s ros2 topic echo --once /dynamic_joint_states > "$RUN/dynamic-before.log"
sleep 60
timeout 10s ros2 topic echo --once /dynamic_joint_states > "$RUN/dynamic-after.log"
```

Expect stationary feedback and `ready=1` on healthy disabled exchange.
For every trial let OP run for at least 60 seconds. Use the ordered helper
while the ROS control loop is still running, then Ctrl+C in terminal A
after successful unconfigure.

```bash
set -o pipefail
ETHERCAT_CLI=/opt/etherlab/bin/ethercat \
  bash src/riser_hardware/scripts/shutdown_bottom_motor.sh | tee "$RUN/shutdown.log"
sudo journalctl -k -b --since "$(cat "$RUN/start-time")" --no-pager > "$RUN/kernel.log"
grep -E 'Disabled PDO summary|Incomplete OP detail|Overrun' "$RUN/launch.log"
```

Require complete WKC (3/3), no incomplete established-OP samples, zero
enabled and faulted samples, and a deliberate OP exit with a PREOP tail.
Check idle/PREOP, error/detail zero, disabled statusword and original
raw position/velocity within measurement uncertainty. Save the startup
kernel log, WKC breakdown and maximum PDO read interval. Record sync
warnings, observed DC acquisition/offset, and intentional shutdown
transitions separately. Host timing results do not qualify those checks.

## DC patch review

In pristine 1.6.13 `master/fsm_master.c`, the FPRD at `0x0910` reads the
ESC time and the existing offset at `0x0920` in one datagram.
`ec_fsm_master_state_dc_read_offset()` checks receipt and WKC 1. The
stock 64-bit path adds a correction from elapsed jiffies, compares with
the current application time and only changes the offset if the difference
exceeds 1 ms. The slave setup checks 10 µs convergence and warns after 5 s.
There is no per-datagram application timestamp in the stock source.

`git apply --check` of the archived patch failed at `master/master.c:1108`.
The 1.6.13 send loop publishes `EC_DATAGRAM_SENT` with a release store after
stamping the datagram and removing it from the sent queue. Any later port
must capture the sending cycle's application time before this publication,
preserve retries and timestamp resets, and retain the one-slave G5
identity, active master, 64-bit and INIT/PREOP guards. The patch checksum is
`da1e0d46dd8bc5ed4a118d2651e4935a85870fb5c5b47428ede9ae1c64b1478b`.
It bypasses the coarse tolerance for that guarded branch. The Jetson result supports
this comparison but gives no x86 bench outcome.

Keep the module stock. If timeout or established-OP WKC 2 recurs, capture
application time, DC reference and sync monitor around OP entry and frame
phase near the event. The Jetson stock failure entered OP at 5.228 s at a
1.793 ms difference and had 18 WKC 2 events 3–58 µs before inferred SYNC0.
Two other stock Jetson starts were clean. Reproduce the timestamp/phase
signature before a matched comparison. Preserve the signed stock module
set, and compare on the same kernel, G5, rate, phase, scheduling and
feedback workload. If a candidate is ever installed, remove its whole
matching module set, run depmod and confirm the selected and loaded stock
versions on rollback.
