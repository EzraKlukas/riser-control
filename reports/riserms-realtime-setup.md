# riserms realtime and EtherCAT setup

For current drive settings, motion-inhibited first contact, and the powered-test prerequisites, use the [G5 motion bringup checklist](g5-motion-bringup.md).


## Operating configuration

riserms is a Minisforum MS-01 running Ubuntu 24.04.5 LTS amd64 with
Secure Boot and a LUKS-encrypted root. Keep the drive disconnected for
host maintenance. Disconnected software acceptance passed after the
coordinated persistence reboot. Motion launches are stopped.

| Item | Setting |
| --- | --- |
| Normal boot | Ubuntu Pro GA `6.8.1-1065-realtime`, package `6.8.1-1065.66`. PREEMPT_RT enabled, 1,000 Hz tick, matching headers. |
| GRUB | `/etc/default/grub.d/99-riser-commissioning.cfg` sets `GRUB_DEFAULT=saved`, `GRUB_SAVEDEFAULT=false`, and a visible 10-second menu. The saved versioned RT entry is under **Advanced options for Ubuntu**. Full generated IDs and menu verification are in `prepare-system.log`. |
| Fallback | **Advanced options for Ubuntu → Ubuntu, with Linux 6.8.0-151-generic**. Both kernels' images, modules, and extra modules are marked manually installed. The GA RT meta package remains eligible for updates. |
| Management | I226-LM `8086:125b`, PCI `0000:59:00.0`, permanent MAC `38:05:25:37:9f:33`, `enp89s0`, Linux `igc`. This port carries SSH to `10.34.191.186`. |
| EtherCAT | I226-V `8086:125c`, PCI `0000:57:00.0`, permanent MAC `38:05:25:37:9f:32`, `enp87s0` under Linux. Native `ec_igc` takes this PCI device only. |
| IgH | Stock 1.6.13 at `61cc654f5b721ddd54df0f58bdd34106d91c5359`. CLI `/opt/etherlab/bin/ethercat`, headers `/opt/etherlab/include`, library `/opt/etherlab/lib/libethercat.so.1.2.0`. |
| Device access | Dedicated udev rule for `/dev/EtherCAT0`, `root:smplab`, mode `0660`. |
| Operator RT | `/etc/security/limits.d/90-riser-commissioning.conf` gives only `smplab` realtime priority up to 80 and unlimited locked memory. SSH PAM applies the limits on a fresh login. |
| Jazzy overlay | `/home/smplab/riser-control/install`. The source checkout is `/home/smplab/riser-control`, also called riser-control-ws in discussion. |

From a fresh SSH login on the management port:

```bash
uname -r
cat /sys/kernel/realtime
cat "/lib/modules/$(uname -r)/build/include/config/kernel.release"
mokutil --sb-state
fingerprint=$(openssl x509 -inform DER -in /var/lib/shim-signed/mok/MOK.der -noout -fingerprint -sha1)
mokutil --list-enrolled | grep -Fi "SHA1 Fingerprint: ${fingerprint#*=}"
grub-editenv list
ip route get "${SSH_CONNECTION%% *}"
lspci -nnk -s 57:00.0
lspci -nnk -s 59:00.0
systemctl status --no-pager riserms-ethercat.service
stat -c '%U:%G %a %n' /dev/EtherCAT0
ulimit -r
ulimit -l
chrt -f 80 true
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves -v
```

Disconnected acceptance requires master 0 in Idle, Active no, Slaves 0,
main MAC `38:05:25:37:9f:32 (attached)`, and Link DOWN. The verbose slave
list is empty. The operator can read the master without sudo.

## Recovery

If RT fails to boot, select the versioned generic entry at the local
console, enter the LUKS passphrase, and confirm SSH. The bind service
requires an Ubuntu GA realtime kernel and a signed local module pair
matching that release. On the generic fallback both I226 ports use
Linux `igc`. To change the saved fallback deliberately:

```bash
sudo grep -E '^submenu |menuentry .*6\.8' /boot/grub/grub.cfg
sudo grub-set-default 'gnulinux-advanced-f5d7422b-7c4f-4884-81bb-293b194db848>gnulinux-6.8.0-151-generic-advanced-f5d7422b-7c4f-4884-81bb-293b194db848'
grub-editenv list
```

With ROS stopped and the master idle, `sudo systemctl stop
riserms-ethercat.service` restores the fieldbus port to Linux `igc`.
Use `sudo systemctl disable riserms-ethercat.service` to suspend boot
binding. If systemd lost track of a manual bind, use
`sudo /usr/local/sbin/riserms-ethercat-port stop` under the same
conditions. The guard refuses to stop an active master, clears the
fieldbus PCI override only, restores that device, and unloads the two
IgH modules. Check both BDFs and the management route afterward.

Avoid the upstream `ethercatctl` start/stop path on this host because
it tries to unload the shared Linux network driver. Keep `igc`
available to the management port and the generic fallback. To remove
the stock module selection, stop and disable the unit, move the
complete `updates/riser-igh-stock` directory out of the corresponding
module tree, and run `sudo depmod -a <release>`.

```bash
journalctl -u riserms-ethercat.service -b --no-pager
journalctl -k -b --no-pager --grep='EtherCAT|ec_igc|signature'
```

## Source, packages, and production build

The active checkout is on `main`, tracking `origin/main`, base
`86219c3c29542bfec1d7bbda26c62f452b1edec7`. The dirty working tree
already contained the Jazzy bottom-axis plugin, guard, separate
bringup package, launch, tests, and shutdown helper. These were
preserved. This pass adds the bind helper, unit, and udev rule in
[src/riser_hardware/scripts/riserms](../src/riser_hardware/scripts/riserms),
this document, and a pointer from the prior report. Initial and final
Git status are saved in the evidence directory.

The Jetson reference `~/riser-ethercat-jetson` remains read-only. It is
an archive of `feature/humble-ethercat-csv` based on
`62e0cbc8d6318850ac50dbf97cc458e839a0913a` with uncommitted changes.
The [prior commissioning record](riserms-ethercat-commissioning.md)
contains the detailed Jetson DC patch review and earlier generic-host
measurements. This host uses stock code at the IgH 1.6.13 revision above.

| Package | Version |
| --- | --- |
| Ubuntu Pro client, Ubuntu realtime meta | `38ubuntu0~24.04`, `1.1.2~24.04.1` |
| GA RT image, headers and modules | `6.8.1-1065.66` |
| Generic fallback image and modules | `6.8.0-151.151` |
| Signed GRUB, shim, mokutil | `1.202.5+2.12-1ubuntu7.3`, `1.58+15.8-0ubuntu1`, `0.6.0-2build3` |
| rt-tests, ros2-apt-source | `2.5-1`, `1.3.0~noble` |
| Jazzy ros2_control manager, controller and hardware interfaces | `4.50.0` |
| Jazzy joint state broadcaster, robot state publisher, rclcpp | `4.43.1`, `3.3.4`, `28.1.23` |

```bash
cd /home/smplab/riser-control
source /opt/ros/jazzy/setup.bash
rosdep check --rosdistro jazzy --from-paths src --ignore-src
colcon build --packages-up-to riser_motion_bringup riser_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DRISER_WITH_IGH=ON \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so \
  --no-warn-unused-cli
source install/setup.bash
ldd install/riser_hardware/lib/libriser_bottom_csv.so | grep libethercat
colcon test --packages-select riser_hardware riser_motion_bringup riser_bringup \
  --return-code-on-test-failure
colcon test-result --verbose
ros2 launch riser_motion_bringup bottom_motor.launch.py --show-args
```

Ten production packages built against the installed library, and all
25 selected tests passed. The launch defaults to `commissioning_disabled:=1`
and omits the motion controller. The disabled hardware path rejects
command-interface claims in both switching phases and sends zero
controlword and target if it reaches cyclic exchange. In the disconnected
smoke test, the plugin opened the installed master and logged `EtherCAT slave not found in bus scan` before PDO setup.
Jazzy controller manager aborted on the deliberately failed initial
hardware activation and launch stopped the publisher. There was no
motion spawner, velocity claim, or active cyclic domain. The master
was idle and inactive before and after. Evidence is
`bottom-disconnected.log` and `smoke-master-{before,after}.log`.

## Kernel updates and module signing

Keep `ubuntu-realtime` and the GA `linux-realtime` packages updateable.
The versioned saved GRUB entry stays on the qualified kernel when APT
installs another release. Keep a known-good RT image and its modules
beside the generic fallback.

Install the candidate's matching Ubuntu headers. With `release` set to
the appropriate installed GA RT version, rebuild the stock source in a
clean working directory:

```bash
release=$(uname -r)
test "$(cat "/lib/modules/$release/build/include/config/kernel.release")" = "$release"
grep -qx 'CONFIG_PREEMPT_RT=y' "/boot/config-$release"
git clone --depth 1 --branch 1.6.13 https://gitlab.com/etherlab.org/ethercat.git "igh-1.6.13-$release"
cd "igh-1.6.13-$release"
test "$(git rev-parse HEAD)" = 61cc654f5b721ddd54df0f58bdd34106d91c5359
test -z "$(git status --porcelain)"
./bootstrap
./configure --prefix=/opt/etherlab --with-systemdsystemunitdir=no \
  --with-linux-dir="/lib/modules/$release/build" --disable-generic --enable-igc --with-igc-kernel=6.8
make -j6
make -j6 modules CC=x86_64-linux-gnu-gcc-13
```

Check `include/generated/compile.h` in the target headers for the compiler
identity. The initial build used GCC 13.3.0 from the same Ubuntu package
under the shorter name `gcc-13`, which produced Kbuild's compiler-name
warning. It built both modules successfully. No project source warning
was emitted in the final Jazzy build.

The root-held Ubuntu module-only MOK is in `/var/lib/shim-signed/mok`.
Subject is `CN = riserms Secure Boot Module Signature key` and SHA-256
fingerprint is
`30:0B:54:C2:C6:BC:34:B9:F0:28:AC:4E:C9:F7:08:FD:56:AC:8D:5C:D8:CE:A1:64:12:CB:98:81:7C:85:50:DC`.
When replacing an absent key, use `sudo update-secureboot-policy --new-key`
and queue the DER with `sudo mokutil --import
/var/lib/shim-signed/mok/MOK.der`. Verify the request with
`sudo mokutil --list-new`. Its EFI variables are root-readable only.
With the console attended, select **Enroll MOK → Continue → Yes** in
MokManager, enter the one-time password, and reboot. Boot the saved
entry, unlock LUKS, confirm SSH, and verify the certificate fingerprint.
On this host `mokutil --test-key` reported enrolled but returned an
error from its keyring lookup. An exact fingerprint match against
`mokutil --list-enrolled` and successful module loads under Secure Boot
verify trust. [Ubuntu Secure Boot reference](https://documentation.ubuntu.com/security/security-features/platform-protections/secure-boot/)

```bash
mok=/var/lib/shim-signed/mok/MOK
fingerprint=$(openssl x509 -inform DER -in "$mok.der" -noout -fingerprint -sha1)
mokutil --list-enrolled | grep -Fi "SHA1 Fingerprint: ${fingerprint#*=}"
for file in master/ec_master.ko devices/igc/ec_igc.ko; do
  sudo "/lib/modules/$release/build/scripts/sign-file" sha256 "$mok.priv" "$mok.der" "$file"
  sudo install -D -m 0644 "$file" "/lib/modules/$release/updates/riser-igh-stock/${file##*/}"
done
sudo depmod -a "$release"
for module in ec_master ec_igc; do
  modinfo -k "$release" -n "$module"
  modinfo -k "$release" -F vermagic "$module"
  modinfo -k "$release" -F signer "$module"
done
```

There is no DKMS build or upstream service on this host. The
`riserms-ethercat.service` oneshot starts after `network-online.target`.
It checks RT, signatures and vermagic before starting. The helper verifies
the two PCI-to-interface mappings and permanent MACs, and checks that the
fieldbus interface has no host addresses or routes. It loads stock
`ec_master` for the fieldbus MAC, registers `ec_igc` while both PCI
devices are still on Linux, then switches the fieldbus device using its
PCI driver override. It checks master attachment and rolls back if the
start fails. The service starts no ROS processes. Installed files are
`/usr/local/sbin/riserms-ethercat-port`,
`/etc/systemd/system/riserms-ethercat.service`, and
`/etc/udev/rules.d/90-riserms-ethercat.rules`.

Reuse the 1.6.13 userspace install for ABI-matched kernel-only updates.
For a replacement userspace build, stage with `make DESTDIR=<stage>
install` and install the CLI, headers, and versioned library under
`/opt/etherlab`. Preserve SONAME and development symlinks.

To qualify a new kernel, set `release` explicitly to the candidate when
building, sign and install its pair, and select the versioned candidate
entry once at the console. The previous saved kernel stays the default.
After LUKS unlock confirm SSH, the running release, loaded source
versions, both NIC drivers, and disconnected acceptance. Repeat the
host timing tests. If the candidate passes, manually retain its image,
modules and extra modules, then promote its exact entry:

```bash
sudo grep -E '^submenu |menuentry .*realtime' /boot/grub/grub.cfg
# Substitute the printed menu entry IDs.
advanced_id='<advanced submenu ID>'
candidate_id='<versioned RT entry ID>'
sudo grub-set-default "$advanced_id>$candidate_id"
grub-editenv list
```

Verify the promoted default on the next coordinated boot. For a failed
candidate, leave the saved entry unchanged and return to the previous
kernel. [GNU GRUB saved entry reference](https://www.gnu.org/software/grub/manual/grub/html_node/saved_005fentry.html)

## Acceptance and host timing

Secure Boot remained enabled after the MOK console reboot. Both
modules loaded with the enrolled signer and matching running
vermagic. Loaded and selected source versions matched
`7949E2337AEAD177ED9463E` for `ec_master` and
`6317F6DF1351DB005C46855` for `ec_igc`. With boot persistence
disabled, manual start, stop and start each completed. Rollback
returned the fieldbus port to Linux `igc` and removed the IgH pair.
The management port and SSH route remained on `enp89s0` throughout.
After rebind, the disconnected master and operator node permissions
matched the acceptance checks. The operator confirmed a new
non-multiplexed laptop SSH connection through `enp89s0` and read the
master before enabling persistence. The next coordinated reboot
produced boot ID `877f4bdf-dcb2-420f-b458-07d1d14ed01b`. The unit started
automatically at 18:34:02 UTC and remained enabled and active (exited).
On that boot the running RT release, saved GRUB entry, enrolled key,
loaded source versions, native fieldbus binding, Linux management
driver, operator access and idle master all passed. No ROS controller
manager was running. Evidence is `transient-bind.log`,
`pre-persistence-reboot.log`, and `persistent-boot.log`.

Raw evidence is in `log/riserms-realtime-20261120/` in the active
workspace. The previous generic-host evidence is under
`~/riserms-commissioning-20261120/`. CPU 3 is on a P-core and shares
it with CPU 2. The hybrid PMU exposes P-cores 0–11 and E-cores 12–19.
CPU 3 preserves comparability with the earlier generic run. The
baseline uses `powersave`, default RT runtime budget 950,000 µs/s,
and no CPU isolation, IRQ steering, or kernel command-line tuning.

The 1 kHz tests ran sequentially after MOK enrollment in a fresh
operator SSH session with RT priority limit 80 and unlimited memlock.
At 18:24 UTC the host had been up one minute. Load averages at the
start of cyclictest were 2.55 / 1.39 / 0.54.

| Test | Actual update thread | Result |
| --- | --- | --- |
| cyclictest 2.50, `--default-system`, one thread, 60 s | FIFO 80, CPU 3, 68,512 kB locked | 60,000 wakes. Mean 10.64 µs, p50 10 µs, p99 17 µs, p99.9 23 µs, p99.99 26 µs, max 43 µs. Zero histogram overflows and zero wakes at least 1 ms late. |
| Headless Jazzy mock, 82 s launch window | FIFO 80, CPU 3, 449,156 kB locked | 55,798 hardware reads at the statistics snapshot. Manager periodicity 900.617–1,135.179 Hz, equivalent to about 881–1,110 µs intervals. Zero runtime overrun warnings. An exact missed-period count was unavailable. |

The normal mock position controller and joint-state broadcaster were
active. No trajectory was sent. The mock used temporary
`cpu_affinity: 3`, `thread_priority: 80`, and `lock_memory: true`
settings with `MALLOC_ARENA_MAX=2` to bound allocator arenas. A listener
on CPU 19 collected 49,174 joint-state source-stamp intervals over
50 seconds. Median 1,000.028 µs, p99 1,017.440 µs, p99.9 1,047.895 µs,
p99.99 1,087.930 µs, max 3,971.558 µs, and three intervals at least
1.5 ms. Subscriber loss can widen these intervals. The manager's own
diagnostics reported a smaller maximum interval. The manager shut down
cleanly. The mock physical-state publisher printed a Python interruption
traceback at SIGINT.

Evidence is in `rt-baseline.iCG9PQ/`. To repeat, run
`bash log/riserms-realtime-20261120/measure-rt.sh` in a fresh operator
SSH session with ROS stopped. The earlier generic-kernel FIFO run
measured 13 µs p99 and 40 µs max under different ambient load. These
individual runs establish neither an improvement nor a latency bound.
No persistent performance tuning was added. The measurements exercise
the host scheduler and ROS mock only.

## First contact

After the disconnected acceptance checks pass, keep ROS stopped and
the drive disabled. Connect only the dedicated fieldbus port to the G5.
Run these commands as `smplab`:

```bash
EC=/opt/etherlab/bin/ethercat
"$EC" master
"$EC" slaves -v
```

Expect Idle and inactive master 0, the same attached fieldbus MAC, link
UP, and one slave in PREOP at ring position 0, alias 0. The expected
rotary G5 identity is vendor `0x00000083`, product `0x00000004`,
revision `0x00020001`. If the link, count, ring position, alias, state
or identity differs, stop at those readouts, save the outputs and kernel
log, and check the fieldbus port, wiring, drive model and revision with
the lab operator. Keep ROS stopped.

When the topology and identity match, collect the following readbacks:

```bash
"$EC" upload -p 0 -t uint32 0x1018 1
"$EC" upload -p 0 -t uint32 0x1018 2
"$EC" upload -p 0 -t uint32 0x1018 3
"$EC" upload -p 0 -t uint16 0x6041 0
"$EC" upload -p 0 -t uint16 0x603f 0
"$EC" upload -p 0 -t uint16 0x4001 0
"$EC" upload -p 0 -t int32 0x6064 0
"$EC" upload -p 0 -t int32 0x606c 0
"$EC" upload -p 0 -t uint32 0x6091 1
"$EC" upload -p 0 -t uint32 0x6091 2
"$EC" upload -p 0 -t int16 0x605b 0
"$EC" pdos -p 0
```

The identity object should agree with the scan. Statusword operation-enabled
bit 2 and fault bit 3 should be clear, and both error reads should be zero.
Save the raw stationary position and velocity. The reference gearing was
1:1 and the Shutdown option was `-1`. PDO inspection records the current
map. Investigate any discrepancy or alarm while keeping the drive disabled.
Use the disabled cyclic checks below before a supervised hold.

### Servo-disabled cyclic check

After the readbacks match, do two fresh disabled runs on stock IgH, at
1 kHz with the reference SYNC0 shift of +650 µs. Start the state-only
broadcaster in the second run. Keep the same sourced environment and
ROS domain in each terminal.

```bash
cd /home/smplab/riser-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST MALLOC_ARENA_MAX=2
RUN=$(mktemp -d "$PWD/log/g5-checkout.XXXXXX")
printf '%s\n' "$RUN"
python3 - "$RUN/controllers.yaml" <<'PY'
import sys
from pathlib import Path
import yaml

config = yaml.safe_load(Path("src/riser_motion_bringup/config/controllers.yaml").read_text())
config["controller_manager"]["ros__parameters"].update(
    cpu_affinity=3, lock_memory=True, thread_priority=80)
config["motor_profile"]["ros__parameters"]["speed_m_s"] = 0.0
Path(sys.argv[1]).write_text(yaml.safe_dump(config, sort_keys=False))
PY
# Repeat the launch with FEEDBACK=true for the second run.
FEEDBACK=false
# The launch reads the commissioning limits from axes.yaml.
ros2 launch riser_motion_bringup bottom_motor.launch.py \
  commissioning_disabled:=1 sync0_shift_ns:=650000 feedback:="$FEEDBACK" \
  controllers_file:="$RUN/controllers.yaml" \
  2>&1 | tee "$RUN/disabled-launch.log"
```

From the second terminal, set `RUN` to the printed path, source
the same ROS environment, and set the same ROS domain. Wait for OP and
check the domain WKC and interface claims. On the feedback run, confirm
that the launched broadcasters show stationary feedback and `ready=1`. Observe
at least 60 seconds of OP per run.

```bash
EC=/opt/etherlab/bin/ethercat
"$EC" slaves
"$EC" domains
ros2 control list_controllers
ros2 control list_hardware_interfaces
# Feedback run only.
ros2 topic echo --once /dynamic_joint_states
```

With the launch still running, use the ordered shutdown helper.
Then stop the launch. Require complete WKC `3/3`, zero incomplete,
enabled, and faulted samples in the launch summary, clear drive alarms,
and idle/PREOP after shutdown. Treat a DC warning, unexplained motion,
nonzero alarm, or unexpected OP exit as a stop before any hold.

```bash
bash src/riser_hardware/scripts/shutdown_bottom_motor.sh
journalctl -k -b --no-pager --grep='Slave did not sync|EtherCAT'
```

### Supervised zero-velocity hold

Proceed only after both disabled runs pass. Before this step, verify
mechanical support against gravity, brake release behavior, travel
clearance, and the independent stop. Agree on a short observation
window and position and velocity tolerances. Keep someone at the
machine with the stop in reach.

From idle/PREOP, use the same zero-speed YAML and approved guard limits.
This time the launch loads `motor_profile` inactive.

```bash
ros2 launch riser_motion_bringup bottom_motor.launch.py \
  commissioning_disabled:=0 sync0_shift_ns:=650000 \
  controllers_file:="$RUN/controllers.yaml" \
  2>&1 | tee "$RUN/hold-launch.log"
```

Start the host recorder in the second terminal, then leave the
feedback monitor running. Before activation, verify OP with WKC `3/3`
on this fresh start, check the kernel log for DC sync warnings, and
confirm stationary, finite feedback with `ready=1`. Record the
initial position.

```bash
ros2 run riser_motion_bringup g5_trial start zero_hold /dynamic_joint_states /g5_plot/dynamic_joint_states
ros2 topic echo /dynamic_joint_states
```

From another sourced terminal on the same ROS domain, confirm
`motor_profile` is inactive, `bottom_motor/velocity` is unclaimed,
and the configured speed is exactly zero.

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 param get /motor_profile speed_m_s
```

Only after these checks, activate the profile deliberately:

```bash
ros2 control set_controller_state motor_profile active
```

Expect `ready=2`, near-zero measured speed, and position within the
agreed tolerance as the brake releases. Use the independent stop on
unexpected motion, loss of feedback, or failure to reach the expected
enabled state. The controller commands zero velocity and provides no
position correction. It retains enable when the profile timer ends.

At the end of the observation window, confirm the mechanism is
supported before removing torque. Run the ordered helper while the
launch is still running. The helper deactivates the profile, verifies
the velocity interface is unclaimed, and requests PREOP before
releasing the master.

```bash
bash src/riser_hardware/scripts/shutdown_bottom_motor.sh --stop-profile
```

Confirm idle/PREOP, zero alarm and detail, and a disabled statusword.
Then finalize the recording with `ros2 run riser_motion_bringup g5_trial stop`, retain its reported rates and loss counts, and stop the launch.

## Remote LUKS unlock

Pre-boot SSH uses Ubuntu's `dropbear-initramfs` on TCP 2222. Its only
listen address is `fe80::3a05:25ff:fe37:9f33` on the wired
`enp89s0` link. The script verifies the I226-LM PCI path, MAC
`38:05:25:37:9f:33`, and the Linux `igc` driver before bringing that
interface up. The stable link-local address needs no DHCP and stays
on the local network segment. The laptop must be on that same segment.
Normal DHCP, OpenSSH, and the EtherCAT port keep their existing setup.

The initramfs has a dedicated ED25519 host key with fingerprint
`SHA256:966RWXq2i9i5587dgRNmXzGwoM1tEeY/LjNAJmPio6I`. The only authorized
client key is the ED25519 key used by the operator's laptop
(`SHA256:z09WFR6VWdzrxsZ+cGPBsOJe0kh/OgP5UcgE/lpnLBE`). Authentication
forces `cryptroot-unlock` with an interactive terminal and disables
forwarding. Enter the **existing LUKS passphrase** directly at the
prompt. The usual LUKS prompt also remains available at the console.
There is no stored LUKS passphrase. Dropbear shuts down when the root
filesystem opens, and initramfs releases its management address to the
normal boot networking.

From the laptop, set `LAN_IF` to the lab-network interface. On macOS,
use `route -n get 10.34.191.186` to find its `interface:` line. On
Linux, use `ip route get 10.34.191.186` to find the `dev` field. Use
the pinned pre-boot key in `~/.ssh/known_hosts.riserms-initramfs`. For
initial laptop setup, add this line and run
`ssh-keygen -lf ~/.ssh/known_hosts.riserms-initramfs` to verify the
fingerprint:

```text
riserms-initramfs ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICV8E4PclpjreS969u6dqxcS9oILy1ULjApIIcWJpOJA
```

Unlock and reconnect:

```bash
ssh -6 -tt -p 2222 \
  -o HostKeyAlias=riserms-initramfs \
  -o UserKnownHostsFile="$HOME/.ssh/known_hosts.riserms-initramfs" \
  -o StrictHostKeyChecking=yes -o ClearAllForwardings=yes \
  "root@fe80::3a05:25ff:fe37:9f33%$LAN_IF"
# After cryptroot-unlock completes and the connection closes:
ssh smplab@10.34.191.186
```

The IPv4 address above is the current normal DHCP lease. If it changes,
use the current lease or reach normal OpenSSH on the management
link-local address using the **normal SSH host key**. The pre-boot
key belongs only to port 2222. A key mismatch is a reason to stop
before entering the passphrase.

The image uses `/etc/dropbear/initramfs/dropbear.conf`,
`/etc/dropbear/initramfs/authorized_keys`, and
`/etc/initramfs-tools/hooks/riserms-unlock-igc`. The RT image is
root-readable only because it contains the pre-boot host private key.
The saved GRUB default is the existing signed RT kernel. The
`6.8.0-151-generic` initramfs remains the console fallback.

The backup is `/root/riserms-unlock-backup.gF7OLA`. It contains the
original RT and generic initramfs images, hashes, and the boot and
initramfs configuration archive. Ubuntu `dropbear-initramfs` and
`dropbear-bin` are version `2022.83-4`. The image inspection in
`log/riserms-remote-unlock/stage.log` checked Dropbear, the expected
forced-command public key, the separate host key, cryptroot-unlock,
the LUKS root entry, the udev naming rule, and the signed kernel's
`igc` driver. The generic image was verified byte-for-byte unchanged.
The package's first automatic rebuild warned before an authorized key
was configured. Only the inspected candidate was installed.

For the attended reboot test, the operator reported successful access
after the remote-unlock procedure. The new boot ran
`6.8.1-1065-realtime` with PREEMPT_RT and Secure Boot enabled. Normal
SSH accepted the laptop key on `enp89s0`, the usual IPv4 DHCP address
was present, and TCP 2222 had no listener after boot. The EtherCAT unit
remained active. Host verification is in
`log/riserms-remote-unlock/host-after.log`.

Initial recovery is the local LUKS prompt. If the RT image needs to be
reverted, select the generic entry and use the console passphrase.
In a normal boot, move the dedicated initramfs hook and umask drop-in
into the root-only backup directory and purge `dropbear-initramfs`.
Restore the saved original RT image atomically:

```bash
BACKUP=/root/riserms-unlock-backup.gF7OLA
RT=6.8.1-1065-realtime
sudo mv /etc/initramfs-tools/hooks/riserms-unlock-igc \
  /etc/initramfs-tools/conf.d/riserms-unlock-umask "$BACKUP/"
sudo apt-get purge dropbear-initramfs
sudo cp -a "$BACKUP/initrd.img-$RT" "/boot/initrd.img-$RT.restore"
sudo mv -T "/boot/initrd.img-$RT.restore" "/boot/initrd.img-$RT"
grub-editenv list
```

If APT rebuilt the generic image during recovery, restore its matching
backup as well. Keep the saved RT entry and Secure Boot settings.
Coordinate a local-console reboot to verify the local prompt and
normal SSH after recovery.
