# G5 bottom-axis commissioning

## Evidence and the first velocity attempt

The November 20 hold log under `log/g5-checkout.npeGlD/` ends after configuring the inactive controller. It contains no alarm or time series from the reported brake event. A brief release followed by engagement is consistent with loss of servo enable. The initiating event is unresolved. On the next attempt start the host recorder **before controller activation**. Preserve the 1 kHz controlword, statusword, mode, drive error, host fault, carriage velocity, torque, limit word and validity. Retain the bag, kernel log and PREOP error detail before clearing anything.

The archived November 17 Sysmac export is `~/riser-ethercat-jetson/reports/G5_parameter_reference.html`, SHA-256 `f9b9ba7e77933b4c196b5f13a89ab1ffce8dfae20495e67926870fbea03392aa`. The separately named `G5_parameter_reference(1).html` was unavailable. Compare its digest before treating it as identical. The matching archive ESI is `R88A-KN ESI_Files/Omron R88D-KNxxx-ECT.xml`, rotary R88D-KN04L-ECT device revision 0x00020001. The Jetson handoff and logbook document the 1 ms cycle and +650 µs SYNC0 shift. riserms uses stock IgH 1.6.13. [Commissioned host configuration](riserms-realtime-setup.md)

| Setting | November 17 snapshot and consequence |
|---|---|
| Pn400.0/.1/.2 | 0x94, NC immediate stop in each mode. The operator subsequently reported **all three set to 0**. Confirm the packed selection at 0x3400 and the effective input functions. Verify the independent stop path with this assignment removed. |
| Pn401/Pn402, Pn504/Pn505 | 0x81 POT and 0x82 NOT, NC across modes. Both logical prohibitions were asserted. Pn504=1 disables drive-side prohibition. After verifying each wire and polarity, the intended value is **0**. Both open with this selection can cause a drive error. Pn505=0 applies dynamic braking then zero torque command into the prohibited direction. Retain a genuine drive fault through any retreat attempt. |
| Brake | Pn410.0/.1/.2=0x03 assigns SO1 to brake release. Pn437=0 ms at standstill, Pn438=0 ms during operation, Pn439=30 r/min threshold. Measure the relay/coil and mechanical engagement delay under support before accepting zero timing. OUT03 and PHOUT00 have no EtherCAT address in the export. 0x60FE is an output command. Bit 26 of 0x60FD supplies no independent brake measurement. |
| Direction and scale | Pn000=1 gives positive CCW viewed from the load side. Pn001=0 uses motor feedback. Pn748/Pn749=1:1 and a 20-bit encoder imply 1,048,576 command units per motor revolution. At the 0.010 m/rev configured in `axes.yaml`, that gives 104,857,600 drive units per metre. Verify direction relative to each machine endpoint. |
| Software travel | Pn801=3 disables both software limits. Pn743/744 at 0x607D:01/02 are −500000/+500000 command units. Qualify homing, coordinate origin, applicability in CSV and both thresholds before enabling drive-side software protection. The host checks the path length of each jog. |
| CSV torque | Pn521=6. With positive/negative external torque-limit inputs off, evaluate min(0x60E0, Pn753) forward and min(0x60E1, Pn754) reverse, in common units. With the corresponding input on, use Pn730/Pn731. All four parameter limits were 300% versus the 500% defaults. Also read CSV Max torque 0x6072 (0.1% rated), physical and EtherCAT torque-limit inputs and model peak capability. The effective cap is the tightest applicable restriction. A 300% setting alone does not establish a low commissioning cap. |
| Watchdog and stop | SM2 output watchdog enabled in the CSV map. Verify actual ESC watchdog divider/time, vendor response and measured time to loss of torque separately from DC loss. Pn776=1 tolerates one consecutive communications error. Pn777=0 selects a 120 s missing-SYNC timeout. Pn735/Pn736/Pn738=−1, Pn737=1. G5 ignores CiA Quick Stop. Host interlock requests Shutdown (0x0006) and zero target. Loss of frames prevents delivery of a host stop. |

Pn504 is the known conditional change. Other defaults needing an engineering decision are brake timing, software-limit enable and reference, torque selection/ceiling, and watchdog response time. Exact replacement values require the measured brake delay, signed stroke, adequate holding torque and the approved stop time. No live parameter read or write was performed. [Omron I576, chapters 5, 6, 9 and appendix A](https://files.omron.eu/downloads/latest/manual/en/i576_accurax_g5_servomotors_drives_with_ethercat_control_users_manual_en.pdf?v=1)

## First connected session — motion inhibited

1. Support the load mechanically. Isolate motor power or inhibit torque by an independently verified means appropriate to the bench. Keep all motion controllers stopped. Confirm the independent stop is accessible. Only the fieldbus I226-V at PCI 57:00.0 goes to ECAT IN. Management stays on I226-LM/enp89s0.
2. With ROS stopped run `/opt/etherlab/bin/ethercat master` and `/opt/etherlab/bin/ethercat slaves -v`. Expect master 0 idle/inactive, fieldbus MAC 38:05:25:37:9f:32, link up and one bottom drive in PREOP at absolute position 0, alias 0. Record vendor 0x83, product 4, revision 0x20001 and serial (0x1018:01 through :04), plus the motor nameplate. Associate each additional drive with a physically traced axis, ring position and distinct serial before enabling it.
3. Upload 0x603F, manufacturer detail 0x4001, status 0x6041, actual position/velocity 0x6064/606C and 0x60FD. Save all readings. In Sysmac read the **effective** Pn400, 401, 402, 410, 437–439, 504 and 505 settings with their mode selections. Independently actuate one physical switch at a time. Record the full 32-bit word in the centre, at each switch and after release. Bit 0 asserted means negative prohibited and bit 1 means positive prohibited. Both asserted blocks both directions. Resolve any centre assertion and confirm the circuit response to an open wire before changing Pn504.
4. Compare SO1 wiring, driver/relay and brake coil to the machine drawing. Observe electrical output and a separately measured brake state if fitted. Write down what each measurement proves. The console's brake-release request tracks the host's servo-enable request. Output feedback and physical state are Unknown without independent sensors.
5. Read 0x6091:01/02, Pn000, Pn001, Pn801, 0x607D:01/02, Pn521, Pn730/731/753/754, 0x60E0, 0x60E1 and 0x6072. Record units, signed direction and an effective torque cap per direction. Raw 0x6077 is signed INT16 in per mille. The archived R88M-K40030L-BS2 motor family has rated shaft torque **1.3 N·m**, so −500 corresponds to −0.65 N·m. Confirm the actual installed motor. This channel is drive-reported torque. [Omron motor characteristics, I816](https://files.omron.eu/downloads/latest/datasheet/en/i816_r88m-k_r88d-kt_kn_-ml2_kn_-ect_ac_servomotor_drives_g5_series_datasheet_en.pdf?v=3)
6. Save `ethercat pdos -p 0` and live assignments 0x1C12/0x1C13. During disabled cyclic checkout verify the **configured** variable PDOs. Rx 0x1600 contains 6040:00/16, 60FF:00/32, 6060:00/8. Tx 0x1A00 contains 6041:00/16, 6064:00/32, 606C:00/32, 6061:00/8, 60FD:00/32, 6077:00/16, 603F:00/16 in order. These are 7 and 19 bytes respectively. 6077 and 603F have Tx mapping flags in the matching ESI. Confirm assignments, entries and values on the drive after remap.
7. Use the disabled checkout and ordered PREOP procedure from the host handoff. For the no-broadcaster trial add `feedback:=false`. The default starts the 1 kHz and 25 Hz state-only broadcasters. Check actual thread scheduling, OP, WKC, jitter, zero target/controlword, operation-enabled bit clear, no motion and clear errors over repeated starts. Record the SM watchdog and SYNC loss reactions with torque independently inhibited. Save evidence of stop timing and any drive or host trip.
8. With claims released, run `bash src/riser_hardware/scripts/shutdown_bottom_motor.sh` on riserms. Verify settled PREOP, inactive master and final error/detail/status before ending the launch. This helper is scoped to the dedicated one-drive bench. Diagnose a latched alarm first. The explicit fault-clear helper requires idle master, PREOP, identified drive and disabled operation. It prints before/after readings and never requests servo enable.

Example read-only upload on the verified single-drive bench, usable even with ROS stopped:

```bash
source /opt/ros/jazzy/setup.bash
source ~/riser-control/install/setup.bash
ros2 run riser_motion_bringup g5_upload 0 0 0 0x60fd 0 uint32
```

## Powered-test prerequisites

Confirm the independently supported setup, stop circuit, direction, load envelope, actual brake operation, both limit circuits, effective torque ceiling, travel and speed budgets, watchdog behavior and clean disabled trials above. Have one local operator at the independent stop. Agree on maximum displacement, stopping time and abort criteria. Use a zero-target hold first. Controller activation itself can release the brake. Use the approved 1 kHz RT configuration, verify complete feedback and record before activation. Jog only after reviewing the hold.

If the brake re-engages, keep the recording running through the transition. Examine the first change in 0x6040, 0x6041 bit 3, 0x603F, 0x6061, 0x60FD, feedback_valid and host_fault. A validity drop marks the retained feedback stale. The first host fault is latched until an explicit hardware lifecycle restart. Its numeric codes are 1 (I/O, sync or invalid command), 2 (cycle interval), 3 (lost feedback or acquisition timeout), 4 (unexpected operation disable or quick-stop state), 5 (drive fault), 6 (nonfinite feedback), 7 (former session displacement guard), 8 (measured speed) and 9 (mode mismatch). Capture 0x4001 and the drive panel alarm before reset. A host trip can coexist with a separate drive error.

## Laptop management and discovery acceptance — pending

The observed SSH session terminated on enp89s0 at the host's link-local IPv6 address. Earlier commissioning recorded a fresh laptop login on this interface. Only the originating laptop can establish whether its SSH configuration uses a jump or proxy, and whether its Jazzy environment can reach DDS (including from a VM). Use the same SSH target that normally works. The historical direct IPv4 login was `smplab@10.34.191.186`. Check the current lease. On the laptop:

```bash
export RISER_SSH=riserms
ssh -G "$RISER_SSH" | grep -E '^(hostname|user|port|proxyjump|proxycommand) '
ssh -vv -o ControlMaster=no -o ControlPath=none "$RISER_SSH" \
  'hostname; echo "$SSH_CONNECTION"; ip -br addr show enp89s0; ip route get "${SSH_CONNECTION%% *}"'
```

The verbose log distinguishes a direct TCP connection from a proxy command or jump. Verify the route to the management peer at each end. On Linux use `ip -br addr` and `ip route get PEER` (use `ip -6 route get ADDRESS oif INTERFACE` for a link-local peer). The riserms fieldbus NIC belongs to IgH and must carry no IP traffic. With only the intended management path available to ROS, use a fresh, common test domain. In the SSH terminal on riserms:

```bash
cd ~/riser-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 launch riser_motion_bringup bottom_motor.launch.py drive_backend:=mock
```

On the laptop in the Jazzy workspace, run the following. Save the middleware, interface, discovery, sample rate and service response before selecting a persistent DDS profile. If any fail, collect both ends' network namespace, interface addresses, routes, ROS environment and multicast/firewall behavior with the network operator. An SSH login confirms only the management shell path.

```bash
source /opt/ros/jazzy/setup.bash
source ~/riser-control/install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 daemon stop
ros2 node list --no-daemon --spin-time 3
ros2 topic echo --once /g5_plot/dynamic_joint_states
timeout 10 ros2 topic hz /g5_plot/dynamic_joint_states
timeout 10 ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

Keep high-rate recording local to riserms. Subscribe PlotJuggler's live ROS stream to `/g5_plot/dynamic_joint_states`. Use the confirmed SSH alias for the GUI and bag transfer. Stop the mock after checking. No persistent DDS or relay configuration was selected during disconnected development.

## Verification on riserms

The affected Jazzy C++ targets were built against the installed stock IgH headers/library. Tests cover four PDO byte regions, signed torque, both limit bits and retreat, fault latching, stale feedback, disabled output, four identical product IDs with different serials, swapped or missing drives, omitted axes, bounded profiles, cancel, heartbeat loss and calibration validity. The four-axis calibration test invalidated one ML input and preserved independent pitch angles. The balance adapter ran with the backboard and ankle-wrench subset and rejected ML and footplate modes. The offscreen PySide6 console observed separate CLI activation/deactivation, interface ownership, positive and negative mock motion and cancellation. The actual hardware plugin also passed configure, activate, command claims, shared-domain read/write and cleanup against a linked in-memory IgH implementation with one and four drives. The maintenance workflow changed an inactive mock mask, rebuilt the motion package, verified new manager PIDs, inactive profiles and a separately restored inactive controller instance, and stopped the replacement launch. The disconnected hardware launch saw link down and zero slaves, selected BottomCsvSystem, refused configuration and returned launch status 1. Master 0 was Idle/inactive afterward. The four affected packages passed 8 native test executables and 19 Python test cases.

Mock trial `20261124T205648813312Z_mock_check` finalized to MCAP plus an NPZ. It contains 5,385 full-rate messages and 134 plot messages across 6.56 s including a pause. Observed whole-trial rates were 821.0 and 20.6 Hz. Largest receipt gaps were 1.16 and 1.20 s, spanning the pause. Excluding the single pause gap for each topic gives 997.3 and 25.1 Hz respectively. The recorder reported **one transport-layer message lost**. Offline topic selection retained exactly the 134 plot samples. In a headless two-window test on riserms, PlotJuggler 3.17.2 loaded the selected MCAP, and its live ROS subscriber continued on the 25 Hz topic. The live graph showed a single publisher, the `g5_plot` broadcaster. Laptop display, management-network latency/rates/drops and all connected behavior remain to be measured.

No separate recorder script was located in this checkout or the archived Jetson tree. The host workflow wraps Jazzy rosbag2 and its pause/resume services. Supply the path or revision if a separate lab recorder needs compatibility checks. [Console, recording and calibration operation](../src/riser_motion_bringup/README.md)

## Changed files

Paths below are relative to the repository root. The workspace retains the operator's existing edits.

| Area | Files |
|---|---|
| Hardware in `src/riser_hardware/` | `CMakeLists.txt`, `package.xml`, `bottom_csv_plugins.xml`, `src/bottom_csv_system.cpp`, `src/g5_axis.hpp`, `include/riser_hardware/{csv_logic,g5_topology}.hpp`, `scripts/reset_g5_fault.sh`, `test/{csv_logic,g5_pdo,g5_system}_test.cpp`. Removed the obsolete `src/ethercat_boundary.cpp` and `plugins.xml`. |
| Controller and configuration in `src/riser_motion_bringup/` | `CMakeLists.txt`, `package.xml`, `launch/bottom_motor.launch.py`, `config/{axes.yaml,controllers.yaml,g5_plot.xml}`, `src/motion_controller.cpp`, `include/riser_motion_bringup/jog.hpp`, `srv/Jog.srv`, `test/jog_test.cpp`, `test/mock_console.py`. |
| Operator tools in `src/riser_motion_bringup/scripts/` | `g5_console`, `g5_jog`, `g5_upload`, `g5_trial`, `g5_plot_trial`, `g5_rebuild`. |
| Bringup in `src/riser_bringup/` | `CMakeLists.txt`, `package.xml`, `launch/control.launch.py`, `urdf/backends.xacro`, `scripts/g5_calibration.py`, `test/{test_g5_calibration,test_xacro}.py`. |
| Balance in `src/riser_inverted_pendulum/` | `CMakeLists.txt`, `test/mode_test.cpp`, `include/riser_inverted_pendulum/inverted_pendulum_controller.hpp`, `src/inverted_pendulum_controller.cpp`. |
| Handoff and trial storage | `README.md`, `src/riser_motion_bringup/README.md`, `reports/{g5-motion-bringup,riserms-ethercat-commissioning,riserms-realtime-setup}.md`, `.gitignore` (generated trial data). |
