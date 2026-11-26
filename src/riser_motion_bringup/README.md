# G5 motor commissioning

Build on riserms against the commissioned stock IgH installation. The real entry point is `ros2 launch riser_bringup control.launch.py backend:=ethercat`. It starts the G5 backend, read-only calibration observer and two feedback broadcasters. No automatic fault reset or motion claim occurs. `riser_motion_bringup bottom_motor.launch.py` runs just the bench hardware and controller manager. Normal simulation continues to use `backend:=mock` or `backend:=gazebo` in riser_bringup.

```bash
cd ~/riser-control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select riser_hardware --cmake-args -DRISER_WITH_IGH=ON \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so
colcon build --packages-up-to riser_bringup
source install/setup.bash
```

[First connected session, audit and powered-test prerequisites](../../reports/g5-motion-bringup.md)

## Profiles and jogs

`config/axes.yaml` defines the configured axes, enabled names, absolute ring positions, aliases, expected serials and rated motor torques. It also supplies the carriage pitch (0.010 m/rev), the commissioning limits (0.020 m/s, 0.160 m travel, 0.08 m/s²), and the 0.001 m/s calibration speed. Launch passes the shared limits to the G5 consumers. The enabled list generates both hardware resources and controller bindings. An excluded, physically present drive needs its own commissioned identity. One bottom drive at ring zero is the bench exception for an unknown serial. The multi-drive profile requires distinct commissioned serials and the same verified motor-unit scaling. Collect that evidence for the other three axes before adding them. Edit the profile with the hardware unconfigured and relaunch the manager to apply its resource list. A launch override such as `enabled_axes:=bottom,top` selects a subset. `axes_file:=/absolute/path` selects a commissioned profile.

The real default sends a zero controlword and zero target. `commissioning_disabled:=0` makes velocity claims possible and loads each motion controller inactive. **Activation requests servo enable and can release the brake, even before the first jog.** Use the powered setting only after completing the connected checklist. The real manager defaults to the host commissioning settings of FIFO 80, CPU 3, memory locking and two allocator arenas. A supplied controller file can override those manager settings.

Use the explicit G5 mock for development:

```bash
ros2 launch riser_motion_bringup bottom_motor.launch.py drive_backend:=mock \
  enabled_axes:=bottom,top,torso,hip commissioning_disabled:=0
# In the same ROS domain
ros2 control set_controller_state motor_profile active
ros2 run riser_motion_bringup g5_jog --axis bottom --velocity -0.01 --duration 1
ros2 run riser_motion_bringup g5_jog --axis bottom --cancel
ros2 control set_controller_state motor_profile inactive
```

Each active controller owns one velocity interface. A jog requires fresh operation-enabled feedback, signed carriage velocity in m/s, duration and sufficient ramp/travel budget. The default ramp is 0.25 s, the maximum duration is 10 s, and the speed-times-duration check also applies. The 0.160 m budget is checked for each jog and for calibration travel while seeking the second switch. A displacement limit for the entire hardware session is pending. The host rejects requests older than 0.5 s, checks its steady-clock deadline every cycle and stops a jog after 0.6 s without its matching heartbeat. Keep laptop and host clocks synchronized. A limit cancels motion into that switch. A fresh jog away requires healthy feedback. Cancel and expiry command zero velocity. Deactivation releases the claim and requests drive shutdown. Disconnecting the laptop stops the jog by lease expiry while the active controller still owns the servo. A latched communication/drive fault inhibits re-enabling until the operator resolves it and explicitly restarts the hardware lifecycle.

If a jog is rejected after activation, read the host's response in the console
output or `g5_jog`. A negative signed age means the stamp is ahead of the host
ROS clock at receipt. The host allows up to 5 ms of future skew.
An age above 500 ms can come from clock skew or request
delay. On Linux, run `timedatectl status` on both machines and verify they use
the intended ROS clock (`use_sim_time` is normally false on hardware). Limit
rejections include the values used by the configured controller. Compare those
with `ros2 param dump /motor_profile` (or the selected axis profile). An edit
while inactive takes effect when the controller is configured again.

## Laptop console and data

Build and source the updated Jazzy motion package on both the robot host and laptop for the metre-based jog service. The GUI uses the operator's existing SSH configuration for host-only commands, and Jazzy DDS for state, lifecycle and jog. Complete the management-network acceptance in the commissioning checklist first. The following installs Qt in a laptop virtual environment, preserving the system ROS Python packages:

```bash
cd ~/riser-control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to riser_motion_bringup
source install/setup.bash
python3 -m venv --system-site-packages ~/.venvs/riser-qt
source ~/.venvs/riser-qt/bin/activate
python -m pip install PySide6-Essentials==6.8.3
export RISER_SSH=riserms       # use the tested laptop SSH alias or user@host
ros2 run riser_motion_bringup g5_console --ssh-host "$RISER_SSH"
```

**Motion** shows carriage position in metres, velocity in m/s, torque in N·m and raw per mille, active-high reverse/forward limits, brake indications, CiA words, host fault and calibration angles in radians. During a stale interval the last received values remain labelled Stale. Use the read-only Upload SDO action with explicit index, subindex and datatype. Its host process checks identity and reports hex, mailbox abort or timeout. With the graph stopped, the console defaults to the dedicated single bottom-drive bench for readback and explicit fault clear.

**Data** starts a detached recorder on riserms with selected absolute topic names at their original rates. Record `/dynamic_joint_states` for the 1 kHz drive sequence, `/g5_plot/dynamic_joint_states` for the 25 Hz stream, and optionally `/g5_calibration/state`. Pause and resume use rosbag2 services. Stop sends SIGINT, waits for finalization, reads the bag back and reports counts, reception rates and maximum gaps. The trial log includes rosbag2 transport loss reports. Dated UTC trials live under `~/riser-control/data/`. Descriptions accept letters, digits, underscores and hyphens. List finalized trials, select one and export DynamicJointState topics to a sibling NPZ file. Each named channel has source nanoseconds and unit-labelled values. The recorder tags new bags with motor units in the metadata. For an unmarked bag, use `g5_trial export TRIAL --motor-units rad` for historical shaft data or `--motor-units m` for metre data recorded separately. Status/bit fields are numeric. Transfer the selected trial using the displayed scp command, or append `.npz` to the remote trial path to copy the export.

Live PlotJuggler uses the laptop's live ROS domain. Accept the streaming prompt and confirm the preselected 25 Hz topic in the ROS topic dialog. Drag the desired feedback channels into the plot. Live carriage position and velocity are in m and m/s. Historical bag plots use the values as recorded. For historical plots, select topics and open the transferred bag in a separate window. The reader filters a temporary MCAP offline and launches PlotJuggler in a different localhost domain. Bag playback publishes no messages to the live graph. The laptop needs PlotJuggler with the ROS 2 bag and streaming plugins.

**System** polls controller-manager services, node discovery and the selected typed parameter. CLI actions and output are copyable. Use load/configure/activate/deactivate/unload explicitly. Configuration changes take effect on controller reconfiguration, and the GUI honours parameter descriptors.

For maintenance, deactivate command owners and complete the ordered PREOP shutdown on the real bottom bench. Leave the launch running with hardware unconfigured. Select the package and choose **Rebuild / restart inhibited**. The host verifies the owning G5 launch, ROS domain, unclaimed interfaces and hardware lifecycle, stops the old process, builds the package and restarts with motion inhibited. It checks the new manager PID and inactive motion profile, and prints the new launch log path. A failed build leaves the old manager stopped. If the manager was already stopped when rebuilding, it stays stopped. The helper restores additional loaded instances of the selected package with their parameter snapshot, configures them and verifies that they are inactive. A package with no previous instance can be loaded later with compatible feedback. Re-enter the supervised powered procedure for any subsequent motion.

**Apply inactive mask** performs the same controlled restart without rebuilding. Release command claims first. On a mock use **Inactivate mock hardware**. On the real bench complete the ordered shutdown. The helper preserves existing launch arguments, replaces the enabled mask and starts motion profiles inactive. **Stop released manager** terminates the identified launch after checking the same preconditions. The CLI equivalents are `g5_rebuild --package riser_motion_bringup`, `g5_rebuild --mask bottom,top` and `g5_rebuild --stop` under `ros2 run riser_motion_bringup`.

## ELM3604 checkout

Add `force_plates:=true` to the EtherCAT launch. For a feedback-only
checkout, leave `commissioning_disabled:=1`. The ELM uses a separate input
domain with its existing one-sample PDO maps. X1–X3 use DC-coupled
0–10 V Extended Range with IEPE bias off.
Connect the G5 output to the EK1100 port marked X1 IN.

X1, X2 and X3 fill the left plate's Mx, Fz and Fy inputs respectively.
The other inputs are zero. Counts are converted to µV/V using
`counts × 1.28 / (gain × excitation volts)` before applying the left
inverse-sensitivity matrix. The MSA-6 gain and excitation entries in
`force_plate_wrench_calibration.yaml` are provisional jumper settings in
X1, X2, X3 order. The read publishes all six results about the factory
transducer origin. Cross-terms can produce nonzero outputs for unwired
axes. The right wrench is zero. Each plate's `live_mask`
encodes wiring in Fx, Fy, Fz, Mx, My, Mz order from bit 0. The left mask is
14 and the right mask is 0. `valid` checks the ELM samples. The existing
`/dynamic_joint_states` and `/g5_plot/dynamic_joint_states` carry these states.

On 2026-11-26 with the G5 first and the ELM last, inhibited 60-second
checks logged zero incomplete G5 PDO cycles both without ELM acquisition
(60,054 OP cycles) and with it (60,266). All three devices reached OP in
the combined run. The sampled working counters were 3/3 and 1/1. X2 and
X3 varied while X1 held at 0 and showed underrange. The bus returned to
PREOP with no G5 alarm.

## Calibration and balance

The C++ `calibration_controller` claims the enabled `*_motor/velocity` interfaces
and reads carriage position, velocity, ready, feedback validity, and the two
directional switches. The G5 bench launch loads it inactive with the motion
profiles when `commissioning_disabled:=0` or `load_motion_inactive:=true`.
The full EtherCAT launch also accepts `calibrate:=true` to load it inactive
with the default drive inhibit. Use an explicit controller switch after
releasing any conflicting velocity claims.

With healthy operation-enabled feedback, calibration seeks the reverse switch,
then the forward switch, and commands return to their measured midpoint.
Each asserted switch sample captures an endpoint, including early manual
assertions. The controller writes zero on each transition before moving in
the opposite direction on the next update. It reports
`result.<axis>_motor.{negative_travel_m,positive_travel_m,valid}`.

For a bench-only launch, start the passive observer explicitly with
`ros2 run riser_bringup g5_calibration.py`. The full EtherCAT launch includes it.
The Motion **Capture** buttons record stationary negative and positive switch
positions and an independently established flat, vertical or centred zero.
The observer exports `/g5_calibration/state` and protected
`result.<axis>_motor.{zero,negative_travel,positive_travel,units,valid}`
parameters in metres. It uses measured carriage displacement in the CAD
mapping. The derived pitch and ML angles are in radians. A missing, stale or
faulted axis invalidates its session references.

The inverted-pendulum controller's `mode` is `backboard`. It claims just backboard velocity and uses measured backboard state plus the ankle wrench provider. The existing equation produces only backboard acceleration and rate. Requests for `footplate` or `ml` fail configuration with a reason. Bottom-only carriage velocity cannot implement a footplate balance law without its specified equation and an identified transmission.

## Disconnected checks

```bash
colcon test --packages-select riser_hardware riser_motion_bringup riser_bringup riser_inverted_pendulum
colcon test-result --verbose
# From the Qt environment, select an unused local test domain
ROS_DOMAIN_ID=126 python src/riser_motion_bringup/test/mock_console.py
```
