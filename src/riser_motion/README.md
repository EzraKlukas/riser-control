# G5 motor commissioning

Build on riserms against the commissioned stock IgH installation. The EtherCAT entry point starts the G5 backend and the feedback broadcasters with drive startup inhibited. `riser_motion bottom_motor.launch.py` exposes the hardware and controller configuration arguments directly.

```bash
cd ~/riser-control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to riser_hardware --cmake-args -DRISER_WITH_IGH=ON \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so
colcon build --packages-up-to riser_bringup
source install/setup.bash
```

[First connected session, audit and powered-test prerequisites](../../reports/g5-motion-bringup.md)

## Bounded motion

Gazebo and EtherCAT run the same calibration and profile controllers. Motor
profiles claim `*_motor/velocity` or `*_motor/position` in m/s or metres.
Angle profiles claim the selected mechanism's `velocity` or `position` in
rad/s or radians. Gazebo supplies both modes. EtherCAT exports carriage
velocity only, so `bottom_motor/position` and the other real linear position
commands are unavailable. Real footplate velocity uses the transmission
Jacobian. Real footplate position uses the existing closed carriage-position
loop over CSV, enabled only with a positive, commissioned
`footplate_position_gain_s_inv`. The real backboard and ML angle contracts
are unavailable.

### Local Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch riser_bringup control.launch.py backend:=gazebo plot:=false wrench_input:=false
# A second sourced terminal on the same ROS domain
ros2 run riser_motion g5_console --local-gazebo
```

For a bottom-only session, add `enabled_axes:=bottom` to the Gazebo launch.
The console offers `bottom_motor` and `foot_plate_pitch`, and calibration
sweeps only the bottom motor. Use comma-separated names for other combinations,
for example `enabled_axes:=torso,hip,bottom`. Both lateral motors are needed
for ML angle control. Gazebo defaults to all four motors.

1. In **Motion**, pick `bottom_motor`, **velocity**, and **Activate selected
   controller**. Pick **jog** and enter `0.005` and `1.0` for velocity and
   duration. Click **Start**, then **Cancel** or let the jog finish.
2. Click **Deactivate selected controller**. In **System**, enter
   `calibration_controller` and click **Activate**. Wait for all four
   `midpoint established` messages and `foot_plate_pitch/valid = 1`.
   The **Motion** readout shows this validity when the footplate is selected.
3. Switch explicitly to the footplate velocity controller with the command
   below, or deactivate calibration in **System** and activate
   `foot_plate_pitch` / **velocity** in **Motion**. Try another jog, or pick
   **triangle** with `0.008` rad/s and `2.0` s. Ramp also uses the **Ramp**
   field for its rise and fall. Sine runs one full signed cycle, and triangle
   rises and falls without changing direction.
4. Pick **CSV**, then **Choose / preview CSV**. Check the accepted row count,
   duration and displayed rows before pressing **Start**. Deactivate the
   active owner before switching to the **position** controller. Repeat with
   the position example on either the footplate or a Gazebo carriage.

```bash
ros2 control switch_controllers --strict \
  --deactivate calibration_controller --activate angle_profile
```

Watch or record `/g5_plot/dynamic_joint_states`. Plot the selected axis'
`position` and `velocity`, both directional switches and either
`feedback_valid` (motor) or `valid` (angle). The ros2_control introspection
topics `/controller_manager/introspection_data/{names,values}` provide
`<profile>.command` for the commanded position or rate and
`<profile>.playback_active` for playback status. The physical footplate pitch is also on
`/riser/physical_joint_states`.

### Motion CSV format

- [Position example](config/motion_position.csv) uses `time_s,position`.
  Values are relative to the held position setpoint at **Start**. Activation
  and cancellation seed the hold from measured feedback. Using the held
  setpoint keeps **Start** continuous under steady tracking error. Each
  segment uses cubic smoothstep interpolation with zero endpoint rates.
- [Velocity example](config/motion_velocity.csv) uses `time_s,velocity`.
  Values are signed rates and linearly interpolated. The first and last
  velocities must be zero.
- Time starts at zero, increases strictly and is in seconds. The first value
  is zero in both formats. Use metres and m/s for carriages, radians and
  rad/s for angles. The validator checks finite samples, duration, speed,
  acceleration and travel. The live controller checks feedback, direction,
  timing and heartbeat. Position cancellation holds the measured position
  when feedback is healthy. Velocity cancellation requests zero.
- [Rejected example](config/motion_rejected.csv) repeats a timestamp. The
  same controller validates preview and execution. The GUI sends the exact
  previewed CSV content to the execution host again at **Start**. Motion
  CSVs are separate from [ankle-wrench perturbation CSVs](../riser_bringup/config/perturbation-tutorial.md).

The Gazebo-only check exercises the failure cases as well as motion. Run it
against a fresh isolated launch on the same local ROS domain:

```bash
python3 src/riser_bringup/test/gazebo_motion.py
```

### Laptop GUI to riserms with startup inhibited

Start in a sourced terminal on riserms after the IgH build:

```bash
ros2 launch riser_motion bottom_motor.launch.py drive_backend:=real \
  commissioning_disabled:=1 load_motion_inactive:=true force_plates:=true
```

On the laptop, use the same ROS domain and DDS configuration and open
`ros2 run riser_motion g5_console --ssh-host riserms`. In **Motion**, select
`bottom_motor` and check the live carriage fields. **Choose / preview CSV**
can validate a velocity file against the inactive riserms controller. In
**System**, confirm that calibration, profile and balance controllers are
inactive when configured. Add `footplate_direction:=1` or
`footplate_direction:=-1` once the encoder sign is known to include the
footplate and balance controllers. Keep the inhibit for this checkout. Profile activation is expected
to fail while inhibited. The **System** and **Data** tabs retain host
maintenance and trial operations over SSH.

## Profiles and jogs

`config/axes.yaml` defines the configured axes, enabled names, absolute ring positions, aliases, expected serials and rated motor torques. It also supplies the carriage pitch (0.010 m/rev), the commissioning limits (0.10 m/s, 0.180 m travel, 0.5 m/s²), and the 0.01 m/s calibration speed. Launch passes the shared limits to the G5 consumers. The enabled list generates both hardware resources and controller bindings. An excluded, physically present drive needs its own commissioned identity. One bottom drive at ring zero is the bench exception for an unknown serial. The multi-drive profile requires distinct commissioned serials and the same verified motor-unit scaling. Collect that evidence for the other three axes before adding them. Edit the profile with the hardware unconfigured and relaunch the manager to apply its resource list. A launch override such as `enabled_axes:=bottom,top` selects a subset. `axes_file:=/absolute/path` selects a commissioned profile.

The real default sends a zero controlword and zero target. `commissioning_disabled:=0` makes velocity claims possible and loads each motion controller inactive. **Activation requests servo enable and can release the brake, even before the first jog.** Use the powered setting only after completing the connected checklist. The real manager defaults to the host commissioning settings of FIFO 80, CPU 3, memory locking and two allocator arenas. A supplied controller file can override those manager settings.

Use the explicit G5 mock for development:

```bash
ros2 launch riser_motion bottom_motor.launch.py drive_backend:=mock \
  enabled_axes:=bottom,top,torso,hip commissioning_disabled:=0
# In the same ROS domain
ros2 control set_controller_state motor_profile active
ros2 run riser_motion g5_jog --axis bottom --velocity -0.01 --duration 1
ros2 run riser_motion g5_jog --axis bottom --cancel
ros2 control set_controller_state motor_profile inactive
```

Each active controller owns one velocity interface. A jog requires fresh operation-enabled feedback, signed carriage velocity in m/s, duration and sufficient ramp/travel budget. The default ramp is 0.25 s, the maximum duration is 10 s, and the speed-times-duration check also applies. The configured travel budget is checked for each jog and for calibration travel while seeking the second switch. A displacement limit for the entire hardware session is pending. The host rejects requests older than 0.5 s, checks its steady-clock deadline every cycle and stops a jog after 0.6 s without its matching heartbeat. Keep laptop and host clocks synchronized. A limit cancels motion into that switch. A fresh jog away requires healthy feedback. Cancel and expiry command zero velocity. Deactivation releases the claim and requests drive shutdown. Disconnecting the laptop stops the jog by lease expiry while the active controller still owns the servo. A latched communication/drive fault inhibits re-enabling until the operator resolves it and explicitly restarts the hardware lifecycle.

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
colcon build --packages-up-to riser_motion
source install/setup.bash
python3 -m venv --system-site-packages ~/.venvs/riser-qt
source ~/.venvs/riser-qt/bin/activate
python -m pip install PySide6-Essentials==6.8.3
export RISER_SSH=riserms       # use the tested laptop SSH alias or user@host
ros2 run riser_motion g5_console --ssh-host "$RISER_SSH"
```

**Motion** shows carriage position in metres, velocity in m/s, torque in N·m and raw per mille, active-high reverse/forward limits, brake indications, CiA words, host fault and calibration angles in radians. During a stale interval the last received values remain labelled Stale. Use the read-only Upload SDO action with explicit index, subindex and datatype. Its host process checks identity and reports hex, mailbox abort or timeout. With the graph stopped, the console defaults to the dedicated single bottom-drive bench for readback and explicit fault clear.

**Data** starts a detached recorder on riserms with selected absolute topic names at their original rates. Record `/dynamic_joint_states` for the 1 kHz drive sequence, `/g5_plot/dynamic_joint_states` for the 25 Hz stream, and optionally `/g5_calibration/state`. Pause and resume use rosbag2 services. Stop sends SIGINT, waits for finalization, reads the bag back and reports counts, reception rates and maximum gaps. The trial log includes rosbag2 transport loss reports. Dated UTC trials live under `~/riser-control/data/`. Descriptions accept letters, digits, underscores and hyphens. List finalized trials, select one and export DynamicJointState topics to a sibling NPZ file. Each named channel has source nanoseconds and unit-labelled values. The recorder tags new bags with motor units in the metadata. For an unmarked bag, use `g5_trial export TRIAL --motor-units rad` for historical shaft data or `--motor-units m` for metre data recorded separately. Status/bit fields are numeric. Transfer the selected trial using the displayed scp command, or append `.npz` to the remote trial path to copy the export.

Live PlotJuggler uses the laptop's live ROS domain. Accept the streaming prompt and confirm the preselected 25 Hz topic in the ROS topic dialog. Drag the desired feedback channels into the plot. Live carriage position and velocity are in m and m/s. Historical bag plots use the values as recorded. For historical plots, select topics and open the transferred bag in a separate window. The reader filters a temporary MCAP offline and launches PlotJuggler in a different localhost domain. Bag playback publishes no messages to the live graph. The laptop needs PlotJuggler with the ROS 2 bag and streaming plugins.

**System** polls controller-manager services, node discovery and the selected typed parameter. CLI actions and output are copyable. Use load/configure/activate/deactivate/unload explicitly. Configuration changes take effect on controller reconfiguration, and the GUI honours parameter descriptors.

For maintenance, deactivate command owners and complete the ordered PREOP shutdown on the real bottom bench. Leave the launch running with hardware unconfigured. Select the package and choose **Rebuild / restart inhibited**. The host verifies the owning G5 launch, ROS domain, unclaimed interfaces and hardware lifecycle, stops the old process, builds the package and restarts with motion inhibited. It checks the new manager PID and inactive motion profile, and prints the new launch log path. A failed build leaves the old manager stopped. If the manager was already stopped when rebuilding, it stays stopped. The helper restores additional loaded instances of the selected package with their parameter snapshot, configures them and verifies that they are inactive. A package with no previous instance can be loaded later with compatible feedback. Re-enter the supervised powered procedure for any subsequent motion.

**Apply inactive mask** performs the same controlled restart without rebuilding. Release command claims first. On a mock use **Inactivate mock hardware**. On the real bench complete the ordered shutdown. The helper preserves existing launch arguments, replaces the enabled mask and starts motion profiles inactive. **Stop released manager** terminates the identified launch after checking the same preconditions. The CLI equivalents are `g5_rebuild --package riser_motion`, `g5_rebuild --mask bottom,top` and `g5_rebuild --stop` under `ros2 run riser_motion`.

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
axes. The EtherCAT aggregation selects the left plate. Its `live_mask` is 14,
encoding the wired axes in Fx, Fy, Fz, Mx, My, Mz order from bit 0. `valid` requires a fresh sample from each of
X1–X3. General error, underrange, overrange and invalid PDO bits block
validity. The diagnostic-message bit alone leaves the sample usable.
`BottomMotor/read_time` is the most recent valid acquisition in host system
seconds and stays unchanged on stale PDOs. The existing
`/dynamic_joint_states` and `/g5_plot/dynamic_joint_states` carry these states.

On 2026-11-26 with the G5 first and the ELM last, inhibited 60-second
checks logged zero incomplete G5 PDO cycles both without ELM acquisition
(60,054 OP cycles) and with it (60,266). All three devices reached OP in
the combined run. The sampled working counters were 3/3 and 1/1. X2 and
X3 varied while X1 held at 0 and showed underrange. The bus returned to
PREOP with no G5 alarm.

In the pitch-interface checkout, the original mask produced zero valid
left samples. X1/X2 reported `0x1001` and X3 intermittently `0x1201`,
excluding the cycle-counter bits. With the diagnostic flag handled
separately, the inhibited runs delivered 4,061 and 7,112 valid samples.
X3 still reported underrange. No load calibration was performed.
The G5 recorded ten isolated incomplete PDO cycles during startup in one
ELM-enabled run. The paired repeat recorded zero during 16,084 OP cycles
with acquisition off and 19,783 with acquisition on. PREOP failed to
settle before the final test teardown and the G5 latched communications
synchronization alarm 83.3 (`0x603F=0xFF83`, `0x4001=0xF383`). All three
slaves subsequently reached PREOP with the master idle. The explicit
`reset_g5_fault.sh` reset cleared both error registers and returned
statusword `0x0270`. Servo enable stayed off in every run.

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

A complete run installs the measured midpoint as mechanical zero in the
backend. Read back `foot_plate_pitch/{position,velocity,valid,reverse_limit,forward_limit}`.
Positive footplate angle and rate raise the toes. The active-high forward
switch blocks motion in that direction.
An incomplete or failed run, hardware restart, or loss of the encoder reference
invalidates angular control. Carriage feedback remains independently available
when the drive sends valid feedback. Verify setup level at the measured
midpoint as a separate commissioning check.

Before any powered real angular test, independently verify carriage
polarity. Use `footplate_direction:=1` when positive bottom carriage motion
raises the toes and `footplate_direction:=-1` when it lowers them. This setting
relates signed encoder travel to CAD carriage stroke. Keep `footplate_position_gain_s_inv` zero until
the existing position loop has been commissioned. Confirm drive enable and
brake sequencing, switch polarity, loss of feedback, midpoint level, the
toes-up transmission sign, and fresh force acquisition under supervision.

The default inverted-pendulum mode is `footplate`. It reads the ankle wrench
and calibrated footplate position and velocity and commands footplate
angular velocity. It projects ankle torque into the selected angle coordinate
and the model subtracts the projected torque. Position
outside `lower_pendulum_angle_rad` and `upper_pendulum_angle_rad` stops the
controller. Calibration and all motion profiles must release the mechanism
before balance claims it.

Bottom-only operation supports a direct strict switch from calibration to
balance once the midpoint is established and `ankle_wrench` is active. Set
`mass` in kilograms while the pendulum is inactive, using **System** in the GUI
or the CLI below. The next activation uses this value.

```bash
# Example for a known 70 kg mass
ros2 param set /inverted_pendulum mass 70.0
ros2 control switch_controllers --strict \
  --deactivate calibration_controller --activate inverted_pendulum
```

Set `mass` to `0.0` to use the measured value. Tare the empty, level plate
with `/ankle_wrench/tare` and then call `/ankle_wrench/measure_mass` with the
participant standing on the level plate. If calibration is already inactive,
use only `--activate inverted_pendulum` in the switch request.
