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

`config/axes.yaml` defines the configured axes, enabled names, absolute ring positions, aliases, expected serials and rated motor torques. The enabled list generates both hardware resources and controller bindings. An excluded, physically present drive needs its own commissioned identity. One bottom drive at ring zero is the bench exception for an unknown serial. The multi-drive profile requires distinct commissioned serials and the same verified motor-unit scaling. Collect that evidence for the other three axes before adding them. Edit the profile with the hardware unconfigured and relaunch the manager to apply its resource list. A launch override such as `enabled_axes:=bottom,top` selects a subset. `axes_file:=/absolute/path` selects a commissioned profile.

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

Each active controller owns one velocity interface. A jog requires fresh operation-enabled feedback, signed shaft velocity, duration and sufficient ramp/travel budget. Default launch bounds are 0.1047197551 rad/s, 8 s, 0.25 s ramp, 0.45 rad/s² and 0.05 rad travel. These are bench configuration values. The host rejects requests older than 0.5 s, checks its steady-clock deadline every cycle and stops a jog after 0.3 s without its matching heartbeat. Keep laptop and host clocks synchronized. A limit cancels motion into that switch. A fresh jog away requires healthy feedback. Cancel and expiry command zero velocity. Deactivation releases the claim and requests drive shutdown. Disconnecting the laptop stops the jog by lease expiry while the active controller still owns the servo. A latched communication/drive fault inhibits re-enabling until the operator resolves it and explicitly restarts the hardware lifecycle.

## Laptop console and data

Build the Jazzy motion package on the laptop for the generated service type. The GUI uses the operator's existing SSH configuration for host-only commands, and Jazzy DDS for state, lifecycle and jog. Complete the management-network acceptance in the commissioning checklist first. The following installs Qt in a laptop virtual environment, preserving the system ROS Python packages:

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

**Motion** shows shaft radians, rad/s, torque in N·m and raw per mille, active-high reverse/forward limits, brake indications, CiA words, host fault and calibration angles in radians. During a stale interval the last received values remain labelled Stale. Use the read-only Upload SDO action with explicit index, subindex and datatype. Its host process checks identity and reports hex, mailbox abort or timeout. With the graph stopped, the console defaults to the dedicated single bottom-drive bench for readback and explicit fault clear.

**Data** starts a detached recorder on riserms with selected absolute topic names at their original rates. Record `/dynamic_joint_states` for the 1 kHz drive sequence, `/g5_plot/dynamic_joint_states` for the 25 Hz stream, and optionally `/g5_calibration/state`. Pause and resume use rosbag2 services. Stop sends SIGINT, waits for finalization, reads the bag back and reports counts, reception rates and maximum gaps. The trial log includes rosbag2 transport loss reports. Dated UTC trials live under `~/riser-control/data/`. Descriptions accept letters, digits, underscores and hyphens. List finalized trials, select one and export DynamicJointState topics to a sibling NPZ file. Each named channel has source nanoseconds and unit-labelled values. Status/bit fields are numeric. Transfer the selected trial using the displayed scp command, or append `.npz` to the remote trial path to copy the export.

Live PlotJuggler uses the laptop's live ROS domain. Accept the streaming prompt and confirm the preselected 25 Hz topic in the ROS topic dialog. Drag the desired feedback channels into the plot. For historical plots, select topics and open the transferred bag in a separate window. The reader filters a temporary MCAP offline and launches PlotJuggler in a different localhost domain. Bag playback publishes no messages to the live graph. The laptop needs PlotJuggler with the ROS 2 bag and streaming plugins.

**System** polls controller-manager services, node discovery and the selected typed parameter. CLI actions and output are copyable. Use load/configure/activate/deactivate/unload explicitly. Configuration changes take effect on controller reconfiguration, and the GUI honours parameter descriptors.

For maintenance, deactivate command owners and complete the ordered PREOP shutdown on the real bottom bench. Leave the launch running with hardware unconfigured. Select the package and choose **Rebuild / restart inhibited**. The host verifies the owning G5 launch, ROS domain, unclaimed interfaces and hardware lifecycle, stops the old process, builds the package and restarts with motion inhibited. It checks the new manager PID and inactive motion profile, and prints the new launch log path. A failed build leaves the old manager stopped. If the manager was already stopped when rebuilding, it stays stopped. The helper restores additional loaded instances of the selected package with their parameter snapshot, configures them and verifies that they are inactive. A package with no previous instance can be loaded later with compatible feedback. Re-enter the supervised powered procedure for any subsequent motion.

**Apply inactive mask** performs the same controlled restart without rebuilding. Release command claims first. On a mock use **Inactivate mock hardware**. On the real bench complete the ordered shutdown. The helper preserves existing launch arguments, replaces the enabled mask and starts motion profiles inactive. **Stop released manager** terminates the identified launch after checking the same preconditions. The CLI equivalents are `g5_rebuild --package riser_motion_bringup`, `g5_rebuild --mask bottom,top` and `g5_rebuild --stop` under `ros2 run riser_motion_bringup`.

## Calibration and balance

For a bench-only launch, start the passive observer explicitly with `ros2 run riser_bringup g5_calibration.py`. The real full launch includes it. Capture the stationary negative and positive switch positions and the independently established flat/vertical/centred zero with the Motion buttons. Each capture requires fresh healthy feedback and the corresponding switch combination. No calibration service commands a motor. The observer exports `/g5_calibration/state` and protected `result.<axis>_motor.{zero,negative_travel,positive_travel,units,valid}` parameters. A missing/stale/faulted axis invalidates its session references.

The existing powered Gazebo procedure drives **position targets in metres**, finds both switches, and commands home at a quarter of bottom stroke or half of the other strokes. Physical G5 CSV uses **motor-shaft radians**. Verify the signed metres per shaft radian, which physical switch defines each endpoint, and how the operator measures flat or vertical before authorizing automatic travel. The observer accepts a startup parameter such as `bottom_motor.metres_per_rad` only from verified commissioning data. Zero leaves the angle unavailable. Calibrated shaft displacement passes through the existing CAD branch and neutral offset for pitch. ML requires both calibrated slides and the documented height projection. No encoder calibration survives a feedback-session loss.

The inverted-pendulum controller's `mode` is `backboard`. It claims just backboard velocity and uses measured backboard state plus the ankle wrench provider. The existing equation produces only backboard acceleration and rate. Requests for `footplate` or `ml` fail configuration with a reason. Bottom-only shaft velocity cannot implement a footplate balance law without its specified equation and an identified transmission.

## Disconnected checks

```bash
colcon test --packages-select riser_hardware riser_motion_bringup riser_bringup riser_inverted_pendulum
colcon test-result --verbose
# From the Qt environment, select an unused local test domain
ROS_DOMAIN_ID=126 python src/riser_motion_bringup/test/mock_console.py
```
