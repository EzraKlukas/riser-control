# Runtime configuration

| File | Purpose |
| --- | --- |
| `control_coordinates.json` | Public joint order, lateral mapping, operating limits, servo gains and physical-state publication rate. |
| `controllers.yaml` | Controller types, interfaces, wrench geometry/transforms/gains and controller-manager update rate. |
| `force_plate_wrench_calibration.yaml` | Factory calibration and transducer offsets for physical bridge signals; reference data, not a ROS parameter file. |
| `force_plate_bridge.yaml` | Gazebo-to-ROS force-sensor topic and message mappings. |
| `riser.rviz` | RViz display settings for `view.launch.py`. |

Commands support position and velocity. State feedback exposes position, velocity
and effort. Servo gains and limits are provisional simulation values.

## Registering a controller

Register the instance name and exported plugin type under
`controller_manager.ros__parameters`. Give its node a block of the same name and
pass that file to the spawner with `--param-file`. The controller's interface
configuration methods define its claims. Load controllers with competing claims
inactive and switch them explicitly.

The [G5 launch](../../riser_motion_bringup/launch/bottom_motor.launch.py)
generates the registration and parameters for `calibration_controller` from
`axes.yaml` and loads it inactive. The motion bringup package declares the
plugin as a runtime dependency.

The controller package must separately export its C++ class with
`PLUGINLIB_EXPORT_CLASS`, describe it in `plugins.xml`, call
`pluginlib_export_plugin_description_file(controller_interface plugins.xml)` in
CMake, and install its shared library. See [riser_calibration](../../riser_calibration)
for those files. A YAML `type` entry alone cannot register the C++ plugin.

Rebuild and source the workspace after changing registration or installation:

```bash
colcon build --packages-up-to riser_bringup
source install/setup.bash
ros2 control list_controller_types
ros2 control list_controllers
```

The last two commands require a running controller manager. See the
[homing instructions](../../../README.md#actuator-homing-in-gazebo) for launch
and strict controller-switch commands. The Gazebo adapter exports lower and
upper limit-switch state interfaces for all four actuators.

Model geometry, kinematics, rigid-body properties and reduced pitch dynamics live
in `../urdf/model`; plate properties are in `../urdf/riser.urdf.xacro`. Simulation
world `../worlds/riser_world.sdf` is used by launch. Set the force-sensor rate
with `force_plate_rate:=...`.

`reduced_pitch.json` stores full-precision samples in the order listed by
`columns`: angle, inertia, inertia derivative, gravity torque and damping.
Keep the uniform angle grid and columns consistent when changing the table.
Its inertial feedback uses acceleration delayed by one physics step. See
[CAD model derivation](../../../reports/model.md).

## Ankle wrench

Gazebo bringup activates `ankle_wrench`, followed by
`ankle_wrench_state_broadcaster`. Other backends do not yet provide force-plate
states. The provider claims both plates' six wrench components, their
`valid` flags, the common read time and footplate pitch. It exports the eight wrench fields and
`subject_mass_kg` under `ankle_wrench/`.
The consumer publishes all eight values on
`/ankle_wrench_state_broadcaster/dynamic_joint_states` for ROS tools and PlotJuggler.
It uses local topics to keep these diagnostic values separate from `/joint_states`.

The output origin is the approximate ankle midpoint, at the foot-platform pitch
pivot. Its **physical axes rotate with the foot platform**: X left, Y forward,
Z down at neutral. This is an explicit controller convention; the paper does
not explicitly identify a rotating ankle frame. The two fixed pivot-to-surface
vectors in `controllers.yaml` use these axes and metres. No TF lookup or pose
state is needed for this platform-attached output. To express the result in fixed
robot/world axes later, rotate both the net force and net torque using foot pitch.
Do not label the present components as fixed `base_link` axes.

Processing order in `WrenchAggregationLogic::compute_wrench()`:

1. Optional custom hook (currently a no-op).
2. Optional row-major 3×3 matrix, applied to force and torque separately.
   `same_transformation=true` selects `symmetric_wrench_transformation` for both
   plates; false selects `left_wrench_transformation` and `right_wrench_transformation`.
3. Optional scalar gain on all six components. Independently,
   `same_scaling=true` selects `ankle_wrench_gain` for both plates; false selects
   `left_wrench_gain` and `right_wrench_gain`.
4. Sum forces and `plate torque + (ankle-to-plate arm × plate force)`.
5. Subtract the latest tare from all six ankle wrench components.
6. Add any enabled AP perturbations to `torque.x` just before export.

Both perturbations default to off. The
[AP perturbation tutorial](perturbation-tutorial.md) covers the parameters,
playback and multisine services, and before/after torque plotting.

Thus a shared matrix with different gains uses `same_transformation=true` and
`same_scaling=false`. Matrix entries are arbitrary experimental operations on
physical components, **not a change of coordinate frame**; geometric moment arms
remain in the physical axes. Defaults are identity matrices and unit gains.
The tare removes a constant offset in platform axes. Gravity and inertial
loads can still change as the platform moves.

The combined wrench carries one `sample_time`, the time the backend read both
plates. `read_time_interface` selects `RiserSystem/read_time` in Gazebo. An
empty setting uses the controller callback time. Gazebo reports a missing or
stale plate through its `valid` flag. Missing or nonfinite inputs yield NaN
and `valid=0` at the ankle until the inputs recover.
Updates use fixed storage without allocation, blocking I/O or logging; this does
not guarantee scheduling latency or hard realtime execution.

### Tare and weigh before an experiment

With the inverted pendulum inactive, keep the footplate flat and the plates
empty for the first call. For the second call, have the participant stand still
with their full weight supported by the plates. For the third call, have them
hold the upright posture that should define zero for the experiment.

```bash
ros2 service call /ankle_wrench/tare std_srvs/srv/SetBool '{data: true}'
ros2 service call /ankle_wrench/measure_mass std_srvs/srv/Trigger '{}'
ros2 param get /ankle_wrench subject_mass_kg
ros2 service call /ankle_wrench/tare std_srvs/srv/SetBool '{data: false}'
```

Each service replies after averaging one second of valid hardware reads.
Keep the simulation running during capture. Empty tare and weighing
require measured pitch within 0.01 rad of flat. Weighing divides the magnitude
of the mean vertical force above the empty baseline by 9.81 m/s². It assumes
the wrench transforms and gains preserve a calibrated vertical force.
A standing tare uses the mean of the six pre-tare ankle components, so taring
twice replaces the bias without accumulating it. The empty reference and
participant mass survive the standing tare. An empty tare or reconfiguration
clears the previous mass. Zero means no measurement is available.

After the platform settles at its starting pose, complete the procedure above.
Load the experimental controller inactive and switch to it. The controller's angular velocity claims select the
new mode automatically.

```bash
ros2 run controller_manager spawner inverted_pendulum --inactive
ros2 control switch_controllers --strict \
  --deactivate riser_position_controller --activate inverted_pendulum
```

On activation it takes one snapshot of `ankle_wrench/subject_mass_kg` for the
model and claims the same provider's tared torques. The pendulum parameter
`mass` defaults to zero (use the provider) and accepts a positive manual
override for testing. Leave the tare and mass unchanged while the pendulum is active.

Stop/deactivate the broadcaster and any other consumers before deactivating,
unloading or reconfiguring the provider. The manager also treats the general
`joint_state_broadcaster` (which selects all states) as a dependent consumer,
even if it was activated before these outputs existed. Include it in the stop:

```bash
ros2 control switch_controllers --strict --deactivate \
  ankle_wrench_state_broadcaster joint_state_broadcaster ankle_wrench
# Reactivate after any required reconfiguration:
ros2 control switch_controllers --strict --activate \
  ankle_wrench joint_state_broadcaster ankle_wrench_state_broadcaster
```

Include `ankle_wrench_plot_broadcaster` in both requests if it is running.

Settings are cached at configuration:
deactivate, edit parameters, then clean up/configure and reactivate the provider
before its consumers. Unloading and respawning with `--param-file controllers.yaml`
is another way to load edited settings. Active parameter edits are rejected.

```bash
ros2 param get /ankle_wrench same_scaling
ros2 param dump /ankle_wrench
ros2 topic echo /ankle_wrench_state_broadcaster/dynamic_joint_states
ros2 control list_controllers --verbose
ros2 control view_controller_chains
```
