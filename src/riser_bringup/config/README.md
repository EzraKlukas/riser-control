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

Calibration provides a complete example. These are the bringup changes needed
after implementing a controller plugin:

1. In `controllers.yaml`, register its **instance name** under
   `controller_manager.ros__parameters`. `type` must match the class name exported
   by the controller package's plugin XML:

   ```yaml
   controller_manager:
     ros__parameters:
       calibration_controller:
         type: riser_calibration/CalibrationController
   ```

2. Add the controller's parameters under a top-level block with the **same
   instance name**. Only parameters declared/read by that controller have an
   effect; listing interfaces in YAML does not make a custom controller claim
   them. Its `command_interface_configuration()` and
   `state_interface_configuration()` methods define the claims.

   ```yaml
   calibration_controller:
     ros__parameters:
       lower_limits: [-0.335, -0.335, -0.335, -0.08]
       upper_limits: [0.335, 0.335, 0.335, 0.24]
       speeds: [0.01, 0.01, 0.01, 0.005]
   ```

3. In `riser_bringup/package.xml`, add the plugin package as a runtime dependency:
   `<exec_depend>riser_calibration</exec_depend>`. This makes the package
   dependency explicit; the controller is loaded dynamically, not linked into
   bringup.
4. In `launch/control.launch.py`, select its instance name for the existing
   controller-manager spawner. Calibration adds `calibrate:=true`, requires
   Gazebo's `actuator_position` mode, and selects `calibration_controller`
   instead of the normal position controller. Pass `controllers.yaml` to each
   spawner with `--param-file` to load its parameters. The spawner loads,
   configures and activates the selected controller. Do not activate controllers with
   competing command claims. This Gazebo adapter also consumes only its
   startup-selected command family, regardless of controller claims.

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
states. The provider claims both plates' six wrench components, `sample_time`
and `valid`, and exports the same eight names under `ankle_wrench/`.
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

Thus a shared matrix with different gains uses `same_transformation=true` and
`same_scaling=false`. Matrix entries are arbitrary experimental operations on
physical components, **not a change of coordinate frame**; geometric moment arms
remain in the physical axes. Defaults are identity matrices and unit gains.
There is no plate-weight, gravity or inertial compensation.

Both plate timestamps remain available at the inputs because transport delivery
can differ even at equal sampling rates. The result carries the older timestamp,
with at most `max_sample_skew_s` separation (default 0.01 s). Samples are used
without interpolation. `max_sample_age_s` defaults to 0.1 s.
Unavailable, stale, future-dated, excessively skewed or nonfinite data makes the
output NaN with `valid=0`; fresh input recovers automatically. Activation
requires bound input interfaces and valid configuration. The read-only provider
starts with invalid output until good samples arrive. `read_time_interface`
selects `RiserSystem/read_time`, so sample-quality and age checks use the exact
hardware read-cycle time and elapsed time between successive reads. An empty
parameter uses the supplied controller time and period;
use that only when those timestamps accurately represent the hardware cycle. Consumers must
check validity and age before using the wrench.
Updates use fixed storage without allocation, blocking I/O or logging; this does
not guarantee scheduling latency or hard realtime execution.

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
