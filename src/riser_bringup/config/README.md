# Runtime configuration

| File | Purpose |
| --- | --- |
| `control_coordinates.json` | Public joint order, lateral mapping, operating limits, servo gains and physical-state publication rate. |
| `controllers.yaml` | Controller types, command/state interfaces, joint lists and controller-manager update rate. |
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
   **instead of** the normal position controller. The spawner loads, configures
   and activates the selected controller. Do not activate controllers with
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
and strict controller-switch commands. Additional Gazebo switch state interfaces
are exported by `riser_gazebo`; no bringup URDF changes were needed for them.

Model geometry, kinematics, rigid-body properties and reduced pitch dynamics live
in `../urdf/model`; plate properties are in `../urdf/riser.urdf.xacro`. Simulation
world `../worlds/riser_world.sdf` is used by launch. Set the force-sensor rate
with `force_plate_rate:=...`.

`reduced_pitch.json` stores full-precision samples in the order listed by
`columns`: angle, inertia, inertia derivative, gravity torque and damping.
Keep the uniform angle grid and columns consistent when changing the table.
Its inertial feedback uses acceleration delayed by one physics step. See
[CAD model derivation](../../../../reports/model.md).
