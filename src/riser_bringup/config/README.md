# Runtime configuration

| File | Purpose |
| --- | --- |
| `control_coordinates.json` | Public joint order, lateral mapping, operating limits, servo gains and physical-state publication rate. |
| `controllers.yaml` | Controller types, command/state interfaces, joint lists and controller-manager update rate. |
| `force_plate_bridge.yaml` | Gazebo-to-ROS force-sensor topic and message mappings. |
| `riser.rviz` | RViz display settings for `view.launch.py`. |

Commands support position and velocity. State feedback exposes position, velocity
and effort. Servo gains and limits are provisional simulation values.

Model geometry, kinematics, rigid-body properties and reduced pitch dynamics live
in `../urdf/model`; plate properties are in `../urdf/riser.urdf.xacro`. Simulation
world `../worlds/riser_world.sdf` is used by launch. Set the force-sensor rate
with `force_plate_rate:=...`.

`reduced_pitch.json` stores full-precision samples in the order listed by
`columns`: angle, inertia, inertia derivative, gravity torque and damping.
Keep the uniform angle grid and columns consistent when changing the table.
Its inertial feedback uses acceleration delayed by one physics step. See
[CAD model derivation](../../../../reports/model.md).
