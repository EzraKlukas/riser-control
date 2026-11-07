# RISER control

ROS 2 simulation and control interfaces for the RISER balance robot. The model
has independently driven backboard and foot-platform pitch, coordinated torso
and hip lateral slides, and two force plates. Gazebo applies physical joint
torques and forces, with gravity, inertia and plate contact enabled.

![Functional RISER model](reports/functional-model.png)

## Build

Target environment: Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress, with a
compatible `gz_ros2_control` installation. From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In each additional terminal, source ROS and `ros2_ws/install/setup.bash`.

## Explore

Start Gazebo with the position controller:

```bash
ros2 launch riser_bringup control.launch.py backend:=gazebo plot:=false
# Without the Gazebo GUI:
ros2 launch riser_bringup control.launch.py backend:=gazebo headless:=true plot:=false
```

Run one launch at a time; stop it with Ctrl+C before starting another. `plot:=false`
disables the optional PlotJuggler window, which launch otherwise opens by default.
For a kinematic preview with sliders, use
`ros2 launch riser_bringup view.launch.py`. For controller experiments without
physics, use `control.launch.py backend:=mock plot:=false`.

In a second sourced terminal, inspect the active controllers and move all three
coordinates slightly over five simulated seconds:

```bash
ros2 control list_controllers
ros2 topic pub --once /riser_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [backboard_pitch, foot_plate_pitch, ml_angle], points: [{positions: [0.02, -0.02, 0.01], time_from_start: {sec: 5}}]}'
ros2 topic echo --once /joint_states
ros2 topic echo --once /riser/force_plates/left/wrench
```

Commands use radians in the order `backboard_pitch`, `foot_plate_pitch`,
`ml_angle`; zero is the saved CAD pose. Position is the default command mode.
`command_mode:=velocity` selects the velocity controller, accepting a
`std_msgs/msg/Float64MultiArray` on `/riser_velocity_controller/commands` in the
same order, in rad/s. Send zeros to stop velocity commands. State feedback exposes
position, velocity and effort; `/riser/physical_joint_states` gives the actual
pitch and lateral-slide states used for TF.

## Force plates and model details

The left and right `geometry_msgs/msg/WrenchStamped` topics are
`/riser/force_plates/left/wrench` and `/riser/force_plates/right/wrench`.
Frames are `left_force_plate` and `right_force_plate`, with origins at the plate
top-surface centres. Values use N and N m; positive Fz is upward support on the
plate. Readings include plate weight and inertia. Sensor timestamps use simulation
time; plate motion requires compensation before interpreting readings as subject
forces.

Physics uses a 1 ms step, the controller is configured for 1000 Hz, and each
force sensor defaults to 500 Hz (`force_plate_rate:=...`). These are simulation
settings; wall-clock throughput and ROS delivery depend on rendering, contacts
and scheduling. Reduced pitch inertia uses acceleration delayed by one physics
step. Servo gains and travel bounds are provisional simulation parameters.
See [CAD model derivation](reports/model.md) for the geometry and dynamics.

Subject dynamics and a balance controller are not yet implemented. The EtherCAT
backend is an unimplemented hardware boundary and refuses configuration.

## Development

| Path | Contents |
| --- | --- |
| `ros2_ws/src/riser_bringup/` | Launch files, controllers, URDF/Xacro, model parameters, worlds and state adapters. |
| `ros2_ws/src/riser_gazebo/` | Physical actuation and reduced pitch dynamics. |
| `ros2_ws/src/riser_hardware/` | Hardware boundary and standalone actuator-mapping helpers. |
| `ros2_ws/src/riser_subject_simulation/` | Draft subject-load script; not yet a ROS package. |
| `evidence/` | Small offline linkage and actuator-mapping references. |

[Configuration reference](ros2_ws/src/riser_bringup/config/README.md) identifies
the runtime settings. [Offline model references](evidence/README.md) describe
the retained mapping data.

From `ros2_ws`, run the model/interface checks after building:

```bash
colcon test --packages-select riser_bringup
colcon test-result --verbose
```

