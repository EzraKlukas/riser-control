# RISER control

ROS 2 simulation and control interfaces for the RISER balance robot. The model
has independently driven backboard and foot-platform pitch, coordinated torso
and hip lateral slides, and two force plates. Gazebo applies physical joint
torques and forces, with gravity, inertia and plate contact enabled.

![Functional RISER model](reports/functional-model.png)

## Build

Requires Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic binary packages.
Build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In each additional terminal, source ROS and `install/setup.bash`.

For the stock IgH G5 backend and laptop console, see the
[G5 commissioning checklist](reports/g5-motion-bringup.md).

## Bounded motion in Gazebo and on riserms

From the sourced workspace on your laptop:

```bash
ros2 launch riser_bringup control.launch.py backend:=gazebo \
  plot:=false plot_telemetry:=true wrench_input:=false
# In a second sourced terminal with the console's Qt environment active
ros2 run riser_motion g5_console --local-gazebo
```

Add `headless:=true` for server-only Gazebo. Startup loads the motion controllers
inactive. Start with the [motion and calibration procedure](src/riser_motion/README.md#bounded-motion).
It includes the GUI sequence, motion CSVs, trace names, an isolated Gazebo test,
and inhibited startup on riserms.

For a kinematic preview with sliders, use
`ros2 launch riser_bringup view.launch.py`. For controller experiments without
physics, use `control.launch.py backend:=mock plot:=false`.
The physical pitch and lateral slide states for TF are on
`/riser/physical_joint_states`.

## Footplate-only simulated participant

Start a fresh Gazebo session with the participant plugin available:

```bash
ros2 launch riser_bringup control.launch.py backend:=gazebo human:=true \
  plot_telemetry:=true plot:=false wrench_input:=false
ros2 control switch_controllers --strict --activate calibration_controller
```

Wait for all four midpoints and then deactivate calibration. Let the empty
platform settle near flat before taring it. The Gazebo balance configuration
uses the participant model's known mass of 70 kg.

```bash
ros2 control switch_controllers --strict --deactivate calibration_controller
ros2 service call /ankle_wrench/tare std_srvs/srv/SetBool '{data: true}'
```

With the local console, choose `foot_plate_pitch` / **position**, activate
the selected controller, and preview and start
[footplate_initial_pitch.csv](src/riser_motion/config/footplate_initial_pitch.csv).
It adds 0.035 rad to the settled starting position. Leave the position
controller active to hold the tilted plate while you spawn the participant.
Then switch to the footplate velocity balance controller.

```bash
ros2 run riser_human_balance spawn_participant --footplate-only
ros2 control switch_controllers --strict \
  --deactivate angle_position_profile --activate inverted_pendulum
gz topic -e -t /riser/human_balance
```

A vertical guide holds body orientation in the world while allowing vertical
translation and weight transfer through the feet. The feet articulate through
the ankle hinge and contact the plates. The goal is zero world foot pitch,
which is also neutral ankle pitch with an upright body. The footplate run leaves
the backboard at its held reference.

The participant stream reports simulated seconds, world body pitch, world foot
pitch, and applied ankle torque in that order. Compare those with actual
`foot_plate_pitch` from `/riser/physical_joint_states` and
`ankle_wrench/torque.x` from
`/ankle_wrench_plot_broadcaster/dynamic_joint_states`. Angles and torques
are in rad and N·m. To repeat the experiment from an empty platform, restart
the launch.

For output ankle-wrench perturbations, see the
[AP perturbation tutorial](src/riser_bringup/config/perturbation-tutorial.md).

## Force plates and model details

The left and right `geometry_msgs/msg/WrenchStamped` topics are
`/riser/force_plates/left/wrench` and `/riser/force_plates/right/wrench`.
Frames are `left_force_plate` and `right_force_plate`, with origins at the plate
top-surface centres. Values use N and N m; positive Fz is upward support on the
plate. Readings include plate weight and inertia. Sensor timestamps use simulation
time; plate motion requires compensation before interpreting readings as subject
forces.

Controllers can also claim `left_force_plate/force.x` through
`left_force_plate/torque.z`, and the corresponding `right_force_plate` state
interfaces. Each sensor exposes `valid`
(1 for a finite sample within its configured `sample_timeout`, otherwise
0). Unavailable force/torque values are NaN. The state interfaces report force
and torque at the surface centres in physical axes `(y_gz, x_gz, -z_gz)`.
The wrench topics report in the Gazebo plate frames. See the [Gazebo interface reference](src/riser_gazebo/README.md#force-plate-state-interfaces).

Gazebo bringup also runs `ankle_wrench`, exporting the combined wrench at the
ankle midpoint in physical axes attached to the foot platform. Inspect all eight
output states on `/ankle_wrench_state_broadcaster/dynamic_joint_states`, including
`valid` and `sample_time`.

For plotting at 25 Hz, select
`/ankle_wrench_plot_broadcaster/dynamic_joint_states` in PlotJuggler. This feed
also includes the pre-perturbation ankle torque and measured subject mass.
In Gazebo, `plot_telemetry:=true` enables the feed with `plot:=false`. The
real G5 launch enables it with `force_plates:=true`. Check the received rate with
`ros2 topic hz /ankle_wrench_plot_broadcaster/dynamic_joint_states`. In Gazebo
the received rate follows the simulation speed.

Matrices, gains and moment arms are in
`src/riser_bringup/config/controllers.yaml`; see the
[parameter and frame reference](src/riser_bringup/config/README.md#ankle-wrench).
Factory calibration is kept separately in that directory and is bypassed for
simulated SI surface wrenches. Weight and inertia remain in the combined reading.

The Gazebo control wrapper supplies exact physics time to the hardware adapter.
`RiserSystem/read_time` supplies the single read-cycle timestamp on the combined
ankle wrench. ROS clock-message delivery does not determine hardware integration
intervals.

Physics uses a 1 ms step, the controller is configured for 1000 Hz, and each
force sensor defaults to 1000 Hz (`force_plate_rate:=...`). These are simulation
settings; wall-clock throughput and ROS delivery depend on rendering, contacts
and scheduling. Reduced pitch inertia uses acceleration delayed by one physics
step. Servo gains and travel bounds are provisional simulation parameters.
See [CAD model derivation](reports/model.md) for the geometry and dynamics.

The inverted-pendulum controller implements the backboard law. The real hardware launch uses the motion-inhibited G5 CSV profile by default.

## Development

| Path | Contents |
| --- | --- |
| `src/riser_bringup/` | Launch files, controllers, URDF/Xacro, model parameters, worlds and state adapters. |
| `src/riser_gazebo/` | Physical actuation and reduced pitch dynamics. |
| `src/riser_hardware/` | IgH G5 CSV backend and actuator-mapping helpers. |
| `src/riser_subject_simulation/` | Subject wrench world plugin, ROS bridge input and optional GUI. |
| `src/riser_human_balance/` | Spawnable participant, sagittal ankle balance and pitch coupling. |
| `evidence/` | Small offline linkage and actuator-mapping references. |

[Configuration reference](src/riser_bringup/config/README.md) identifies
the runtime settings. [Offline model references](evidence/README.md) describe
the retained mapping data.

From the repository root, run the model/interface checks after building:

```bash
colcon test
colcon test-result --verbose
```

