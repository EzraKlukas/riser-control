# Gazebo angular and actuator references

`AngularSystem` keeps the reduced direct-pitch physics. Both command families
ultimately provide two pitch-angle targets and two independent physical slide
targets to the PD servos. Virtual actuator resources provide stroke references
and feedback without adding physical pitch actuator bodies.

## Simulation timing

`PhysicsClockControlPlugin` wraps the installed Gazebo control plugin and records
`UpdateInfo.simTime`, `dt` and pause state on the model before each physics step.
`AngularSystem` uses this synchronous context for hardware reads, effort writes
and reduced-dynamics calculations. Callback timestamps from the resource manager
can come from delayed `/clock` delivery and are not used to advance the physics.
Every advancing physics step receives its effort commands, including when the
controller reference remains constant. The context handoff adds no per-step
allocation, blocking I/O or mutex acquisition; it runs in Gazebo's update phases.
ROS publication and transport still have their existing scheduling costs.

`RiserSystem/read_time` is a read-only state in simulation seconds, describing the
hardware read cycle. The wrench controller stamps the combined wrench with
this state, selected by
`read_time_interface`; no Gazebo-specific dependency is added to that controller.
`/clock` remains available for ROS tools. Pause stops physics-time advancement.
For a full reset, restart the launch. Gazebo's in-place full-world reset
reconstructs the binary controller manager; controller service recovery was not
reliable during validation. Pause/resume keeps the existing manager and state.

## The three index spaces

`reference_mapping.hpp` names the index spaces so different array orders cannot
be mistaken for each other:

| Array | Order | Units |
| --- | --- | --- |
| Public state/reference arrays (`PublicJoint`) | Backboard, FootPlate, Ml, TopActuator, TorsoActuator, HipActuator, BottomActuator | First three rad; last four m |
| Mapping/physical reference arrays (`kin::AxisIndex`) | FramePitch, TorsoLateral, HipLateral, FootPitch | rad, m, m, rad |
| Gazebo entities/drive effort (`PhysicalJoint`) | BackboardJoint, FootPlateJoint, TorsoJoint, HipJoint | Torques N m, then forces N |

`PublicForAxis` and `PhysicalForAxis` explicitly connect these orders.
`read()` uses `ReferenceMapping::observe()` to report all seven coordinates.
Canonical stroke signs are preserved: both lateral actuator strokes are the
negative of physical slide displacement. Top and bottom strokes are also signed
according to the canonical CAD slider directions, not a guessed motor sign.
Actuator effort feedback is the commanded equivalent `F = tau_q/J`, not a
measured drive force. Reduced-body correction torques are not included in that
commanded actuator effort report.

## Reference steps 0, 0.5 and 1

The ROS-independent `ReferenceMapping::update()` contains the reference stages
and is used directly by the plugin's `write()` and unit tests:

1. **Step 0:** select the active command family. Check position commands against
   its native bounds; their reference velocities are zero, as in the original
   position servo. Clamp velocity commands and integrate them exactly once into
   native position references. Reaching an endpoint sets that reference rate
   to zero. Unused command arrays do not influence the robot.
2. **Step 0.5, actuator mode:** call `Axis::inverse(s, previous_q, sdot)` for each
   independent axis. This returns physical position `q=f^-1(s)` and velocity
   `qdot=sdot/J(q)`. For top/bottom, q is the desired pitch angle. For torso/hip,
   q is the signed physical slide position. Previous physical references seed
   the inverse to maintain branch continuity. Both reference positions AND
   reference velocities are converted. If an actuator velocity request would
   exceed the physical pitch rate, cap the pitch increment and velocity and
   map the accepted pair back to stroke.
3. **Step 1, angular mode:** use the checked angular pitch targets directly and
   `LateralCoupling::forward()` to expand ML into torso/hip slide targets. Use
   `Axis::forward(q,qdot)` to obtain matching actuator references and rates.
4. Check transformed pitch references against mechanical position and angular
   velocity bounds, and all actuator references against stroke bounds. The
   native pitch-actuator speed caps apply in actuator mode. Pitch stroke rates
   in angular mode are derived diagnostics. On a mapping error, `write()`
   zeros drive commands and commits no reference update.

An active simulated actuator switch stops a velocity command toward that
switch. The angular ML command stops as a pair if either slide reaches its
switch. Reverse motion remains available.

Actuator-mode torso/hip targets must **not** go through a common ML angle: that
would discard independent motion and prevent calibration. The fitted ML reference
is diagnostic in actuator mode and may exceed the normal ML command envelope
while each independent slide remains within its own travel. Both servos consume
physical slide targets directly. Angular mode remains ML-coupled.

Activation seeds targets from measured state and zeros velocity commands. If
activation precedes the first valid Gazebo state, seeding is deferred to the first
write with valid state. A backward simulation-clock jump also reseeds references.

## Simulated actuator limit switches

The Gazebo export boundary maps to `top_motor`, `torso_motor`,
`hip_motor` and `bottom_motor`. Carriage feedback is in metres and m/s.
`reverse_limit` and `forward_limit` are active-high, and `ready` and
`feedback_valid` expose the simulated operational state. The GPIO names are
ROS resources. The physical joints keep their model names.
`joint_state_broadcaster.use_urdf_to_filter: false` includes the GPIO feedback
in the joint state stream.

The shared calibration controller measures both switches and installs the
midpoint. Angular motion claims require calibrated, valid contributing
carriages. Angles expose `position`, `velocity`, `valid`, and the two
directional switches. Positive public footplate pitch raises the toes, and
positive backboard pitch leans forward. The physical joint stream uses raw
Gazebo pitch. The pitch switch directions follow the public angle signs.
The ML switches OR torso and hip forward together and reverse together.
The same mode-switch path reserves the mechanism for calibration, motor
profiles, angle profiles or the balance controller.

Switches use measured neutral-relative actuator stroke, 3 mm inside each existing
lower/upper hard stop, including overshoot and a 1e-10 m roundoff tolerance.
The optional hardware parameter `limit_switch_offset` sets this inset in metres
(default `0.003`; `0` restores exact-boundary switches). This allows position
servos with gravity/tracking error to reach the switches within their reference
bounds. The existing PD servos need a nonzero position error to hold residual
gravity torque; in a trial, top stroke settled about 0.20 mm and bottom stroke about 2.62 mm short of
their respective limit targets.
Set the inset with a `<param name="limit_switch_offset">...</param>` inside the
Gazebo hardware block in `riser_bringup/urdf/backends.xacro`.
Beyond a physical stop, measurement mapping extends the endpoint tangent (exact
for the linear slides, approximate for pitch actuators) so Gazebo contact overshoot
does not hide the switch behind a mapping error. Reference bounds remain strict.

The Gazebo wrapper calls hardware `read()` followed by controller `update()` in
PostUpdate; subsequent physics-step writes enforce the reported switches.
There is no assumed wall-clock reaction delay. Holding the previous reference or
moving away is allowed, including while a switch remains hit. Only advancing an
actuator reference farther into a reported switch (by more than 1e-10 m per
write) zeros all drive efforts and returns the existing hardware write error.
This applies to both command families through their mapped actuator references.
Velocity references already clamped at an endpoint do not advance and are allowed.
Out-of-bounds position commands still fail the existing reference checks.

A homing controller can therefore approach within the configured travel, read
the switch and hold or reverse in that update without faulting. Switches are
sampled at the controller read rate; these are simulated contacts, not a hardware
safety interlock. The added checks use fixed-size storage and no blocking IO;
the existing error path still logs and uses exceptions.

## Force-plate state interfaces

The Gazebo backend declares `left_force_plate` and `right_force_plate` as
ros2_control sensors. Each exports `force.x`, `force.y`, `force.z`, `torque.x`,
`torque.y`, `torque.z` and `valid`. For example, a controller can claim
`left_force_plate/force.z`. There are 44 state handles in total, including
`RiserSystem/read_time`.

Force and torque are in N and N m at each plate's top-surface centre. The state
interfaces express both vectors in physical axes `(y_gz, x_gz, -z_gz)`. The
wrench topics report in the Gazebo plate frames. When using the state interfaces,
express moment arms in the corresponding axes before shifting torque.

The adapter subscribes directly to each Gazebo sensor topic, so state feedback
does not depend on the ROS bridge. Transport callbacks copy into fixed storage;
`read()` attempts one nonblocking mutex acquisition per sensor and otherwise
retains the last snapshot. Exported storage is updated only by `read()`.
Sensors and controllers both default to 1000 Hz in simulation time. Sensor
delivery is asynchronous, so a controller can still reuse a previous sample.
There is no wall-clock delivery or synchronisation guarantee between the plates
or joint feedback.

The backend uses Gazebo's source timestamps internally to detect stale or
future-dated data. `valid` is 1 for a complete finite sample within the
`sample_timeout` in `backends.xacro` (default 0.1 simulated seconds).
Otherwise the six values are NaN and `valid` is 0. The ankle wrench combines
the valid values from a read cycle and stamps them with `RiserSystem/read_time`.

## Neutral-relative bounds and CAD dynamics

`riser_bringup/urdf/model/kinematics.yaml` supplies the operational neutral,
mechanical mapping domains and actuator stroke limits. All public positions
are relative to vertical backboard, flat footplate and centred carriage contacts.
The geometry factory remains CAD-relative. The adapter explicitly evaluates
`q_CAD=q+neutral_CAD`, `s=f(q_CAD)-f(neutral_CAD)`; inverse adds the neutral stroke
before mapping and subtracts the CAD neutral from the resulting q. Rates and
Jacobians are evaluated at that shifted CAD pose. ML uses centred slide positions.
These are fixed geometric offsets, not encoder calibration.

| Actuator | Lower stroke (m) | Upper stroke (m) | Velocity cap (m/s) |
| --- | ---: | ---: | ---: |
| Top | -0.335 | +0.335 | 0.67 |
| Torso | -0.335 | +0.335 | 0.67 |
| Hip | -0.335 | +0.335 | 0.67 |
| Bottom | -0.08 | +0.08 | 0.16 |

These estimated switch-to-switch ranges come from the user's approximate stroke
measurements and neutral assumptions. Canonical linkage inversion gives backboard
pitch approximately [-13.18,+12.40] degrees and footplate [-11.66,+12.65] degrees.
Coupled ML spans +/-13.95 degrees. Both command families use these geometric position bounds. The balance controller
adds its configured pendulum angle bounds.

The Gazebo physical pitch and slide caps are each axis's travel divided by
one simulated second. Backboard and footplate command caps are the same as
their physical caps, about 0.4464 and 0.4242 rad/s. The ML rate cap is about
0.4633 rad/s and keeps both expanded slide rates within 0.67 m/s across its
range. The pendulum reads the selected angle cap from the ros2_control description.
A positive `max_rate_rad_s` can reduce that cap.
The nonlinear pitch-to-stroke mapping can make a traverse in actuator mode
take longer as the physical angular cap becomes active. Actual tracking
depends on the servo, load, and approach to the switches. Use time-parameterized
trajectories for position commands.

The direct-pitch dynamics tables still cover CAD q = +/-0.08 rad. Lookup adds the
pitch neutral offset, interpolates inside the table and holds endpoint
coefficients outside it. Startup warns about insufficient table coverage. The
geometric stops do not move to the old sampling endpoints. This is approximate
full-travel dynamics, with no quantified error bound; the existing delayed
acceleration correction and servo gains are unchanged. See
[the motion-envelope report](../../reports/motion-envelope.md) for the derivation
and a Windows/CAD task prompt for generating wider tables.

## Startup modes and controllers

Use `ros2 launch riser_bringup control.launch.py backend:=gazebo` for the
[motion workflow](../riser_motion/README.md#bounded-motion). Calibration,
profile and balance controllers start inactive. The original raw command
controllers are available for development through the controller manager:

| `command_mode` | Spawned controller | Joint order |
| --- | --- | --- |
| `angular_position` (legacy `position`) | `riser_position_controller` | backboard_pitch, foot_plate_pitch, ml_angle |
| `angular_velocity` (legacy `velocity`) | `riser_velocity_controller` | same three angular joints |
| `actuator_position` | `riser_actuator_position_controller` | top_motor, torso_motor, hip_motor, bottom_motor |
| `actuator_velocity` | `riser_actuator_velocity_controller` | same four actuator joints |

Position controllers accept `JointTrajectory` commands; velocity controllers
accept `Float64MultiArray` commands. Carriage commands use m or m/s and angle
commands use rad or rad/s. `motion:=false` allows the raw carriage controllers
to start in Gazebo when their `command_mode` is selected. Angular claims require
calibration.

## Runtime switching

Use `ros2 control switch_controllers --strict` to stop the old motion
controller and start the new one in one request. A claim uses one command
family and can select a subset of coordinates for profiles or balance.
State consumers and broadcasters can remain active.

`prepare_command_mode_switch()` checks the incoming and outgoing claim lists
using the shared [RISER command policy](../riser_kinematics/include/riser_kinematics/command_mode.hpp).
`perform_command_mode_switch()` applies the same check in the update thread,
seeds position references from feedback, clears velocity commands, and sets
the new family. This step uses fixed storage without blocking I/O. With no
motion controller the Gazebo servo holds the measured physical pose captured
at the switch. Angular mode couples the two lateral slides through the ML
mapping. Let actuator homing settle before engaging angular mode so the slides
are near the coupled neutral pose.

## Work still required before hardware operation

See the [EtherCAT handoff](../riser_motion/README.md#laptop-gui-to-riserms-with-startup-inhibited)
for the command contracts already exposed by the G5 backend and the remaining
supervised checks. Gazebo's read and write paths use physics time, publish ROS
state, and use exceptions and logging on errors. The hardware backend keeps its
own cyclic guard and acquisition timestamps.

## Checks

The [kinematics library tests](../riser_kinematics/README.md#integration-and-verification)
check CAD geometry, mapping round trips and Python parity over the enlarged
mechanical domains. Bringup tests check three Gazebo angle resources and four
carriage GPIOs, startup modes, neutral poses and derived limits. The
[Gazebo motion test](../riser_bringup/test/gazebo_motion.py) exercises live
calibration, controller switches and bounded playback.
