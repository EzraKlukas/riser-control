# Gazebo angular and actuator references

`AngularSystem` keeps the reduced direct-pitch physics. Both command families
ultimately provide two pitch-angle targets and two independent physical slide
targets to the existing PD servos. This change adds reference conversion, not
simulated pitch actuator bodies or encoder calibration.

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
   reference velocities are converted, even in velocity mode; no second angular
   integration is performed.
3. **Step 1, angular mode:** use the checked angular pitch targets directly and
   `LateralCoupling::forward()` to expand ML into torso/hip slide targets. Use
   `Axis::forward(q,qdot)` to obtain matching actuator references and rates.
4. Check transformed pitch references against mechanical position and angular
   velocity bounds, and all actuator references against stroke and rate bounds. On failure, commit no partial reference update;
   `write()` zeros drive commands and returns an error. It does not independently
   clamp a converted velocity, which would break `sdot=J*qdot`.

Actuator-mode torso/hip targets must **not** go through a common ML angle: that
would discard independent motion and prevent calibration. The fitted ML reference
is diagnostic in actuator mode and may exceed the normal ML command envelope
while each independent slide remains within its own travel. Both servos consume
physical slide targets directly. Angular mode remains ML-coupled.

Activation seeds targets from measured state and zeros velocity commands. If
activation precedes the first valid Gazebo state, seeding is deferred to the first
write with valid state. A backward simulation-clock jump also reseeds references.

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
| Top | -0.335 | +0.335 | 0.02 |
| Torso | -0.335 | +0.335 | 0.02 |
| Hip | -0.335 | +0.335 | 0.02 |
| Bottom | -0.08 | +0.24 | 0.01 |

These estimated switch-to-switch ranges come from the user's approximate stroke
measurements and neutral assumptions. Canonical linkage inversion gives backboard
pitch approximately [-13.18,+12.40] degrees and footplate [-27.42,+12.65] degrees.
Coupled ML spans +/-13.95 degrees. Both command families use these geometric
position bounds; no additional angular software-limit layer is implemented.

Backboard/footplate/ML rate caps are respectively 0.012, 0.01 and approximately
0.01383 rad/s. Transformed checks still reject actuator commands whose inverse
rates exceed those angular caps. Reference limits do not guarantee actual
tracking speed; use time-parameterized trajectories instead of position steps.
Effort config values do not install an actuator-force limiter.

The direct-pitch dynamics tables still cover CAD q = +/-0.08 rad. Lookup adds the
pitch neutral offset, interpolates inside the table and holds endpoint
coefficients outside it. Startup warns about insufficient table coverage. The
geometric stops do not move to the old sampling endpoints. This is approximate
full-travel dynamics, with no quantified error bound; the existing delayed
acceleration correction and servo gains are unchanged. See
[the motion-envelope report](../../reports/motion-envelope.md) for the derivation
and a Windows/CAD task prompt for generating wider tables.

## Startup modes and controllers

Use `ros2 launch riser_bringup control.launch.py backend:=gazebo` with one of:

| `command_mode` | Spawned controller | Joint order |
| --- | --- | --- |
| `angular_position` (legacy `position`) | `riser_position_controller` | backboard_pitch, foot_plate_pitch, ml_angle |
| `angular_velocity` (legacy `velocity`) | `riser_velocity_controller` | same three angular joints |
| `actuator_position` | `riser_actuator_position_controller` | top_actuator, torso_actuator, hip_actuator, bottom_actuator |
| `actuator_velocity` | `riser_actuator_velocity_controller` | same four actuator joints |

Position controllers accept `JointTrajectory` commands; velocity controllers
accept `Float64MultiArray` commands. Actuator commands use m or m/s; angles use
rad or rad/s. Mock and the unimplemented EtherCAT boundary retain three joints;
actuator startup modes are currently supported only by Gazebo.

## Work still required before hardware operation

Runtime switching is intentionally not implemented. All fourteen command handles
exist, but only the startup-selected family is consumed. Do not manually activate
another controller and assume its claims select a new hardware mode. Controller
manager does not recognize overlap between ML and actuator resource names.

Implement `prepare_command_mode_switch()` to validate the full proposed claim
set `(current - stops) + starts`, reject overlapping angular/actuator claims and
mixed position/velocity commands, and gate angular operation on valid homing and
slide alignment. In `perform_command_mode_switch()`, commit the prepared mode and
seed references from measured state with zero rates. This update-loop operation
must be bounded, allocation-free and nonblocking. Stop/start the appropriate
controllers in one strict switch request. Broadcasters may share state handles.

EtherCAT remains unimplemented. Homing/encoder offsets, switch inputs and local
limit enforcement, communications-loss/watchdog behavior, verified speed and
force limits, and measured loop latency are still required for hardware. The
existing Gazebo loop also publishes ROS messages, creates component vectors and
uses exceptions/logging on errors; it is not a hard realtime hardware template.

## Checks

The [kinematics library tests](../riser_kinematics/README.md#integration-and-verification)
check CAD geometry, mapping round trips and Python parity over the enlarged
mechanical domains. Bringup tests check seven Gazebo resources/fourteen command
handles, three resources for other backends, startup modes, neutral poses and
derived limits. These checks do not substitute for a full ROS/Gazebo build and
live trajectory trials.
