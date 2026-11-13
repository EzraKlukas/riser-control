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
hardware read cycle. Controllers can use it to check sample age in the same
clock as the sensor timestamps. The wrench controller selects this state with
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

## Simulated actuator limit switches

Each of `top_actuator`, `torso_actuator`, `hip_actuator` and `bottom_actuator`
exports `limit_switch_lower` and `limit_switch_upper` state interfaces, with
`1.0` meaning hit and `0.0` meaning clear. Controllers can claim, for example,
`torso_actuator/limit_switch_lower`. No switches are added to the angular
resources. The four virtual actuators are declared as ros2_control GPIOs;
limits are read from `HardwareInfo::gpios`. The adapter exports all fourteen
command handles and twenty-nine motion/switch state handles, including eight
switch handles. Force-plate sensor states are exported separately below.
`joint_state_broadcaster.use_urdf_to_filter: false` retains actuator feedback on
`/joint_states` despite the absence of physical actuator joints.

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
`torque.y`, `torque.z`, `sample_time` and `valid`; for example, a controller can
claim `left_force_plate/force.z`. There are 46 state handles in total: the existing 45 feedback handles plus
`RiserSystem/read_time`. There are no additional command handles. Mock and EtherCAT resources are unchanged.

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

`sample_time` is the original sensor timestamp in simulation seconds (NaN when
absent). `valid` is 1 only for a complete finite sample dated no later than the
current physics read-cycle time and no older than `sample_timeout`. Before the first
sample, on malformed data, after a clock reset leaving a future-dated sample,
or after timeout, `valid` is 0 and all six wrench values are NaN. Missing sensor
data does not disable motion control; consuming controllers must check validity,
age and left/right timestamp skew before using the wrench for control. Set
`sample_timeout` in each sensor declaration in `backends.xacro` (default 0.1 s
of simulation time). A paused simulation does not advance sample age.

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
mechanical domains. Bringup tests check three Gazebo joints plus four GPIOs/fourteen command
handles, three joints for other backends, startup modes, neutral poses and
derived limits. These checks do not substitute for a full ROS/Gazebo build and
live trajectory trials.
