# RISER kinematics

A C++17 library with no ROS runtime dependency. It provides independent actuator
position/velocity mappings and a separate ML coordination mapping. It does not
perform homing, encoder conversion, trajectory generation or drive control.

## Geometry and provenance

The single authoritative geometry source remains
`evidence/model_reference/actuator_mapping.json` at the repository root. CMake
reads it with Python's standard `json` module and generates the compiled geometry
factory. The installed JSON under `share/riser_kinematics/config` is a build artifact,
not a second editable source. Build this package in the repository checkout (the
`evidence` directory is required). Changing the source JSON rebuilds the factory.
No JSON parser or ROS package lookup executes in the C++ evaluation path.

The geometry and equations were ported from
`src/riser_bringup/scripts/actuator_mapping.py`. That implementation remains an
independent test oracle. `test/parity.py` cross-checks the canonical geometry with
`evidence/linkage_geometry.json` and the URDF's pitch-axis/frame bindings:

* Raw geometry rotates about CAD X. The permutation to local ROS coordinates is
  `(x,y,z) = (CAD z,CAD x,CAD y)`, making that rotation ROS +Y. Linkage evidence is
  relative to the pitch pivot; the URDF world floor-height translation is not
  added to these local vectors.
* Frame pitch is `backboard_pitch` in the URDF. Its angle offset is
  -0.001003136473 rad; foot pitch is `foot_plate_pitch` and has zero offset.
* Assembly branches are +1 for frame pitch and -1 for foot pitch. CAD pin and
  slider coordinates, unit slider directions and rod lengths are used unchanged.
* Both lateral ratios remain -1: positive physical slide q means negative
  actuator s. This preserves the reference model's convention; it is not a
  measured motor/encoder sign calibration.
* Every displacement is relative to the saved CAD pose: `s(0) = 0`, including
  the frame's nonzero angle offset. This is not necessarily the midpoint of
  actuator stroke or a metrologically level robot pose.

Bringup now uses a different operational neutral (vertical/flat pitch and centred
carriages). Its Gazebo adapter converts `q_CAD=q_operational+neutral_CAD` and
`s_operational=f(q_CAD)-f(neutral_CAD)`. The canonical geometry and this library's
CAD-relative contract remain unchanged. See
[the motion-envelope report](../../reports/motion-envelope.md) for the supplied
travel measurements and geometry offsets.

The evidence and reference script do not supply homing offsets, encoder scale,
backlash or verified hardware limits. None are inferred by this package.

## Independent actuator mapping

`Coordinates` always has the canonical order:

| Index | Coordinate q | Units of q | Actuator displacement s |
| --- | --- | --- | --- |
| `FramePitch` | Independent backboard pitch | rad | m |
| `TorsoLateral` | Physical torso slide position | m | m |
| `HipLateral` | Physical hip slide position | m | m |
| `FootPitch` | Independent footplate pitch | rad | m |

This is **not** the three-coordinate public angular interface. In particular,
there is no ML coupling in `Mapping`. The hardware header's old abstract mapping
contract is replaced with an alias to this concrete implementation.

```cpp
#include <riser_kinematics/mapping.hpp>
using namespace riser_kinematics;
Mapping mapping;
// Example simulation domains, not verified hardware limits:
std::array<Domain, 4> domains{{{-.08,.08}, {-.05,.05}, {-.05,.05}, {-.08,.08}}};
auto error = mapping.configure(canonical_geometry(), domains); // before cycling
Coordinates q{}, qdot{}, s{}, sdot{}, recovered{}, recovered_velocity{};
if (error == Error::None)
  error = mapping.forward(q, qdot, s, sdot);
if (error == Error::None)
  error = mapping.inverse(s, sdot, q, recovered, recovered_velocity);
```

Use `Axis` directly when a caller also needs `Sample::jacobian`. For inverse
results the Jacobian remains **ds/dq**, not its reciprocal. Aggregated calls
return the first axis error and leave both output arrays unchanged on failure.
Per-axis calls return a `Sample` whose values must only be used on success.
Configure with caller-supplied domains; no operating limits are copied into the
geometry file. Failed configuration invalidates the object. Configuration must
not race with evaluation.

For a crank, let `b = body_pin - pivot` and `B(q) = pivot + Rx(q+offset)b`.
With `v = B-C0`, `p = u.v`, and `D = l^2-v.v+p^2`, evaluation uses

```
r = p + branch*sqrt(D)
s = r(q) - r(0)
J = p' + branch*(p*p' - v.B')/sqrt(D)
sdot = J*qdot
```

For inverse, define `a = pivot - C0 - (s+r(0))*u`. Expanding the rod closure
`|a+Rx(q+offset)b|^2 = l^2` gives

```
A = 2*(a_y*b_y + a_z*b_z)
B = 2*(-a_y*b_z + a_z*b_y)
C = l^2 - |a|^2 - |b|^2 - 2*a_x*b_x
A*cos(q+offset) + B*sin(q+offset) = C
q = atan2(B,A) +/- acos(C/hypot(A,B)) - offset + 2*pi*k
```

Each pitch domain is at most one revolution wide. Six bounded candidate checks
cover both roots and their neighboring periodic lifts. Domain filtering and a
forward residual check restore the assembly-branch constraint lost when squaring
the rod equation. Of the accepted roots, the one nearest the caller's seed is
selected (ties deterministically retain the first). There is no iterative root
search or hidden mutable seed. Velocity then follows `qdot = sdot/J`.

The slide mapping is `s = ratio*q`, with `J = ratio`.

Explicit errors distinguish unconfigured, invalid configuration, nonfinite input
or arithmetic overflow, out-of-domain, unreachable, singular and wrong-branch
cases. As in the Python reference, crank radicands at or below `1e-12 m^2` are
singular and Jacobians below `1e-8` in magnitude are rejected. Position domain
comparisons allow `1e-12` endpoint roundoff and clamp only that tolerance. Inverse
branch verification allows `1e-10 m` closure roundoff. Configuration validates
finite geometry, unit directions, branch, length, zero and domain endpoints;
every evaluation still checks reachability/singularity. A domain is not assumed
monotonic or nonsingular throughout merely because its endpoints are valid.

## ML coordination

`LateralCoupling` in `lateral.hpp` maps one ML angle/rate to the two physical
slide coordinates, separately from the actuator-stroke mapping above:

```
y_i = h_i*tan(theta)
ydot_i = h_i*(1+tan(theta)^2)*theta_dot
```

Configure positive torso/hip heights and an angular command domain strictly
inside `(-pi/2,pi/2)`. Heights remain supplied from existing backend configuration
(`control_coordinates.json` in bringup); this library embeds no second height
configuration. Gazebo's read/write paths use this component.

The inverse retains the existing least-squares estimate:

```
t = (h_t*y_t + h_h*y_h)/(h_t^2+h_h^2)
theta = atan(t)
theta_dot = (h_t*ydot_t + h_h*ydot_h)/((h_t^2+h_h^2)*(1+t^2))
```

It reports signed orthogonal residuals
`(h_h*y_t-h_t*y_h)/sqrt(h_t^2+h_h^2)` and the corresponding velocity residual.
A nonzero residual exposes incompatible slide states; the fit does not certify
calibration or synchronization. As a measurement conversion, inverse reports
observed angles even outside the command domain. Forward commands obey that
domain. Compose these physical slide coordinates with `Mapping` to obtain the
canonical, oppositely signed actuator strokes.

## Integration and verification

Both backends link `riser_kinematics::riser_kinematics` via ament. Consumers use:

```cmake
find_package(riser_kinematics REQUIRED)
target_link_libraries(my_target riser_kinematics::riser_kinematics)
```

The Gazebo pitch stages remain removed; the library supplies coordinate mappings
without installing command interfaces or a calibration sequence. The Gazebo
adapter applies operational neutral offsets to mapping and dynamics queries;
the EtherCAT backend still refuses configuration.
Encoder and session homing calibration remain in `DriveCalibration` downstream.

The cyclic methods are `const noexcept`, use fixed-size storage and have bounded
work with no allocation, IO or logging. This is not a wall-clock latency guarantee:
platform math-library timing and scheduling still require measurement on the
real control target.

ROS-free build and tests:

```sh
cmake -S src/riser_kinematics -B /tmp/riser-kinematics-build \
  -DRISER_KINEMATICS_AMENT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/riser-kinematics-build
ctest --test-dir /tmp/riser-kinematics-build --output-on-failure
```

Tests cover CAD zeros/endpoints, Python forward/Jacobian/inverse parity, rod/frame
provenance, finite-difference Jacobians, velocity round trips, branch/domain and
singularity rejection, seed-dependent inverse selection, ML fitting/residuals,
unchanged outputs on failure, and allocation counts during repeated evaluations.
The provenance test reads complete existing pitch objects individually from
`kinematics.yaml`; it does not validate unrelated in-progress actuator edits or
replace a full Xacro/Gazebo integration test.
