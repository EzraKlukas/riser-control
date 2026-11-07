# Neutral coordinates and motion ranges

## Neutral convention

Operational zero means:

* The lateral carriage reference points are directly over the force-plate pair's
  midpoint in ML.
* The backboard is vertical (zero pitch).
* The footplate is flat (zero pitch).

The robot reference frame is `base_link`: ML origin at the pair midpoint, AP
origin at the common pitch-axis location, vertical origin at that axis. Both
pitch joint origins are already `(0,0,0)` in base. World-to-base translation
`(0,0,0.6477)` m keeps the world floor-referenced.

Keep geometric neutral offsets separate from per-session encoder offsets. This
study does not specify encoder calibration; the calibration controller will
establish those offsets each physical session.

The audited geometry factory still uses saved CAD coordinates. Convert explicitly:

```
q_CAD = q_neutral_CAD + q_operational
s_operational = f(q_CAD) - f(q_neutral_CAD)
sdot = J(q_CAD) * qdot
```

This preserves pin geometry, assembly branch and signs. The backboard's saved
orientation offset is -0.001003136473 rad, so its vertical neutral has
`q_neutral_CAD = +0.001003136473 rad`. Flat footplate neutral has CAD q = 0.

For the simulation, use the modelled carriage contact-box centre as the lateral
neutral reference point. Its y position at CAD q = 0 is
0.274828 - 0.0060000019 = +0.2688279981 m, so both lateral coordinates have
q_neutral_CAD = -0.2688279981 m. The neutral point must be used consistently in
the model and mappings; changing the world frame alone does not move carriages
relative to force plates.

## Supplied travel measurements

The user reports these approximate limit-switch-to-limit-switch strokes:

| Actuator | Total stroke | Operational stroke endpoints |
| --- | ---: | ---: |
| Top/backboard | about 670 mm | -335 to +335 mm |
| Torso lateral | about 670 mm | -335 to +335 mm |
| Hip lateral | about 670 mm | -335 to +335 mm |
| Bottom/footplate | about 320 mm | -80 to +240 mm |

Neutral is assumed halfway through the top, torso and hip travel, as instructed.
These are measurement-based estimates of travel, not precision metrology or
certified collision/force limits. Encoder scale/reference is not needed to use
these stroke measurements in the geometric simulation.

## Derived backboard pitch limits

Source geometry: `evidence/model_reference/actuator_mapping.json`, evaluated with
`src/riser_bringup/scripts/actuator_mapping.py` on the configured assembly branch.
At vertical neutral, the top CAD-relative stroke is -0.0015358546904 m.
The assumed top stroke endpoints therefore correspond to:

| Operational top stroke | CAD-relative top stroke | Operational pitch from vertical |
| --- | ---: | ---: |
| -335 mm | -336.5358547 mm | approximately +12.4 degrees |
| +335 mm | +333.4641453 mm | approximately -13.2 degrees |

The unequal pitch magnitudes follow from the nonlinear linkage despite symmetric
actuator travel. Numerically, the calculated angles are +12.3958808 and
-13.1820342 degrees; that numerical precision does not increase the accuracy of
the approximate 670 mm measurement. Forward rod-closure residuals are below
1e-12 m at the calculated endpoints.

The reported +6 degree software operating limit lies inside this estimated
mechanical interval. It is a separate control policy and is not implemented by
this geometric configuration.

## Lateral and coupled ML limits

With centred neutral and the canonical lateral ratio -1, each independent slide
has estimated travel +/-335 mm. A positive physical slide displacement gives a
negative operational actuator stroke.

Using the existing fixed heights h_torso = 1.348575 m and h_hip = 0.6215 m:

```
y_torso = h_torso*tan(ML)
y_hip = h_hip*tan(ML)
ML_limit = min(atan(0.335/h_torso), atan(0.335/h_hip))
```

Torso travel limits the coupled motion to approximately **+/-13.95 degrees ML**.
Independent hip travel remains +/-335 mm; coupled ML does not use its full
travel. These calculations assume slide displacement relative to the newly
centred neutral, not the unshifted CAD position.

The force plates' outside footprint spans y = +/-251.968 mm in the current
URDF (centres +/-127 mm; each plate 249.936 mm wide). The supplied actuator
travel extends beyond that footprint. Hardware travel, a chosen footprint-based
operating envelope, and software task limits must remain separate. An inner
"almost to the edge" operating bound needs a chosen clearance; it does not
replace the measured actuator hard-range estimate.

## Footplate endpoints

Flat neutral is approximately 80 mm from the limit facing the backboard and
240 mm from the opposite limit. In the stored geometry, the backboard is in the
negative ROS X direction from the bottom slider; the canonical bottom slider
axis is positive ROS X. Thus the rearward limit is negative stroke:

| Endpoint | Stroke relative to flat | Footplate pitch |
| --- | ---: | ---: |
| Towards backboard | -80 mm | approximately +12.65 degrees |
| Away from backboard | +240 mm | approximately -27.42 degrees |

The inverse gives q_lower = -0.478583600678 rad and
q_upper = +0.220708877578 rad, with the original bottom assembly branch -1.
Both endpoint rod-closure residuals are below 1e-12 m. Approximate physical
measurements do not justify treating the displayed numerical precision as a
calibrated hardware limit. No assumption of a centred footplate actuator is used.

## Complete estimated mechanical intervals

All positions below use the agreed operational neutral; stroke is measured
from neutral, not from an encoder count or the old saved-CAD stroke zero.

| Coordinate | Lower | Upper |
| --- | ---: | ---: |
| Backboard pitch | -13.18 degrees | +12.40 degrees |
| Footplate pitch | -27.42 degrees | +12.65 degrees |
| Torso lateral displacement | -335 mm | +335 mm |
| Hip lateral displacement | -335 mm | +335 mm |
| Coupled ML angle | -13.95 degrees | +13.95 degrees |
| Top actuator stroke | -335 mm | +335 mm |
| Torso actuator stroke | -335 mm | +335 mm |
| Hip actuator stroke | -335 mm | +335 mm |
| Bottom actuator stroke | -80 mm | +240 mm |

These are geometry-consistent estimates from the supplied switch-to-switch
measurements and neutral assumptions. Session encoder calibration remains
outside this study. The supplied
measurements are now sufficient for all four geometric actuator intervals.

## Applied configuration and dynamics approximation

`kinematics.yaml` now applies these operational zeros and estimated mechanical
intervals. Pitch joint origins are vertical/flat at zero; both carriage contact
centres are over ML zero at zero slide displacement. The canonical mapping
geometry remains unchanged. `ReferenceMapping` adds the configured CAD neutral
before forward evaluation, subtracts the neutral stroke, and reverses these
operations for inverse evaluation. ML uses the centred physical slides directly.
No encoder offsets or homing sequence are introduced.

Both angular and actuator commands use these geometric position bounds. Mapping
domains are derived directly from the configured pitch and actuator limits;
there is no separate software-limit layer. Mode/claim switching and calibrated
hardware protection remain separate work.

ML limits and the dummy ML joint now use the same configuration. The ML rate cap
is approximately 0.01383 rad/s and the footplate rate cap is 0.01 rad/s so the
sampled mapped stroke speeds fit the existing actuator caps across the enlarged
domains. Backboard remains 0.012 rad/s. These rates and servo gains are provisional;
position steps and actual tracking speed are not constrained by a reference-rate
cap. Transformed velocity checks still reject incompatible native actuator rates.

The reduced pitch tables cover only **CAD q in [-0.08,+0.08] rad**, or approximately
+/-4.58 CAD degrees. This is a sampling boundary, not a mechanism limit, branch
change, or change in the form of the equations. The supplied measured travel
remains on the original nonsingular assembly branches; the geometry tests now
cover the wider intervals.

Dynamics lookup adds the pitch neutral offset before querying the table. Inside
the table it interpolates; outside it holds each coefficient at its nearest
endpoint. The plugin warns once per affected axis at initialization. Geometric
command bounds and physical joint stops remain independent of this lookup clamp.
The tables themselves have not been extended or relabelled.

This approximation is **model error**, not a known small error per timestep.
Inertia, gravity, damping and the inertia-derivative term can become increasingly
inaccurate away from the sampled interval, affecting trajectories and contact
response. Endpoint-held coefficients do not constitute an energy-consistent
extension of the reduced model. Simulated position differences can grow with time,
but no particular accumulated error or accuracy bound has been established.
The existing one-step acceleration-feedback approximation also remains. Wider
motion is now available for control development; accurate full-travel dynamics
require resampling from the source CAD.

## Task prompt for the Windows CAD repository

Use the following prompt in the repository containing the SolidWorks assembly.
Provide it with this workspace's `evidence`, `urdf/model` files and the current
`riser_system.cpp` if those are not already available there.

> Regenerate RISER's reduced direct-pitch dynamics across the measured mechanical
> travel. Inspect repository instructions and the available SolidWorks extraction
> tools first. Keep Gazebo's two direct pitch joints and the existing JSON schema;
> do not add actuator bodies, change linkage branches, or fit/extrapolate the old
> table to invent data beyond its sampled range.
>
> Use `evidence/model_reference/actuator_mapping.json` as the authoritative pin,
> slider, rod, branch and CAD-coordinate definition; cross-check
> `evidence/linkage_geometry.json`, the assembly and URDF frames. Preserve CAD
> offsets and signs. The operational neutral CAD coordinates are top
> +0.001003136473 rad and bottom 0. Sample uniformly over at least top CAD
> [-0.22906676282907767, +0.217352069299856] rad and bottom CAD
> [-0.4785836006777029, +0.22070887757756963] rad. Use a grid no coarser than
> the existing 0.00016 rad spacing, preserving the existing CAD-q table columns
> `[q, M, dM/dq, Qg, D]` and units.
>
> Audit exactly which moving stage and connecting-rod part instances were removed
> from Gazebo. Extract their disjoint masses, COMs and full inertia tensors with
> units, frame transformations and provenance. Do not double-count bodies already
> represented by URDF inertias. Confirm material/density assumptions and CAD mates
> instead of inferring them from the existing numbers. Keep top and bottom pitch
> independent. Identify any omitted coupling that prevents the current scalar
> reduction from being valid and report it before changing the physics model.
>
> For each removed rigid body, determine COM position x(q), orientation R(q),
> and angular velocity per unit qdot, w_q. Compute
> `M(q) = sum(m*|dx/dq|^2 + w_q^T*I_world(q)*w_q)` and
> `Qg(q) = sum(m*g_vector dot dx/dq)` with downward gravity 9.80665 m/s^2.
> Differentiate M consistently to obtain dM/dq. Preserve the existing damping
> law only if its parameters and coordinate reduction can be established;
> document missing damping information rather than guessing it. Generalized
> viscous damping reduces through squared velocity Jacobians.
>
> Supply a reproducible offline generator, extracted inputs and provenance,
> and replacement `reduced_pitch.json`. Compare all four coefficients against
> the old table throughout its overlap; explain discrepancies rather than
> silently overwriting them. Validate rod closure and original assembly branches,
> positive finite inertia, nonnegative damping, finite-difference dM/dq, gravity
> against the negative potential-energy derivative, and kinetic-energy equivalence.
> Check endpoint, flat and vertical poses independently of grid samples. Include
> convergence checks for differentiation and interpolation. Preserve the existing
> runtime endpoint clamp as a fallback and the one-step acceleration feedback;
> document that this offline calculation does not validate integration stability
> or controller gains. Update existing model documentation with generation steps
> and accuracy limits. Do not command physical hardware.
