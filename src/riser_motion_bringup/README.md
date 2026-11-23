# Bottom-axis CSV bringup

The single-motor launch is separate from the full-robot backends. Build
`riser_hardware` with `-DRISER_WITH_IGH=ON` and stock IgH userspace headers and
library. See the [riserms commissioning record](../../reports/riserms-ethercat-commissioning.md)
for host setup, the disabled launch, and ordered PREOP shutdown.

`bottom_motor.launch.py` defaults to `commissioning_disabled:=1`. It sends zero
controlword and target, rejects velocity claims, and omits the motion
controller. The optional joint state broadcaster can read motor feedback
without claiming a command interface.

Position and velocity represent motor-shaft radians and rad/s, respectively.
The scale defaults to 1,048,576 counts per revolution divided by 2π, from the
20-bit encoder and the archived 1:1 electronic gearing. No sign inversion,
homing, or linkage conversion is applied. Verify live drive identity and
scaling before operation.

An explicit `commissioning_disabled:=0` spawns the profile controller
inactive. A separate controller activation can request drive enable.
`ramp`, `triangle`, and `sine` are finite velocity profiles. A negative
`speed_rad_s` reverses the initial direction. A finished profile commands
zero and retains the command interface until deactivated. Timed velocity
profiles do not guarantee a measured position return. Powered motion
requires a separate supervised setup and validation of brake, travel,
mechanism direction, and stop response.
