# Offline model references

These files support geometry inspection and actuator-mapping development. Bringup
does not load them; runtime parameters live in
[`riser_bringup/urdf/model`](../ros2_ws/src/riser_bringup/urdf/model) and
[`riser_bringup/config`](../ros2_ws/src/riser_bringup/config).

| File | Purpose |
| --- | --- |
| [model_reference/actuator_mapping.json](model_reference/actuator_mapping.json) | CAD pin positions, slider axes, rod lengths and assembly branches for the standalone Python/C++ mapping helpers. |
| [linkage_geometry.json](linkage_geometry.json) | ROS-frame linkage endpoints and rod reference angles for inspecting the CAD mechanism. |

Actuator travel is relative to the saved CAD pose. These references contain no
encoder, drive or homing calibration. The default simulation directly actuates
pitch joints and uses a reduced dynamics table for the omitted rods and stages.

