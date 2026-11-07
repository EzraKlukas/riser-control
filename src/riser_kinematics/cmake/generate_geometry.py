"""Build-time JSON loading; the cyclic C++ library has no parser dependency."""
import json
import math
from pathlib import Path
import sys

source, destination = map(Path, sys.argv[1:])
data = json.loads(source.read_text())
order = ["frame_pitch", "torso_lateral", "hip_lateral", "foot_pitch"]
if data["joint_order"] != order or set(data["axes"]) != set(order):
    raise ValueError("Unexpected independent-coordinate order")


def number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError("Geometry must contain finite numbers")
    return repr(float(value))


def vector(value):
    if len(value) != 3:
        raise ValueError("Expected a 3D vector")
    return "{" + ", ".join(map(number, value)) + "}"


lines = ['// Generated from actuator_mapping.json; do not edit.',
         '#include "riser_kinematics/mapping.hpp"',
         'namespace riser_kinematics {',
         'std::array<Geometry, 4> canonical_geometry() noexcept {',
         '  std::array<Geometry, 4> result{};']
for i, name in enumerate(order):
    spec = data["axes"][name]
    prefix = f"  result[{i}]."
    if spec["type"] == "linear":
        lines += [prefix + "linear = true;", prefix + f'ratio = {number(spec["ratio"])};']
    elif spec["type"] == "slider_crank":
        for target, key in [("offset", "angle_offset_rad"), ("rod_length", "rod_length_m")]:
            lines.append(prefix + f"{target} = {number(spec[key])};")
        if spec["branch"] not in (-1, 1):
            raise ValueError("Assembly branch must be +1 or -1")
        lines.append(prefix + f'branch = {int(spec["branch"])};')
        for target, key in [("pivot", "pivot_cad_m"), ("body_pin", "body_point_cad_m"),
                            ("slider_reference", "slider_reference_cad_m"),
                            ("slider_direction", "slider_axis_cad")]:
            lines.append(prefix + f"{target} = {vector(spec[key])};")
    else:
        raise ValueError("Unsupported geometry type")
lines += ['  return result;', '}', '} // namespace riser_kinematics', '']
destination.write_text("\n".join(lines))
