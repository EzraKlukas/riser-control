"""Relative SI actuator conversion for the reduced four-coordinate RISER model.

No ROS, encoder calibration, EtherCAT, drive enables, or robot commands.
The two crank mappings retain the selected branch and reject unreachable poses.
"""

import json
import math
from pathlib import Path


class AxisMap:
    def __init__(self, spec):
        self.spec = spec
        if spec["type"] == "slider_crank":
            u = spec["slider_axis_cad"]
            if abs(sum(x * x for x in u) - 1) > 1e-9:
                raise ValueError("Slider axis must be unit length")
            self.zero = self._raw(0)[0]

    def _raw(self, q):
        s = self.spec
        angle = q + s["angle_offset_rad"]
        p = s["pivot_cad_m"]
        b = [s["body_point_cad_m"][i] - p[i] for i in range(3)]
        sn, cs = math.sin(angle), math.cos(angle)
        B = [p[0] + b[0], p[1] + cs * b[1] - sn * b[2], p[2] + sn * b[1] + cs * b[2]]
        dB = [0, -sn * b[1] - cs * b[2], cs * b[1] - sn * b[2]]
        d = [B[i] - s["slider_reference_cad_m"][i] for i in range(3)]
        u = s["slider_axis_cad"]
        projection = sum(u[i] * d[i] for i in range(3))
        dp = sum(u[i] * dB[i] for i in range(3))
        radicand = s["rod_length_m"] ** 2 - sum(x * x for x in d) + projection**2
        if radicand <= 1e-12:
            raise ValueError("Unreachable pose or slider-crank branch singularity")
        root = math.sqrt(radicand)
        position = projection + s["branch"] * root
        derivative = (
            dp
            + s["branch"]
            * (projection * dp - sum(d[i] * dB[i] for i in range(3)))
            / root
        )
        C = [s["slider_reference_cad_m"][i] + position * u[i] for i in range(3)]
        return position, derivative, B, C

    def forward(self, q):
        if not math.isfinite(q):
            raise ValueError("Nonfinite coordinate")
        if self.spec["type"] == "linear":
            return self.spec["ratio"] * q
        return self._raw(q)[0] - self.zero

    def derivative(self, q):
        return self.spec["ratio"] if self.spec["type"] == "linear" else self._raw(q)[1]

    def closure_residual(self, q):
        if self.spec["type"] == "linear":
            return 0.0
        _, _, B, C = self._raw(q)
        return (
            math.sqrt(sum((B[i] - C[i]) ** 2 for i in range(3)))
            - self.spec["rod_length_m"]
        )

    def inverse(self, displacement, lower, upper):
        if not math.isfinite(displacement):
            raise ValueError("Nonfinite actuator displacement")
        slopes = [self.derivative(lower + (upper - lower) * i / 32) for i in range(33)]
        if min(abs(v) for v in slopes) < 1e-8 or min(slopes) * max(slopes) <= 0:
            raise ValueError(
                "Interval must be nonsingular and monotonic on the chosen branch"
            )
        lo_s, hi_s = self.forward(lower), self.forward(upper)
        if not min(lo_s, hi_s) - 1e-12 <= displacement <= max(lo_s, hi_s) + 1e-12:
            raise ValueError(
                "Actuator target outside the configured coordinate interval"
            )
        increasing = hi_s > lo_s
        for _ in range(70):
            mid = (lower + upper) / 2
            if (self.forward(mid) < displacement) == increasing:
                lower = mid
            else:
                upper = mid
        return (lower + upper) / 2


def load_maps(config):
    data = json.loads(Path(config).read_text())
    return {name: AxisMap(spec) for name, spec in data["axes"].items()}
