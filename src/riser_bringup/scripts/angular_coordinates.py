"""Three angular coordinates to four relative actuator travels; ROS independent."""

import math


class AngularCoordinates:
    def __init__(self, top, bottom, torso_height, hip_height):
        if min(torso_height, hip_height) <= 0:
            raise ValueError("Positive setup heights required")
        self.maps = [top, bottom]
        self.h = [torso_height, hip_height]

    def forward(self, q):
        if len(q) != 3 or not all(math.isfinite(x) for x in q):
            raise ValueError("Three finite angles required")
        if abs(q[2]) >= math.pi / 2 - 1e-6:
            raise ValueError("ML tangent singularity")
        return [
            self.maps[0].forward(q[0]),
            self.maps[1].forward(q[1]),
            -self.h[0] * math.tan(q[2]),
            -self.h[1] * math.tan(q[2]),
        ]

    def jacobian(self, q):
        self.forward(q)
        return [
            [self.maps[0].derivative(q[0]), 0, 0],
            [0, self.maps[1].derivative(q[1]), 0],
            [0, 0, -self.h[0] / math.cos(q[2]) ** 2],
            [0, 0, -self.h[1] / math.cos(q[2]) ** 2],
        ]

    def velocity(self, q, dq):
        return [sum(a * b for a, b in zip(row, dq)) for row in self.jacobian(q)]

    def effort(self, q, forces):
        if len(forces) != 4:
            raise ValueError("Four actuator forces required")
        j = self.jacobian(q)
        return [sum(j[i][k] * forces[i] for i in range(4)) for k in range(3)]

    def ml_state(self, torso_travel, hip_travel):
        return math.atan(
            -sum(h * s for h, s in zip(self.h, [torso_travel, hip_travel]))
            / sum(h * h for h in self.h)
        )
