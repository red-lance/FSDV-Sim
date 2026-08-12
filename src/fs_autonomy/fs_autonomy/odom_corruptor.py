"""Odometry corruption shared by the SIL harnesses.

A harness owns ground truth, so it can SCORE against the true state while
PUBLISHING a corrupted estimate -- emulating a real state estimator's output.
Error model: white noise on pose/velocity plus a seeded random-walk drift
(expected drift std after t minutes = rate * sqrt(t_minutes)), the dominant
failure mode of dead-reckoned odometry.

Use a dedicated RNG stream (conventionally seed+1) so odom draws never
resample a harness's other error models.

    OdomCorruptor.declare(self)                      # in __init__, params
    self.odom = OdomCorruptor(self, rng, DT)         # after params resolve
    ex, ey, eyaw, ev = self.odom.corrupt(x, y, yaw, v)   # per physics step
"""

import math


class OdomCorruptor:
    PARAMS = (
        ("odom_pos_noise", 0.0),              # m, white
        ("odom_yaw_noise_deg", 0.0),          # deg, white
        ("odom_vel_noise", 0.0),              # m/s, white
        ("odom_drift_m_per_sqrt_min", 0.0),   # random-walk position drift
        ("odom_yaw_drift_deg_per_sqrt_min", 0.0),
    )

    @staticmethod
    def declare(node):
        for name, default in OdomCorruptor.PARAMS:
            node.declare_parameter(name, default)

    def __init__(self, node, rng, dt):
        gp = lambda k: node.get_parameter(k).value
        self.rng = rng
        self.pos_noise = gp("odom_pos_noise")
        self.yaw_noise = math.radians(gp("odom_yaw_noise_deg"))
        self.vel_noise = gp("odom_vel_noise")
        self.drift_q = gp("odom_drift_m_per_sqrt_min") * math.sqrt(dt / 60.0)
        self.yaw_drift_q = math.radians(
            gp("odom_yaw_drift_deg_per_sqrt_min")) * math.sqrt(dt / 60.0)
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0

    def corrupt(self, x, y, yaw, v):
        """Advance drift one step; return the estimated (x, y, yaw, v)."""
        g = self.rng.gauss
        self.drift_x += g(0.0, self.drift_q)
        self.drift_y += g(0.0, self.drift_q)
        self.drift_yaw += g(0.0, self.yaw_drift_q)
        return (x + self.drift_x + g(0.0, self.pos_noise),
                y + self.drift_y + g(0.0, self.pos_noise),
                yaw + self.drift_yaw + g(0.0, self.yaw_noise),
                v + g(0.0, self.vel_noise))
