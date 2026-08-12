#!/usr/bin/env python3
"""Measure TRUE skidpad geometry error while the mission runs in the full sim.

The SIL harness scores its own physics; in the full sim the ground truth
lives on /odom. This script latches the car's true start pose at the moment
the state machine reaches DRIVING, transforms every subsequent true position
into that start frame, and scores it against the ideal skidpad geometry
(crossing point `entry` ahead, circles of `radius` at +/-radius laterally) --
the same metric sil_skidpad reports, so numbers compare directly.

Run it alongside the sim before GO; it exits by itself once the car has
driven and come to rest (or at --duration):

    python3 scripts/measure_skidpad.py --out skidpad_truth.json

Works regardless of what the controller drives on (ground truth or
/odometry/filtered) because it only reads TRUE state -- that's the point:
same instrument for the control run and the EKF run.
"""

import argparse
import json
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SkidpadMeter(Node):
    def __init__(self, args):
        super().__init__("skidpad_meter")
        self.entry = args.entry
        self.radius = args.radius
        self.duration = args.duration
        self.start = None       # (x0, y0, yaw0) latched at DRIVING
        self.driving_seen = False
        self.truth = None
        self.t = 0.0
        self.moved = False
        self.stopped_since = None
        self.max_v = 0.0
        self.dist = 0.0
        self.last_xy = None
        self.err_sum = 0.0
        self.err_max = 0.0
        self.err_n = 0
        self.final_local = (0.0, 0.0)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(String, "/sim/ros_can/state_str", self.on_state, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info("waiting for DRIVING...")

    def on_state(self, msg):
        if "ASState: DRIVING" in msg.data and not self.driving_seen:
            self.driving_seen = True
            if self.truth is not None:
                self.latch()

    def latch(self):
        x, y, yaw, _ = self.truth
        self.start = (x, y, yaw)
        self.get_logger().info("start latched at (%.1f, %.1f, %.1f deg)"
                               % (x, y, math.degrees(yaw)))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear.x
        self.truth = (p.x, p.y, yaw_of(msg.pose.pose.orientation), v)
        if self.start is None:
            if self.driving_seen:
                self.latch()
            return

        x0, y0, yaw0 = self.start
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        lx = c * (p.x - x0) - s * (p.y - y0)
        ly = s * (p.x - x0) + c * (p.y - y0)
        self.final_local = (lx, ly)
        self.max_v = max(self.max_v, v)

        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

        if v > 0.5:
            self.moved = True
        # score only within the circles' x-span: a drifted EXIT leg can sit
        # slightly off-axis (|ly|>1) and would otherwise be scored against
        # the circles it is deliberately driving away from
        in_span = (self.entry - self.radius - 1.0) < lx < (self.entry + self.radius + 1.0)
        if self.moved and abs(ly) > 1.0 and in_span:
            err = min(
                abs(math.hypot(lx - self.entry, ly + self.radius) - self.radius),
                abs(math.hypot(lx - self.entry, ly - self.radius) - self.radius),
            )
            self.err_sum += err
            self.err_max = max(self.err_max, err)
            self.err_n += 1

    def tick(self):
        self.t += 0.5
        v = self.truth[3] if self.truth else 0.0
        if self.moved and abs(v) < 0.02:
            if self.stopped_since is None:
                self.stopped_since = self.t
            elif self.t - self.stopped_since > 3.0:
                raise SystemExit(0)
        else:
            self.stopped_since = None
        if self.t >= self.duration:
            raise SystemExit(0)

    def report(self, out_path):
        mean = self.err_sum / self.err_n if self.err_n else -1.0
        result = {
            "circ_err_mean_m": round(mean, 3),
            "circ_err_max_m": round(self.err_max, 3),
            "max_v_ms": round(self.max_v, 2),
            "meters_traveled": round(self.dist, 1),
            "final_local_xy": [round(v, 2) for v in self.final_local],
            "samples": self.err_n,
        }
        print("RESULT " + " ".join("%s=%s" % kv for kv in result.items()),
              flush=True)
        with open(out_path, "w") as f:
            json.dump(result, f, indent=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--entry", type=float, default=9.8)
    ap.add_argument("--radius", type=float, default=9.25)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--out", default="skidpad_truth.json")
    args = ap.parse_args()

    rclpy.init()
    node = SkidpadMeter(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass
    node.report(args.out)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
