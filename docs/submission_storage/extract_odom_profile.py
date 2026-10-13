#!/usr/bin/env python3
"""Measure a state estimator's error against sim ground truth.

The state-estimation twin of extract_error_profile.py: run the sim, the
estimation stack, and (ideally) a mission so the car actually moves, then
this script for the duration of the run. It samples ground truth and the
estimate, and reports the error statistics in EXACTLY the units the SIL
harness odom knobs consume (odom_corruptor.py).

    # terminals: sim / autonomy.launch / estimation.launch, mission running
    python3 scripts/extract_odom_profile.py --duration 120 --out odom_profile.json

Ground truth defaults to /odom (the sim's true state); the estimate to
/odometry/filtered (robot_localization output).
"""

import argparse
import json
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomProfiler(Node):
    def __init__(self, truth_topic, est_topic, duration):
        super().__init__("odom_profiler")
        self.duration = duration
        self.truth = None          # latest (x, y, yaw, v)
        self.samples = []          # (t, e_pos, e_yaw, e_v, dist_traveled)
        self.dist = 0.0
        self.last_xy = None
        self.t0 = None
        self.create_subscription(Odometry, truth_topic, self.on_truth, 50)
        self.create_subscription(Odometry, est_topic, self.on_est, 50)
        self.create_timer(1.0, self.check_done)
        self.get_logger().info("profiling %s vs %s for %.0f s"
                               % (est_topic, truth_topic, duration))

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = (p.x, p.y, yaw_of(msg.pose.pose.orientation),
                      msg.twist.twist.linear.x)
        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_est(self, msg):
        if self.truth is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose.position
        tx, ty, tyaw, tv = self.truth
        e_yaw = math.atan2(math.sin(yaw_of(msg.pose.pose.orientation) - tyaw),
                           math.cos(yaw_of(msg.pose.pose.orientation) - tyaw))
        self.samples.append((t - self.t0,
                             math.hypot(p.x - tx, p.y - ty),
                             e_yaw,
                             msg.twist.twist.linear.x - tv,
                             self.dist))

    def check_done(self):
        if self.t0 is not None and self.samples and self.samples[-1][0] >= self.duration:
            raise SystemExit(0)

    def report(self, out_path):
        if len(self.samples) < 50:
            print("too few samples (%d) -- is the estimator running?"
                  % len(self.samples))
            return
        t_end, e_pos_end, e_yaw_end, _, dist = self.samples[-1]
        minutes = t_end / 60.0
        e_v = [s[3] for s in self.samples]
        vel_sigma = (sum(x * x for x in e_v) / len(e_v)) ** 0.5
        # short-horizon jitter ~ white noise: std of sample-to-sample change
        dpos = [abs(self.samples[i][1] - self.samples[i - 1][1])
                for i in range(1, len(self.samples))]
        pos_jitter = (sum(x * x for x in dpos) / len(dpos)) ** 0.5

        profile = {
            "meta": {"samples": len(self.samples), "seconds": round(t_end, 1),
                     "meters_traveled": round(dist, 1)},
            "pos_err_final_m": round(e_pos_end, 3),
            "pos_err_max_m": round(max(s[1] for s in self.samples), 3),
            "yaw_err_final_deg": round(math.degrees(e_yaw_end), 3),
            "yaw_err_max_deg": round(max(math.degrees(abs(s[2]))
                                         for s in self.samples), 3),
            "vel_err_sigma_ms": round(vel_sigma, 4),
            "drift_per_meter": round(e_pos_end / dist, 5) if dist > 1 else None,
            # harness knob calibration (first-order random-walk fit):
            "knobs": {
                "odom_drift_m_per_sqrt_min":
                    round(e_pos_end / math.sqrt(minutes), 3) if minutes > 0.05 else None,
                "odom_yaw_drift_deg_per_sqrt_min":
                    round(abs(math.degrees(e_yaw_end)) / math.sqrt(minutes), 3)
                    if minutes > 0.05 else None,
                "odom_pos_noise": round(pos_jitter, 4),
                "odom_vel_noise": round(vel_sigma, 4),
            },
        }
        with open(out_path, "w") as f:
            json.dump(profile, f, indent=2)
        print(json.dumps(profile, indent=2))
        print("wrote", out_path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--truth", default="/odom")
    ap.add_argument("--estimate", default="/odometry/filtered")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--out", default="odom_profile.json")
    args = ap.parse_args()

    rclpy.init()
    node = OdomProfiler(args.truth, args.estimate, args.duration)
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
