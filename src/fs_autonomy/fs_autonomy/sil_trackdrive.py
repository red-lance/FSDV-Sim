#!/usr/bin/env python3
"""Closed-loop test harness for trackdrive_driver, replacing the full sim.

Builds a deterministic closed circuit (wobbly ellipse, ~92 m lap, blue cones
on the left edge, yellow on the right, 3.5 m track width), spawns a kinematic
bicycle on it, and emulates a cone sensor: detections are published in the
CAR frame at ~16 Hz, limited to a configurable FoV and range -- the driver
under test never sees the map.

The sensor has a parameterized ERROR MODEL so a real perception stack's
measured behaviour can be replayed against the controller (Monte-Carlo
robustness / perception-requirements sweeps):

    seed                      RNG seed; same seed = same episode
    profile_json              path to error_profile.json produced by
                              scripts/extract_error_profile.py -- range-
                              bucketed P(detect), color accuracy, bearing /
                              range noise, false-positive rate, all measured
                              from a trained detector
    p_detect_scale            multiplies P(detect) everywhere (sweep knob;
                              with no profile it IS the flat detection prob)
    bearing_noise_deg         }  used when no profile is given
    range_noise_frac          }  (profile buckets override per range)
    color_flip_prob           }
    false_positives_per_frame }
    sensor_range / sensor_fov_deg   geometric limits (defaults match the
                              sim camera: 15 m / 110 deg)
    latency_frames            publish detections N frames old (processing lag)
    realtime_factor           run physics N x faster than wall clock; scale
                              the driver's tick_rate by the same factor

After the car stops (or the time cap), prints one RESULT line:

    RESULT laps=3 dev_mean=0.15 dev_max=0.55 cones_hit=0 max_v=4.0 \
           time=95.2 final_v=0.000 seed=0

Pass criteria: laps == the driver's laps parameter, cones_hit=0, dev_max
well under the 1.75 m half-width, final_v=0.000.

Run against the driver (two terminals, isolated domain, no sim required):

    export ROS_DOMAIN_ID=42
    ros2 run fs_autonomy trackdrive_driver --ros-args -p laps:=3
    ros2 run fs_autonomy sil_trackdrive

Batch sweeps: scripts/run_sweeps.py drives both processes and collects CSV.
"""

import json
import math
import random
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import ConeWithColorProbability, ConeWithColorProbabilityArray

DT = 0.02
TIME_CAP = 240.0
WHEELBASE = 1.53
STEER_MAX = 0.37
STEER_TAU = 0.15

# track shape: wobbly ellipse
ELL_A, ELL_B, WOBBLE = 17.0, 12.0, 0.8
HALF_TRACK = 1.75
CONE_SPACING = 3.5

CONE_PUB_EVERY = 3     # physics steps between cone frames (~16 Hz)

# mean |x| of a zero-mean gaussian = sigma * sqrt(2/pi); profile stores means
MEAN_ABS_TO_SIGMA = 1.2533


def build_track():
    """Centerline samples and cone lists, all in map frame."""
    n = 720
    center = []
    for i in range(n):
        th = -math.pi / 2.0 + 2.0 * math.pi * i / n
        px = ELL_A * math.cos(th)
        py = ELL_B * math.sin(th)
        norm = math.hypot(px / ELL_A**2, py / ELL_B**2)
        nx, ny = (px / ELL_A**2) / norm, (py / ELL_B**2) / norm  # outward normal
        w = WOBBLE * math.sin(3.0 * th)
        center.append((px + w * nx, ELL_B + py + w * ny))  # shift so sample 0 ~ origin

    blue, yellow = [], []
    dist = CONE_SPACING  # place the first pair one spacing in
    for i in range(n):
        x0, y0 = center[i]
        x1, y1 = center[(i + 1) % n]
        seg = math.hypot(x1 - x0, y1 - y0)
        dist += seg
        if dist >= CONE_SPACING:
            dist = 0.0
            tx, ty = (x1 - x0) / seg, (y1 - y0) / seg
            lx, ly = -ty, tx  # left of travel
            blue.append((x0 + HALF_TRACK * lx, y0 + HALF_TRACK * ly))
            yellow.append((x0 - HALF_TRACK * lx, y0 - HALF_TRACK * ly))
    return center, blue, yellow


def load_profile(path):
    """Parse error_profile.json into per-bucket arrays with gaps filled."""
    with open(path) as f:
        p = json.load(f)

    def fill(xs, default):
        vals = list(xs)
        for i, v in enumerate(vals):
            if v is None:
                left = next((vals[j] for j in range(i - 1, -1, -1)
                             if vals[j] is not None), None)
                right = next((vals[j] for j in range(i + 1, len(vals))
                              if vals[j] is not None), None)
                vals[i] = left if left is not None else \
                    (right if right is not None else default)
        return vals

    return {
        "edges": p["range_buckets_m"],
        "p_detect": fill(p["p_detect"], 1.0),
        "color_acc": fill(p["color_accuracy"], 1.0),
        "bearing_sigma_deg": [v * MEAN_ABS_TO_SIGMA for v in
                              fill(p["bearing_err_deg_mean"], 0.0)],
        "range_sigma_frac": [v * MEAN_ABS_TO_SIGMA for v in
                             fill(p["range_err_frac_mean"], 0.0)],
        "fp_per_frame": p.get("false_positives_per_image") or 0.0,
    }


class SilTrackdrive(Node):
    def __init__(self):
        super().__init__("sil_trackdrive")
        self.center, self.blue, self.yellow = build_track()

        self.declare_parameter("seed", 0)
        self.declare_parameter("profile_json", "")
        self.declare_parameter("p_detect_scale", 1.0)
        self.declare_parameter("bearing_noise_deg", 0.0)
        self.declare_parameter("range_noise_frac", 0.0)
        self.declare_parameter("color_flip_prob", 0.0)
        self.declare_parameter("false_positives_per_frame", 0.0)
        self.declare_parameter("sensor_range", 15.0)
        self.declare_parameter("sensor_fov_deg", 110.0)
        self.declare_parameter("latency_frames", 0)
        self.declare_parameter("realtime_factor", 1.0)

        gp = lambda k: self.get_parameter(k).value
        self.seed = gp("seed")
        self.rng = random.Random(self.seed)
        self.profile = load_profile(gp("profile_json")) if gp("profile_json") else None
        self.p_detect_scale = gp("p_detect_scale")
        self.bearing_noise_deg = gp("bearing_noise_deg")
        self.range_noise_frac = gp("range_noise_frac")
        self.color_flip_prob = gp("color_flip_prob")
        self.fp_per_frame = gp("false_positives_per_frame")
        self.sensor_range = gp("sensor_range")
        self.fov_half = math.radians(gp("sensor_fov_deg")) / 2.0
        self.latency_frames = gp("latency_frames")
        rt = gp("realtime_factor")

        if self.profile:
            self.fp_per_frame = self.profile["fp_per_frame"]
            self.get_logger().info(
                "Error profile loaded: %s (fp/frame=%.2f)"
                % (gp("profile_json"), self.fp_per_frame))

        x0, y0 = self.center[0]
        x1, y1 = self.center[1]
        self.x, self.y = x0, y0
        self.yaw = math.atan2(y1 - y0, x1 - x0)
        self.v = 0.0
        self.a_cmd = 0.0
        self.steer_cmd = 0.0
        self.steer = 0.0
        self.t = 0.0
        self.step_count = 0

        self.max_v = 0.0
        self.dev_sum = 0.0
        self.dev_max = 0.0
        self.dev_n = 0
        self.hit = set()
        self.laps = 0
        self.lap_armed = False
        self.moved = False
        self.stopped_since = None
        self.frame_buf = deque()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.state_pub = self.create_publisher(String, "/sim/ros_can/state_str", 10)
        self.cone_pub = self.create_publisher(ConeWithColorProbabilityArray, "/cones", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd", self.on_cmd, 10)
        self.create_timer(DT / rt, self.step)
        self.get_logger().info(
            "Track: %d cones/side, lap ~%.0f m. seed=%d"
            % (len(self.blue), self.lap_length(), self.seed)
        )

    def lap_length(self):
        n = len(self.center)
        return sum(
            math.hypot(self.center[(i + 1) % n][0] - self.center[i][0],
                       self.center[(i + 1) % n][1] - self.center[i][1])
            for i in range(n)
        )

    def on_cmd(self, msg):
        self.a_cmd = msg.drive.acceleration
        self.steer_cmd = max(-STEER_MAX, min(STEER_MAX, msg.drive.steering_angle))

    def error_at(self, r):
        """(p_detect, color_flip_prob, bearing_sigma_deg, range_sigma_frac) at range r."""
        if self.profile is None:
            return (self.p_detect_scale, self.color_flip_prob,
                    self.bearing_noise_deg, self.range_noise_frac)
        edges = self.profile["edges"]
        b = len(edges) - 2  # clamp to last bucket beyond the data
        for i in range(len(edges) - 1):
            if edges[i] <= r < edges[i + 1]:
                b = i
                break
        return (self.profile["p_detect"][b] * self.p_detect_scale,
                1.0 - self.profile["color_acc"][b],
                self.profile["bearing_sigma_deg"][b],
                self.profile["range_sigma_frac"][b])

    def poisson(self, lam):
        if lam <= 0.0:
            return 0
        limit = math.exp(-lam)
        k, p = 0, 1.0
        while True:
            p *= self.rng.random()
            if p <= limit:
                return k
            k += 1

    def sense_frame(self):
        """One sensor frame through the error model: [(bx, by, color), ...]."""
        dets = []
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        for cones, color in ((self.blue, "blue"), (self.yellow, "yellow")):
            for cx, cy in cones:
                dx, dy = cx - self.x, cy - self.y
                bx, by = c * dx - s * dy, s * dx + c * dy
                r = math.hypot(bx, by)
                th = math.atan2(by, bx)
                if r > self.sensor_range or abs(th) > self.fov_half:
                    continue
                p_det, flip, b_sig, r_sig = self.error_at(r)
                if self.rng.random() > p_det:
                    continue
                if r_sig > 0.0:
                    r = max(0.5, r * (1.0 + self.rng.gauss(0.0, r_sig)))
                if b_sig > 0.0:
                    th += math.radians(self.rng.gauss(0.0, b_sig))
                col = color
                if flip > 0.0 and self.rng.random() < flip:
                    col = "yellow" if color == "blue" else "blue"
                dets.append((r * math.cos(th), r * math.sin(th), col))

        for _ in range(self.poisson(self.fp_per_frame)):
            r = self.rng.uniform(1.0, self.sensor_range)
            th = self.rng.uniform(-self.fov_half, self.fov_half)
            dets.append((r * math.cos(th), r * math.sin(th),
                         self.rng.choice(("blue", "yellow"))))
        return dets

    def publish_cones(self):
        self.frame_buf.append(self.sense_frame())
        if len(self.frame_buf) <= self.latency_frames:
            return  # not enough history to serve the delayed frame yet
        dets = self.frame_buf.popleft()

        out = ConeWithColorProbabilityArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        for bx, by, col in dets:
            cone = ConeWithColorProbability()
            cone.point.x, cone.point.y = bx, by
            if col == "blue":
                cone.blue_prob = 1.0
            else:
                cone.yellow_prob = 1.0
            out.cones.append(cone)
        self.cone_pub.publish(out)

    def step(self):
        self.t += DT
        self.step_count += 1

        self.steer += (self.steer_cmd - self.steer) * DT / STEER_TAU
        a = 0.0 if (self.v <= 0.0 and self.a_cmd < 0.0) else self.a_cmd
        self.v = max(0.0, self.v + a * DT)
        self.yaw += self.v / WHEELBASE * math.tan(self.steer) * DT
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT

        self.max_v = max(self.max_v, self.v)
        if self.v > 0.5:
            self.moved = True
            dev = min(math.hypot(self.x - px, self.y - py) for px, py in self.center)
            self.dev_sum += dev
            self.dev_max = max(self.dev_max, dev)
            self.dev_n += 1
            for idx, (cx, cy) in enumerate(self.blue + self.yellow):
                if math.hypot(self.x - cx, self.y - cy) < 0.5:
                    self.hit.add(idx)

        d_start = math.hypot(self.x - self.center[0][0], self.y - self.center[0][1])
        if not self.lap_armed and d_start > 10.0:
            self.lap_armed = True
        elif self.lap_armed and d_start < 4.0:
            self.lap_armed = False
            self.laps += 1

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        self.odom_pub.publish(odom)

        s = String()
        s.data = "AMIState: TRACK_DRIVE  ASState: DRIVING"
        self.state_pub.publish(s)

        if self.step_count % CONE_PUB_EVERY == 0:
            self.publish_cones()

        # finish when the car has driven and then stayed stopped for 3 s
        if self.moved and self.v < 0.02:
            if self.stopped_since is None:
                self.stopped_since = self.t
        else:
            self.stopped_since = None

        if (self.stopped_since and self.t - self.stopped_since > 3.0) or self.t >= TIME_CAP:
            dev_mean = self.dev_sum / self.dev_n if self.dev_n else -1.0
            print(
                "RESULT laps=%d dev_mean=%.3f dev_max=%.3f cones_hit=%d "
                "max_v=%.2f time=%.1f final_v=%.3f seed=%d"
                % (self.laps, dev_mean, self.dev_max, len(self.hit),
                   self.max_v, self.t, self.v, self.seed),
                flush=True,
            )
            raise SystemExit(0)


def main():
    rclpy.init()
    node = SilTrackdrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()
