#!/usr/bin/env python3
"""Closed-loop test harness for trackdrive_driver, replacing the full sim.

Builds a deterministic closed circuit (wobbly ellipse, ~92 m lap, blue cones
on the left edge, yellow on the right, 3.5 m track width), spawns a kinematic
bicycle on it, and emulates the sim's cone sensor: detections are published
in the CAR frame at ~15 Hz, limited to a 110 degree FoV and 15 m range --
the driver under test never sees the map.

After the car stops (or the time cap), prints one RESULT line:

    RESULT laps=3 dev_mean=0.15 dev_max=0.55 cones_hit=0 max_v=4.0 \
           time=95.2 final_v=0.000

Pass criteria: laps == the driver's laps parameter, cones_hit=0, dev_max
well under the 1.75 m half-width, final_v=0.000.

Run against the driver (two terminals, isolated domain, no sim required):

    export ROS_DOMAIN_ID=42
    ros2 run fs_autonomy trackdrive_driver --ros-args -p laps:=3
    ros2 run fs_autonomy fake_trackdrive
"""

import math
import sys

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

# sensor model (matches the sim's camera in plugin_params.yaml)
FOV_HALF = 0.96        # rad (~110 deg total)
RANGE_MAX = 15.0
CONE_PUB_EVERY = 3     # physics steps between cone frames (~16 Hz)


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


class FakeTrackdrive(Node):
    def __init__(self):
        super().__init__("fake_trackdrive")
        self.center, self.blue, self.yellow = build_track()

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

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.state_pub = self.create_publisher(String, "/sim/ros_can/state_str", 10)
        self.cone_pub = self.create_publisher(ConeWithColorProbabilityArray, "/cones", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd", self.on_cmd, 10)
        self.create_timer(DT, self.step)
        self.get_logger().info(
            "Track: %d cones/side, lap ~%.0f m." % (len(self.blue), self.lap_length())
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

    def publish_cones(self):
        out = ConeWithColorProbabilityArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        for cones, color in ((self.blue, "blue"), (self.yellow, "yellow")):
            for cx, cy in cones:
                dx, dy = cx - self.x, cy - self.y
                bx, by = c * dx - s * dy, s * dx + c * dy
                r = math.hypot(bx, by)
                if r > RANGE_MAX or abs(math.atan2(by, bx)) > FOV_HALF:
                    continue
                cone = ConeWithColorProbability()
                cone.point.x, cone.point.y = bx, by
                if color == "blue":
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
                "max_v=%.2f time=%.1f final_v=%.3f"
                % (self.laps, dev_mean, self.dev_max, len(self.hit),
                   self.max_v, self.t, self.v),
                flush=True,
            )
            raise SystemExit(0)


def main():
    rclpy.init()
    node = FakeTrackdrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()
