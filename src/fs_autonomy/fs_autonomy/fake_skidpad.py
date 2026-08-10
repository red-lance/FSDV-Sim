#!/usr/bin/env python3
"""Closed-loop test harness for skidpad_driver, replacing the full sim.

2D kinematic bicycle (rear axle reference, ads-dv wheelbase, steering clamped
to +/-0.37 rad with a first-order actuator lag). Publishes /odom and a
"AMIState: SKIDPAD ASState: DRIVING" state string, integrates the /cmd
acceleration + steering commands, and after the run prints one RESULT line:

    RESULT max_v=4.50 circ_err_mean=0.05 circ_err_max=0.18 turn_total=25.1 \
           net_yaw=0.01 final_x=34.5 final_y=-0.05 final_v=0.000

Pass criteria: turn_total ~= 8*pi = 25.1 (two laps each way), net_yaw ~= 0,
circ_err_max well inside the 1.5 m cone lane half-width, final_y ~= 0,
final_x past entry+exit distance, final_v = 0.000.

Run against the driver (two terminals, no sim required):

    ros2 run fs_autonomy skidpad_driver
    ros2 run fs_autonomy fake_skidpad
"""

import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

DT = 0.02
DURATION = 100.0
WHEELBASE = 1.53
STEER_MAX = 0.37
STEER_TAU = 0.15      # actuator lag [s]

# skidpad geometry matching map_lib/maps/tracks/skidpad.csv (car spawns at origin)
CROSS_X = 9.8
RADIUS = 9.25


class FakeSkidpad(Node):
    def __init__(self):
        super().__init__("fake_skidpad")
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.a_cmd = 0.0
        self.steer_cmd = 0.0
        self.steer = 0.0
        self.t = 0.0

        self.max_v = 0.0
        self.turn_total = 0.0
        self.circ_err_sum = 0.0
        self.circ_err_max = 0.0
        self.circ_samples = 0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.state_pub = self.create_publisher(String, "/sim/ros_can/state_str", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd", self.on_cmd, 10)
        self.create_timer(DT, self.step)

    def on_cmd(self, msg):
        self.a_cmd = msg.drive.acceleration
        self.steer_cmd = max(-STEER_MAX, min(STEER_MAX, msg.drive.steering_angle))

    def step(self):
        self.t += DT

        # actuator lag, then kinematic bicycle; same no-reverse rule as the sim
        self.steer += (self.steer_cmd - self.steer) * DT / STEER_TAU
        a = 0.0 if (self.v <= 0.0 and self.a_cmd < 0.0) else self.a_cmd
        self.v = max(0.0, self.v + a * DT)
        dyaw = self.v / WHEELBASE * math.tan(self.steer) * DT
        self.yaw += dyaw
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT

        self.max_v = max(self.max_v, self.v)
        self.turn_total += abs(dyaw)
        # radial error to the nearest circle, sampled only when clearly off
        # the centerline strip (entry/exit points would skew it otherwise)
        if abs(self.y) > 1.0:
            err = min(
                abs(math.hypot(self.x - CROSS_X, self.y + RADIUS) - RADIUS),
                abs(math.hypot(self.x - CROSS_X, self.y - RADIUS) - RADIUS),
            )
            self.circ_err_sum += err
            self.circ_err_max = max(self.circ_err_max, err)
            self.circ_samples += 1

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
        s.data = "AMIState: SKIDPAD  ASState: DRIVING"
        self.state_pub.publish(s)

        if self.t >= DURATION:
            mean = self.circ_err_sum / self.circ_samples if self.circ_samples else -1.0
            print(
                "RESULT max_v=%.2f circ_err_mean=%.3f circ_err_max=%.3f "
                "turn_total=%.1f net_yaw=%.2f final_x=%.1f final_y=%.2f final_v=%.3f"
                % (self.max_v, mean, self.circ_err_max, self.turn_total,
                   wrapped_net(self.yaw), self.x, self.y, self.v),
                flush=True,
            )
            raise SystemExit(0)


def wrapped_net(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def main():
    rclpy.init()
    node = FakeSkidpad()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()
