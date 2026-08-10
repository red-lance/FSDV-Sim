#!/usr/bin/env python3
"""Closed-loop test harness for accel_driver, replacing the full sim.

Publishes /odom and a DRIVING state string, subscribes to /cmd, and integrates
the commanded acceleration into simple point-mass dynamics (same no-reverse
rule as the sim's kinematic model). After 40 s it prints one RESULT line:

    RESULT max_v=8.00 min_a=-4.00 brake_x=75.2 stop_x=83.7 final_x=83.7 final_v=0.000

Pass criteria: max_v ~= target_speed, brake_x just past finish_distance,
stop_x set (not "never"), final_v = 0.000.

Run it against the driver (two terminals, no sim required -- works on the
Jetson too):

    ros2 run fs_autonomy accel_driver
    ros2 run fs_autonomy fake_sim
"""
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

DT = 0.02
DURATION = 40.0


class FakeSim(Node):
    def __init__(self):
        super().__init__("fake_sim")
        self.x = 0.0
        self.v = 0.0
        self.a = 0.0
        self.t = 0.0
        self.max_v = 0.0
        self.min_a = 0.0
        self.brake_x = None   # x when a first went negative
        self.stop_x = None    # x when v first ~0 after braking
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.state_pub = self.create_publisher(String, "/sim/ros_can/state_str", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd", self.on_cmd, 10)
        self.create_timer(DT, self.step)

    def on_cmd(self, msg):
        self.a = msg.drive.acceleration

    def step(self):
        self.t += DT
        # same rule as the sim's kinematic model: no reversing under brake
        a = 0.0 if (self.v <= 0.0 and self.a < 0.0) else self.a
        self.v = max(0.0, self.v + a * DT)
        self.x += self.v * DT

        self.max_v = max(self.max_v, self.v)
        self.min_a = min(self.min_a, self.a)
        if self.a < -0.5 and self.brake_x is None:
            self.brake_x = self.x
        if self.brake_x is not None and self.stop_x is None and self.v < 0.05:
            self.stop_x = self.x

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.v
        self.odom_pub.publish(odom)

        s = String()
        s.data = "AMIState: ACCELERATION  ASState: DRIVING"
        self.state_pub.publish(s)

        if self.t >= DURATION:
            print(
                "RESULT max_v=%.2f min_a=%.2f brake_x=%s stop_x=%s final_x=%.1f final_v=%.3f"
                % (
                    self.max_v,
                    self.min_a,
                    "%.1f" % self.brake_x if self.brake_x is not None else "never",
                    "%.1f" % self.stop_x if self.stop_x is not None else "never",
                    self.x,
                    self.v,
                ),
                flush=True,
            )
            raise SystemExit(0)


def main():
    rclpy.init()
    node = FakeSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemExit:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()
