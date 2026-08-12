#!/usr/bin/env python3
"""wheel_odometry -- adapt wheel speeds into a twist the EKF can fuse.

Converts eufs_msgs/WheelSpeedsStamped (the ros_can wheel-speed feed) into
geometry_msgs/TwistWithCovarianceStamped for robot_localization: forward
velocity from the mean rear wheel speed, vy pinned to ~0 by the nonholonomic
assumption (a car does not slide sideways -- encoded as a small vy variance).

Rear wheels are used because they are undriven by steering geometry; the
front wheels run faster by 1/cos(steering) along their own path. Yaw rate is
NOT derived here -- eufs_sim2 gives both rear wheels identical speed (no
differential model), so wheel-difference yaw is structurally zero; the IMU
owns yaw rate in the EKF config.

Units: the eufs_msgs comment says RPM, but eufs_sim2 computes rev/s
(eufs_core.cpp: v_x / tyre_circumference, no x60) -- upstream inconsistency.
`speed_unit` selects the conversion so the same node runs against the sim
("rps") and a real ADS-DV ("rpm").

Subscribes: /ros_can/wheel_speeds (eufs_msgs/WheelSpeedsStamped)
Publishes:  /wheel_odometry/twist (geometry_msgs/TwistWithCovarianceStamped)
"""

import math

import rclpy
from rclpy.node import Node
from eufs_msgs.msg import WheelSpeedsStamped
from geometry_msgs.msg import TwistWithCovarianceStamped


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_radius", 0.2525)  # m (ads-dv config)
        self.declare_parameter("speed_unit", "rps")     # "rps" (eufs_sim2) | "rpm" (real car)
        self.declare_parameter("vx_stddev", 0.05)       # m/s, trust in wheel-derived vx
        self.declare_parameter("vy_stddev", 0.1)        # m/s, nonholonomic slack

        radius = self.get_parameter("wheel_radius").value
        unit = self.get_parameter("speed_unit").value
        self.circumference = 2.0 * math.pi * radius
        self.per_rev = self.circumference / (60.0 if unit == "rpm" else 1.0)
        vx_var = self.get_parameter("vx_stddev").value ** 2
        vy_var = self.get_parameter("vy_stddev").value ** 2

        self.cov = [0.0] * 36
        self.cov[0] = vx_var    # vx
        self.cov[7] = vy_var    # vy

        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, "/wheel_odometry/twist", 10)
        self.create_subscription(
            WheelSpeedsStamped, "/ros_can/wheel_speeds", self.on_wheel_speeds, 10)

        self.get_logger().info(
            "wheel_odometry up: radius=%.4f m, unit=%s" % (radius, unit))

    def on_wheel_speeds(self, msg):
        rear_mean = 0.5 * (msg.speeds.lb_speed + msg.speeds.rb_speed)

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp  # preserve source (sim) time
        out.header.frame_id = "base_footprint"
        out.twist.twist.linear.x = rear_mean * self.per_rev
        out.twist.twist.linear.y = 0.0
        out.twist.covariance = self.cov
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
