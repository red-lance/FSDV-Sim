#!/usr/bin/env python3
"""
accel_driver -- autonomy node for the eufs_sim2 ACCELERATION mission.

The sim's VCU hands control over once ASState reaches DRIVING, but nothing
publishes /cmd by default -- so the car just sits there. This node fills that
gap: drive straight under closed-loop speed control, brake to a stop past the
finish line.

The sim consumes ONLY drive.acceleration and drive.steering_angle from /cmd
(eufs_sim2/src/control.cpp); drive.speed is ignored entirely. Speed is
therefore regulated here: a P controller on the odometry speed error outputs
an acceleration command. The vehicle model applies negative acceleration as
braking and clamps it to zero once the car has stopped, so holding a brake
command cannot reverse the car.

Subscribes: /sim/ros_can/state_str (std_msgs/String)      -- gate on DRIVING
            /odom                  (nav_msgs/Odometry)    -- distance + speed
Publishes:  /cmd                   (ackermann_msgs/AckermannDriveStamped)

Run:
    ros2 run fs_autonomy accel_driver
    ros2 launch fs_autonomy accel.launch.py     # loads config/accel_params.yaml

Then in Foxglove: Set Mission -> ACCELERATION, then GO.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class AccelDriver(Node):
    def __init__(self):
        super().__init__("accel_driver")

        self.declare_parameter("odom_topic", "/odom")  # e.g. /odometry/filtered for EKF
        self.declare_parameter("mission", "ACCELERATION")  # AMIState this node responds to
        self.declare_parameter("target_speed", 8.0)      # m/s to hold on the run
        self.declare_parameter("finish_distance", 75.0)  # m before braking (FS accel = 75 m)
        self.declare_parameter("accel_limit", 3.0)       # m/s^2 max forward command
        self.declare_parameter("brake_limit", 4.0)       # m/s^2 max braking magnitude
        self.declare_parameter("kp", 1.5)                # accel per m/s of speed error
        self.declare_parameter("stop_speed", 0.1)        # m/s under which we count as stopped
        self.declare_parameter("brake_hold", 1.0)        # m/s^2 held once stopped (model zeroes it at v<=0)
        self.declare_parameter("publish_rate", 50.0)     # Hz

        self.mission = self.get_parameter("mission").value
        self.target_speed = self.get_parameter("target_speed").value
        self.finish_distance = self.get_parameter("finish_distance").value
        self.accel_limit = self.get_parameter("accel_limit").value
        self.brake_limit = self.get_parameter("brake_limit").value
        self.kp = self.get_parameter("kp").value
        self.stop_speed = self.get_parameter("stop_speed").value
        self.brake_hold = self.get_parameter("brake_hold").value
        rate = self.get_parameter("publish_rate").value

        self.driving = False      # is ASState == DRIVING?
        self.start_xy = None      # pose captured when DRIVING begins
        self.position = None      # latest odom position
        self.speed = 0.0          # latest odom forward speed (body frame)
        self.distance = 0.0       # metres from start
        self.braking = False      # latched once finish_distance is crossed
        self.stopped = False
        self._wrong_mission_logged = False

        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.create_subscription(String, "/sim/ros_can/state_str", self.on_state, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            "Waiting for %s + DRIVING. target_speed=%.1f m/s, finish=%.1f m"
            % (self.mission, self.target_speed, self.finish_distance)
        )

    def on_state(self, msg):
        # engage only when OUR mission is selected AND the VCU has released
        # control -- this is what links the Foxglove mission dropdown to this
        # node; other mission controllers can run alongside and stay idle
        mine = ("AMIState: %s" % self.mission) in msg.data
        driving = mine and "ASState: DRIVING" in msg.data

        if not mine and "ASState: DRIVING" in msg.data and not self._wrong_mission_logged:
            self.get_logger().info("DRIVING but not my mission (%s) -- idle." % self.mission)
            self._wrong_mission_logged = True

        if driving and not self.driving:
            # just entered DRIVING -- latch the start pose and reset the run
            self.start_xy = self.position
            self.distance = 0.0
            self.braking = False
            self.stopped = False
            self.get_logger().info("DRIVING -- go.")
        elif not driving and self.driving:
            self.get_logger().info("Left DRIVING -- stopping.")

        self.driving = driving

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)
        self.speed = msg.twist.twist.linear.x

        if self.start_xy is None:
            return
        dx = p.x - self.start_xy[0]
        dy = p.y - self.start_xy[1]
        self.distance = math.hypot(dx, dy)

    def tick(self):
        if not self.driving:
            return

        # odom may not have arrived before the state change; latch late if so
        if self.start_xy is None:
            if self.position is None:
                return
            self.start_xy = self.position

        if not self.braking and self.distance >= self.finish_distance:
            self.braking = True
            self.get_logger().info(
                "Finish line at %.1f m (v=%.1f m/s) -- braking." % (self.distance, self.speed)
            )

        target = 0.0 if self.braking else self.target_speed
        accel = self.kp * (target - self.speed)
        accel = max(-self.brake_limit, min(self.accel_limit, accel))

        if self.braking and abs(self.speed) < self.stop_speed:
            # hold the brake rather than coasting; the vehicle model clamps
            # negative acceleration to zero once stopped, so this cannot reverse
            accel = -self.brake_hold
            if not self.stopped:
                self.stopped = True
                self.get_logger().info("Stopped at %.1f m." % self.distance)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_footprint"
        cmd.drive.acceleration = float(accel)
        cmd.drive.steering_angle = 0.0        # acceleration is a straight run
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AccelDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
