#!/usr/bin/env python3
"""imu_frontend -- republish the sim IMU with covariance fields filled.

eufs_sim2 injects noise into the IMU *values* (sensor_plugin noise sampler)
but never copies the configured covariance into the message fields, so
/imu/data goes out with all-zero covariances. robot_localization weighs each
sensor by its message covariance -- zeros would make it blindly over-trust
the IMU. This node stamps configured variances onto the stream:

    /imu/data  ->  /imu/data_cov

Keep gyro_stddev / accel_stddev in sync with the sim's imu_plugin
noise_covariance (plugin_params.yaml). Candidate upstream fix: the plugin
already holds sensor_data_.covariance; the message converter should fill it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFrontend(Node):
    def __init__(self):
        super().__init__("imu_frontend")

        self.declare_parameter("gyro_stddev", 0.01)    # rad/s
        self.declare_parameter("accel_stddev", 0.05)   # m/s^2

        gyro_var = self.get_parameter("gyro_stddev").value ** 2
        accel_var = self.get_parameter("accel_stddev").value ** 2

        self.gyro_cov = [0.0] * 9
        self.accel_cov = [0.0] * 9
        for i in (0, 4, 8):
            self.gyro_cov[i] = gyro_var
            self.accel_cov[i] = accel_var
        # orientation comes from true state in the sim; we do not fuse it
        # (dead-reckoning study), mark it wide so nothing downstream trusts it
        self.orient_cov = [0.0] * 9
        for i in (0, 4, 8):
            self.orient_cov[i] = 1000.0

        self.pub = self.create_publisher(Imu, "/imu/data_cov", 50)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)

    def on_imu(self, msg):
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration_covariance = self.accel_cov
        msg.orientation_covariance = self.orient_cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFrontend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
