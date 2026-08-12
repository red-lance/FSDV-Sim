#!/usr/bin/env python3
"""
skidpad_driver -- autonomy node for the eufs_sim2 SKIDPAD mission.

Odometry-based figure-8 follower. The FS skidpad geometry is fixed by the
rules, and the sim's skidpad map (map_lib/maps/tracks/skidpad.csv) matches it:
the car spawns on the entry lane centerline 9.8 m before the crossing point,
the two circles have their centers 9.25 m either side of it, and the exit
gate sits 21 m past it. All of that is parameterised below; positions are
tracked relative to the pose latched when DRIVING begins, so absolute map
coordinates never matter.

Phases: ENTRY (straight to the crossing) -> two clockwise laps of the right
circle -> two counter-clockwise laps of the left circle -> EXIT (straight
through the finish gate) -> brake to a stop.

One pure-pursuit steering law drives all phases (the target point is either
ahead on the centerline or ahead on the current circle), and speed is the
same P-on-odom-error acceleration command proven in accel_driver -- the sim
only reads drive.acceleration and drive.steering_angle from /cmd.

Subscribes: /sim/ros_can/state_str (std_msgs/String)      -- gate on
            AMIState: SKIDPAD + ASState: DRIVING
            /odom                  (nav_msgs/Odometry)    -- pose + speed
Publishes:  /cmd                   (ackermann_msgs/AckermannDriveStamped)

Run:  ros2 run fs_autonomy skidpad_driver   (or via autonomy.launch.py)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# phase names double as log labels
ENTRY, RIGHT, LEFT, EXIT, BRAKE = "ENTRY", "RIGHT", "LEFT", "EXIT", "BRAKE"


class SkidpadDriver(Node):
    def __init__(self):
        super().__init__("skidpad_driver")

        self.declare_parameter("odom_topic", "/odom")  # e.g. /odometry/filtered for EKF
        self.declare_parameter("mission", "SKIDPAD")     # AMIState this node responds to
        self.declare_parameter("entry_distance", 9.8)    # m, start pose -> crossing point
        self.declare_parameter("circle_radius", 9.25)    # m, driving line radius
        self.declare_parameter("laps_per_circle", 2)
        self.declare_parameter("exit_distance", 22.0)    # m past the crossing before braking
        self.declare_parameter("target_speed", 4.5)      # m/s (v^2/R lateral accel: keep modest)
        self.declare_parameter("accel_limit", 3.0)       # m/s^2 max forward command
        self.declare_parameter("brake_limit", 4.0)       # m/s^2 max braking magnitude
        self.declare_parameter("kp", 1.5)                # accel per m/s of speed error
        self.declare_parameter("lookahead", 3.0)         # m, pure pursuit lookahead
        self.declare_parameter("wheelbase", 1.53)        # m (ads-dv kinematic.l)
        self.declare_parameter("steer_limit", 0.35)      # rad (model clamps at 0.37)
        self.declare_parameter("stop_speed", 0.1)        # m/s under which we count as stopped
        self.declare_parameter("brake_hold", 1.0)        # m/s^2 held once stopped

        p = self.get_parameter
        self.mission = p("mission").value
        self.entry_distance = p("entry_distance").value
        self.radius = p("circle_radius").value
        self.laps = p("laps_per_circle").value
        self.exit_distance = p("exit_distance").value
        self.target_speed = p("target_speed").value
        self.accel_limit = p("accel_limit").value
        self.brake_limit = p("brake_limit").value
        self.kp = p("kp").value
        self.lookahead = p("lookahead").value
        self.wheelbase = p("wheelbase").value
        self.steer_limit = p("steer_limit").value
        self.stop_speed = p("stop_speed").value
        self.brake_hold = p("brake_hold").value

        self.driving = False
        self.pose = None          # latest raw odom (x, y, yaw)
        self.speed = 0.0
        self.start_pose = None    # (x, y, yaw) latched at DRIVING
        self.phase = ENTRY
        self.turn_angle = 0.0     # accumulated angle around the current circle
        self.prev_phi = None      # previous angle around the current circle center
        self.stopped = False
        self._wrong_mission_logged = False

        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.create_subscription(String, "/sim/ros_can/state_str", self.on_state, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.create_timer(0.02, self.tick)

        self.get_logger().info(
            "Waiting for %s + DRIVING. entry=%.1f m, R=%.2f m, %dx2 laps, exit=%.1f m"
            % (self.mission, self.entry_distance, self.radius, self.laps, self.exit_distance)
        )

    # ------------------------------------------------------------- callbacks

    def on_state(self, msg):
        mine = ("AMIState: %s" % self.mission) in msg.data
        driving = mine and "ASState: DRIVING" in msg.data

        if not mine and "ASState: DRIVING" in msg.data and not self._wrong_mission_logged:
            self.get_logger().info("DRIVING but not my mission (%s) -- idle." % self.mission)
            self._wrong_mission_logged = True

        if driving and not self.driving:
            self.start_pose = self.pose
            self.phase = ENTRY
            self.turn_angle = 0.0
            self.prev_phi = None
            self.stopped = False
            self.get_logger().info("DRIVING -- go. Phase ENTRY.")
        elif not driving and self.driving:
            self.get_logger().info("Left DRIVING -- stopping.")

        self.driving = driving

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)
        self.speed = msg.twist.twist.linear.x

    # ------------------------------------------------------- frame utilities

    def rel_pose(self):
        """Current pose in the start frame (origin at start, x along initial heading)."""
        x0, y0, yaw0 = self.start_pose
        x, y, yaw = self.pose
        dx, dy = x - x0, y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - yaw0))

    # ----------------------------------------------------------- phase logic

    def advance_phase(self, rx, ry):
        """Update self.phase from position (rx, ry) in the start frame."""
        xc = self.entry_distance  # crossing point is at (xc, 0)

        if self.phase == ENTRY and rx >= xc:
            self.phase = RIGHT
            self.turn_angle = 0.0
            self.prev_phi = None
            self.get_logger().info("Crossing reached -- phase RIGHT (%d laps CW)." % self.laps)

        elif self.phase in (RIGHT, LEFT):
            cy = -self.radius if self.phase == RIGHT else self.radius
            phi = math.atan2(ry - cy, rx - xc)
            if self.prev_phi is not None:
                self.turn_angle += wrap(phi - self.prev_phi)
            self.prev_phi = phi

            full = self.laps * 2.0 * math.pi - 0.05  # small tolerance: hand over at the crossing
            if self.phase == RIGHT and self.turn_angle <= -full:
                self.phase = LEFT
                self.turn_angle = 0.0
                self.prev_phi = None
                self.get_logger().info("Right laps done -- phase LEFT (%d laps CCW)." % self.laps)
            elif self.phase == LEFT and self.turn_angle >= full:
                self.phase = EXIT
                self.get_logger().info("Left laps done -- phase EXIT.")

        elif self.phase == EXIT and rx >= xc + self.exit_distance:
            self.phase = BRAKE
            self.get_logger().info(
                "Exit gate passed at %.1f m past crossing (v=%.1f m/s) -- braking."
                % (rx - xc, self.speed)
            )

    def target_point(self, rx, ry):
        """Pure pursuit target in the start frame for the current phase."""
        xc = self.entry_distance

        if self.phase in (ENTRY, EXIT, BRAKE):
            return (rx + self.lookahead, 0.0)  # ahead on the centerline y=0

        cy = -self.radius if self.phase == RIGHT else self.radius
        phi = math.atan2(ry - cy, rx - xc)
        dphi = self.lookahead / self.radius
        phi_t = phi - dphi if self.phase == RIGHT else phi + dphi  # CW vs CCW
        return (
            xc + self.radius * math.cos(phi_t),
            cy + self.radius * math.sin(phi_t),
        )

    # ------------------------------------------------------------------ tick

    def tick(self):
        if not self.driving:
            return
        if self.start_pose is None:
            if self.pose is None:
                return
            self.start_pose = self.pose

        rx, ry, ryaw = self.rel_pose()
        self.advance_phase(rx, ry)

        # steering: pure pursuit toward the phase's target point
        tx, ty = self.target_point(rx, ry)
        dxb = math.cos(-ryaw) * (tx - rx) - math.sin(-ryaw) * (ty - ry)
        dyb = math.sin(-ryaw) * (tx - rx) + math.cos(-ryaw) * (ty - ry)
        alpha = math.atan2(dyb, dxb)
        ld = math.hypot(dxb, dyb)
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        steer = max(-self.steer_limit, min(self.steer_limit, steer))

        # speed: P on odom speed error -> acceleration command
        target = 0.0 if self.phase == BRAKE else self.target_speed
        accel = self.kp * (target - self.speed)
        accel = max(-self.brake_limit, min(self.accel_limit, accel))

        if self.phase == BRAKE and abs(self.speed) < self.stop_speed:
            # hold the brake; the vehicle model zeroes negative accel at v<=0
            accel = -self.brake_hold
            steer = 0.0
            if not self.stopped:
                self.stopped = True
                self.get_logger().info("Stopped %.1f m past the crossing." % (rx - self.entry_distance))

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_footprint"
        cmd.drive.acceleration = float(accel)
        cmd.drive.steering_angle = float(steer)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SkidpadDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
