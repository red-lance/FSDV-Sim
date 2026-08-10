#!/usr/bin/env python3
"""
trackdrive_driver -- autonomy node for the eufs_sim2 TRACK_DRIVE and
AUTOCROSS missions.

The first controller in this package that drives from perception instead of
odometry: the track is unknown, so steering comes from /cones -- car-relative
cone detections (eufs_msgs/ConeWithColorProbabilityArray, base_footprint
frame). Blue cones mark the left edge of the lane, yellow the right. Each
tick we classify the visible cones, pair the nearest left/right markers, and
pure-pursuit toward the pair's midpoint. If only one side is visible the
target is offset half a track width from it; if nothing is visible we slow
down and hold the last steering (and stop if blind for too long).

Odometry is used only for lap counting (leave the start area, come back) and
speed feedback -- never for path geometry. Laps default to 10 (TRACK_DRIVE);
run the same executable with mission:=AUTOCROSS laps:=1 for autocross.

Speed scales down with commanded steering: fast on straights, slow in
corners. Same P-on-speed-error acceleration law as the other controllers
(the sim reads only drive.acceleration + drive.steering_angle from /cmd).

Subscribes: /sim/ros_can/state_str (std_msgs/String)
            /odom                  (nav_msgs/Odometry)
            /cones                 (eufs_msgs/ConeWithColorProbabilityArray)
Publishes:  /cmd                   (ackermann_msgs/AckermannDriveStamped)

Run:  ros2 run fs_autonomy trackdrive_driver   (or via autonomy.launch.py)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import ConeWithColorProbabilityArray


def classify(cone):
    """Highest-probability colour label for a cone."""
    probs = {
        "blue": cone.blue_prob,
        "yellow": cone.yellow_prob,
        "orange": cone.orange_prob,
        "big_orange": cone.big_orange_prob,
        "unknown": cone.unknown_prob,
    }
    return max(probs, key=probs.get)


class TrackdriveDriver(Node):
    def __init__(self):
        super().__init__("trackdrive_driver")

        self.declare_parameter("mission", "TRACK_DRIVE")  # AMIState this node responds to
        # eufs_sim2's fused /cones publisher exists but is never published
        # (upstream bug: perception_cones_pub_ is created and never used).
        # /cones/lenient carries the actual FoV-filtered car-relative cones.
        self.declare_parameter("cones_topic", "/cones")
        self.declare_parameter("laps", 10)               # FS trackdrive = 10, autocross = 1
        self.declare_parameter("target_speed", 4.0)      # m/s on straights
        self.declare_parameter("min_speed", 1.5)         # m/s floor in the tightest corners
        self.declare_parameter("max_cone_range", 14.0)   # m, ignore detections beyond this
        self.declare_parameter("min_target_dist", 2.0)   # m, skip pair midpoints closer than this
        self.declare_parameter("half_track", 1.75)       # m, offset when only one side is visible
        self.declare_parameter("cone_timeout", 0.5)      # s without cones -> slow + hold steering
        self.declare_parameter("blind_stop_time", 2.0)   # s without cones -> brake to a stop
        self.declare_parameter("lap_arm_distance", 10.0)  # m from start to arm the lap counter
        self.declare_parameter("lap_close_distance", 4.0)  # m from start to count a lap
        self.declare_parameter("accel_limit", 3.0)
        self.declare_parameter("brake_limit", 4.0)
        self.declare_parameter("kp", 1.5)
        self.declare_parameter("wheelbase", 1.53)
        self.declare_parameter("steer_limit", 0.35)
        self.declare_parameter("stop_speed", 0.1)
        self.declare_parameter("brake_hold", 1.0)

        p = self.get_parameter
        self.mission = p("mission").value
        self.laps_total = p("laps").value
        self.target_speed = p("target_speed").value
        self.min_speed = p("min_speed").value
        self.max_cone_range = p("max_cone_range").value
        self.min_target_dist = p("min_target_dist").value
        self.half_track = p("half_track").value
        self.cone_timeout = p("cone_timeout").value
        self.blind_stop_time = p("blind_stop_time").value
        self.lap_arm_distance = p("lap_arm_distance").value
        self.lap_close_distance = p("lap_close_distance").value
        self.accel_limit = p("accel_limit").value
        self.brake_limit = p("brake_limit").value
        self.kp = p("kp").value
        self.wheelbase = p("wheelbase").value
        self.steer_limit = p("steer_limit").value
        self.stop_speed = p("stop_speed").value
        self.brake_hold = p("brake_hold").value

        self.driving = False
        self.position = None
        self.speed = 0.0
        self.start_xy = None
        self.laps = 0
        self.lap_armed = False
        self.finished = False
        self.stopped = False
        self.cones_msg = None
        self.cones_rx_time = None
        self.last_steer = 0.0
        self._wrong_mission_logged = False

        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.create_subscription(String, "/sim/ros_can/state_str", self.on_state, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(
            ConeWithColorProbabilityArray, p("cones_topic").value, self.on_cones, 10
        )
        self.create_timer(0.02, self.tick)

        self.get_logger().info(
            "Waiting for %s + DRIVING. %d lap(s), target_speed=%.1f m/s"
            % (self.mission, self.laps_total, self.target_speed)
        )

    # ------------------------------------------------------------- callbacks

    def on_state(self, msg):
        mine = ("AMIState: %s" % self.mission) in msg.data
        driving = mine and "ASState: DRIVING" in msg.data

        if not mine and "ASState: DRIVING" in msg.data and not self._wrong_mission_logged:
            self.get_logger().info("DRIVING but not my mission (%s) -- idle." % self.mission)
            self._wrong_mission_logged = True

        if driving and not self.driving:
            self.start_xy = self.position
            self.laps = 0
            self.lap_armed = False
            self.finished = False
            self.stopped = False
            self.last_steer = 0.0
            self.get_logger().info("DRIVING -- go. %d lap(s) to run." % self.laps_total)
        elif not driving and self.driving:
            self.get_logger().info("Left DRIVING -- stopping.")

        self.driving = driving

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.position = (pos.x, pos.y)
        self.speed = msg.twist.twist.linear.x

        if self.start_xy is None or self.finished:
            return
        dist = math.hypot(pos.x - self.start_xy[0], pos.y - self.start_xy[1])
        if not self.lap_armed and dist > self.lap_arm_distance:
            self.lap_armed = True
        elif self.lap_armed and dist < self.lap_close_distance:
            self.lap_armed = False
            self.laps += 1
            self.get_logger().info("Lap %d/%d." % (self.laps, self.laps_total))
            if self.laps >= self.laps_total:
                self.finished = True
                self.get_logger().info("All laps done (v=%.1f m/s) -- braking." % self.speed)

    def on_cones(self, msg):
        self.cones_msg = msg
        self.cones_rx_time = self.get_clock().now()

    # -------------------------------------------------------------- steering

    def cones_age(self):
        if self.cones_rx_time is None:
            return float("inf")
        return (self.get_clock().now() - self.cones_rx_time).nanoseconds * 1e-9

    def pick_target(self):
        """Target point in the car frame from the latest cone detections.

        Returns (x, y) or None when nothing usable is visible.
        """
        left, right = [], []
        for cone in self.cones_msg.cones:
            x, y = cone.point.x, cone.point.y
            r = math.hypot(x, y)
            if x < 0.2 or r > self.max_cone_range:
                continue
            label = classify(cone)
            if label == "blue":
                left.append((r, x, y))
            elif label == "yellow":
                right.append((r, x, y))
            elif label in ("orange", "big_orange"):
                # start/finish markers line both edges; split by side
                (left if y > 0.0 else right).append((r, x, y))

        left.sort()
        right.sort()

        if left and right:
            # midpoints of nearest pairs; take the first far enough ahead
            candidates = []
            for (_, bx, by), (_, yx, yy) in zip(left[:3], right[:3]):
                mx, my = (bx + yx) / 2.0, (by + yy) / 2.0
                candidates.append((math.hypot(mx, my), mx, my))
            for dist, mx, my in candidates:
                if dist >= self.min_target_dist:
                    return (mx, my)
            return (candidates[-1][1], candidates[-1][2])

        if left:
            _, cx, cy = left[0]
            return (cx, cy - self.half_track)   # blue is the left edge: aim right of it
        if right:
            _, cx, cy = right[0]
            return (cx, cy + self.half_track)   # yellow is the right edge: aim left of it
        return None

    # ------------------------------------------------------------------ tick

    def tick(self):
        if not self.driving:
            return
        if self.start_xy is None:
            if self.position is None:
                return
            self.start_xy = self.position

        age = self.cones_age()
        target = None
        if self.cones_msg is not None and age <= self.cone_timeout:
            target = self.pick_target()

        if target is not None:
            tx, ty = target
            alpha = math.atan2(ty, tx)
            ld = math.hypot(tx, ty)
            steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
            steer = max(-self.steer_limit, min(self.steer_limit, steer))
            self.last_steer = steer
            blind = False
        else:
            steer = self.last_steer  # hold the line, slow down
            blind = True

        # corner slowdown: fast when straight, min_speed at full lock
        slow = abs(steer) / self.steer_limit
        v_target = self.target_speed - (self.target_speed - self.min_speed) * slow
        if blind:
            v_target = self.min_speed
        if self.finished or (blind and age > self.blind_stop_time):
            v_target = 0.0

        accel = self.kp * (v_target - self.speed)
        accel = max(-self.brake_limit, min(self.accel_limit, accel))

        if v_target == 0.0 and abs(self.speed) < self.stop_speed:
            accel = -self.brake_hold
            steer = 0.0
            if self.finished and not self.stopped:
                self.stopped = True
                self.get_logger().info("Stopped after %d laps." % self.laps)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_footprint"
        cmd.drive.acceleration = float(accel)
        cmd.drive.steering_angle = float(steer)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = TrackdriveDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
