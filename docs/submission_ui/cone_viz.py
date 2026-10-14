#!/usr/bin/env python3
"""
cone_viz.py -- republish eufs_sim2 cone arrays as visualization_msgs/MarkerArray
so stock Foxglove Studio (or RViz) can render them without any custom extension.

Subscribes: /map       (eufs_msgs/msg/ConeWithColorProbabilityArray)
Publishes:  /cones_viz (visualization_msgs/msg/MarkerArray)

Usage:
    ros2 run fs_autonomy cone_viz

Optionally point it at a different topic pair (e.g. the car-relative cones),
with a node rename to avoid a name clash when running two instances:
    ros2 run fs_autonomy cone_viz --ros-args -r __node:=cone_viz_rel \
        -p input_topic:=/cones -p output_topic:=/cones_rel_viz
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from eufs_msgs.msg import ConeWithColorProbabilityArray

# r, g, b in 0..1
COLORS = {
    "blue":       (0.05, 0.35, 1.00),
    "yellow":     (1.00, 0.90, 0.05),
    "orange":     (1.00, 0.45, 0.00),
    "big_orange": (1.00, 0.25, 0.00),
    "unknown":    (0.60, 0.60, 0.60),
}

# (diameter, height) in metres, roughly matching FS cone spec
SIZES = {
    "big_orange": (0.285, 0.505),
    "_default":   (0.228, 0.325),
}


class ConeViz(Node):
    def __init__(self):
        super().__init__("cone_viz")

        self.declare_parameter("input_topic", "/map")
        self.declare_parameter("output_topic", "/cones_viz")
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self._prev_count = 0

        self.pub = self.create_publisher(MarkerArray, out_topic, 1)
        self.create_subscription(
            ConeWithColorProbabilityArray, in_topic, self.on_cones, 10
        )
        self.get_logger().info("Republishing %s -> %s" % (in_topic, out_topic))

    @staticmethod
    def classify(cone):
        """Pick the colour label with the highest probability."""
        probs = {
            "blue":       cone.blue_prob,
            "yellow":     cone.yellow_prob,
            "orange":     cone.orange_prob,
            "big_orange": cone.big_orange_prob,
            "unknown":    cone.unknown_prob,
        }
        return max(probs, key=probs.get)

    def on_cones(self, msg):
        out = MarkerArray()
        frame = msg.header.frame_id or "map"

        for i, cone in enumerate(msg.cones):
            label = self.classify(cone)
            r, g, b = COLORS[label]
            diameter, height = SIZES.get(label, SIZES["_default"])

            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = msg.header.stamp
            m.ns = "cones"
            m.id = i                      # cone.id is always 0 in the sim, so use the index
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(cone.point.x)
            m.pose.position.y = float(cone.point.y)
            m.pose.position.z = float(cone.point.z) + height / 2.0   # sit on the ground
            m.pose.orientation.w = 1.0
            m.scale.x = diameter
            m.scale.y = diameter
            m.scale.z = height
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            out.markers.append(m)

        # clear markers left over from a previously loaded, longer track
        for i in range(len(msg.cones), self._prev_count):
            m = Marker()
            m.header.frame_id = frame
            m.ns = "cones"
            m.id = i
            m.action = Marker.DELETE
            out.markers.append(m)
        self._prev_count = len(msg.cones)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ConeViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
