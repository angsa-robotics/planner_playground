#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Vector3


class InteractiveTFBroadcaster(Node):
    def __init__(self):
        super().__init__('interactive_tf_broadcaster')
        self.initial_x = 20.0
        self.initial_y = 21.0
        self.server = InteractiveMarkerServer(self, "tf_marker")
        self.br = TransformBroadcaster(self)
        self.tf_= TransformStamped()
        self.tf_.header.frame_id = "odom"
        self.tf_.child_frame_id = "base_footprint"
        self.tf_.transform.translation.x = self.initial_x
        self.tf_.transform.translation.y = self.initial_y
        self.create_timer(0.1, self.publish_tf)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = "base_footprint_marker"
        int_marker.scale = 1.0
        int_marker.pose.position.x = self.initial_x
        int_marker.pose.position.y = self.initial_y
        int_marker.pose.orientation.w = 1.0
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 1.0
        int_marker.pose.orientation.z = 0.0

        move_control = InteractiveMarkerControl()
        move_control.name = "move_plane"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation_mode = InteractiveMarkerControl.INHERIT
        move_control.always_visible = True
        int_marker.controls.append(move_control)
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        self.tf_.transform.translation = Vector3(x=feedback.pose.position.x,
                                           y=feedback.pose.position.y,
                                           z=feedback.pose.position.z)
        
    def publish_tf(self):
        self.tf_.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.tf_)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()