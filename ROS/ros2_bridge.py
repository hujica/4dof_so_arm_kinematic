#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__("so_arm101_gui_bridge")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]

    def publish_joint_states(self, angles_deg, velocities_deg_s):
        if not rclpy.ok():
            return False
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [math.radians(a) for a in angles_deg]
            msg.velocity = [math.radians(v) for v in velocities_deg_s]
            self.joint_pub.publish(msg)
            return True
        except Exception:
            return False