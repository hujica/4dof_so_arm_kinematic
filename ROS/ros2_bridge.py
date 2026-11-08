#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class ROS2BridgeNode(Node):
    """ROS 2 Humble Node for Publishing Joint States"""
    
    def __init__(self):
        super().__init__('so_arm101_ros2_bridge')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish_joint_states(self, joint_angles_deg: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        msg.position = [float(np.radians(a)) for a in joint_angles_deg]
        self.joint_pub.publish(msg)