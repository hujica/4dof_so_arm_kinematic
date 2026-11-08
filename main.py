#!/usr/bin/env python3
import sys
import tkinter as tk
import rclpy
from controller.arm_controller import ArmController
from ROS.ros2_bridge import ROS2BridgeNode
from gui.main_gui import SOArm101GUI

def main():
    # 1. Initialize ROS 2
    rclpy.init(args=None)
    ros_node = ROS2BridgeNode()

    # 2. Initialize Controller Architecture
    controller = ArmController(port="/dev/ttyUSB0", baudrate=1000000)
    controller.initialize_arm()

    # 3. Create GUI App
    root = tk.Tk()
    app = SOArm101GUI(root, controller, ros_node)

    # 4. Integrated GUI and ROS 2 Event Loop
    def ros_spin_loop():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        root.after(10, ros_spin_loop)

    root.after(10, ros_spin_loop)
    root.mainloop()

    # Clean Shutdown
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()