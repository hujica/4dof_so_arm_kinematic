#!/usr/bin/env python3
import sys
import threading
import tkinter as tk
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor

PROJECT_ROOT = str(Path(__file__).resolve().parent)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controller.arm_controller import ArmController
from ROS.ros2_bridge import ROS2BridgeNode
from gui.main_gui import SOArm101GUI

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000


def main():
    rclpy.init()
    ros_node = ROS2BridgeNode()

    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()

    controller = ArmController(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not controller.initialize_arm():
        print(f"[WARN] Servo hardware not found on {SERIAL_PORT} - "
              "GUI will run in SIMULATION mode.")

    app = SOArm101GUI(root_window=root, controller=controller, ros_node=ros_node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    finally:
        try:
            controller.shutdown()          # zero commands + close serial port
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()