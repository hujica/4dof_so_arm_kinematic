#!/usr/bin/env python3
import numpy as np
from hardware.sts3215_driver import STS3215Driver
from kinematics.arm_kinematics import ArmKinematics

class ArmController:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 1000000):
        self.hardware = STS3215Driver(port, baudrate)
        self.kinematics = ArmKinematics()
        # Safe Default Pose (Arm folded slightly back inside reach limits)
        self.current_joint_angles = [0.0, 30.0, -45.0, 15.0]
        self.current_end_effector_pose = self.kinematics.forward_kinematics(self.current_joint_angles)

    def initialize_arm(self) -> bool:
        self.hardware.connect()
        return True

    def move_joints(self, target_angles: list) -> bool:
        self.current_joint_angles = target_angles
        self.current_end_effector_pose = self.kinematics.forward_kinematics(self.current_joint_angles)
        for idx, angle in enumerate(target_angles):
            self.hardware.set_servo_angle(idx + 1, angle)
        return True

    def move_cartesian(self, dx: float, dy: float, dz: float) -> bool:
        new_x = self.current_end_effector_pose[0] + dx
        new_y = self.current_end_effector_pose[1] + dy
        new_z = self.current_end_effector_pose[2] + dz
        target_pose = [new_x, new_y, new_z, self.current_end_effector_pose[3]]
        
        try:
            target_angles = self.kinematics.inverse_kinematics(target_pose)
            if any(np.isnan(target_angles)):
                print("[ArmController] Out of Workspace Bound!")
                return False
            return self.move_joints(target_angles)
        except Exception as e:
            print(f"[ArmController] IK Calculation Failed: {e}")
            return False

    def home_arm(self) -> None:
        self.move_joints([0.0, 30.0, -45.0, 15.0])

    def emergency_stop(self) -> None:
        print("[ArmController] EMERGENCY STOP TRIGGERED!")