#!/usr/bin/env python3
import numpy as np

class ArmKinematics:
    """Forward & Inverse Kinematics for 4-DOF SO-ARM101"""
    
    def __init__(self, link_lengths: list = None):
        self.link_lengths = link_lengths if link_lengths is not None else [0.08, 0.12, 0.12, 0.06]
        self.dh_params = [
            [0,                    np.pi/2, self.link_lengths[0], 0],
            [self.link_lengths[1], 0,       0,                   0],
            [self.link_lengths[2], 0,       0,                   0],
            [self.link_lengths[3], 0,       0,                   0]
        ]

    def forward_kinematics(self, joint_angles: list) -> list:
        q_rad = np.radians(joint_angles)
        l1, l2, l3, l4 = self.link_lengths
        th1, th2, th3, th4 = q_rad[0], q_rad[1], q_rad[2], q_rad[3]
        phi = th2 + th3 + th4
        
        r = l2 * np.cos(th2) + l3 * np.cos(th2 + th3) + l4 * np.cos(phi)
        x = r * np.cos(th1)
        y = r * np.sin(th1)
        z = l1 + l2 * np.sin(th2) + l3 * np.sin(th2 + th3) + l4 * np.sin(phi)
        return [float(x), float(y), float(z), float(np.degrees(phi))]

    def inverse_kinematics(self, target_pose: list) -> list:
        x, y, z = target_pose[0], target_pose[1], target_pose[2]
        phi_deg = target_pose[3] if len(target_pose) > 3 else 0.0
        l1, l2, l3, l4 = self.link_lengths
        phi = np.radians(phi_deg)
        
        q1 = np.arctan2(y, x)
        R = np.sqrt(x**2 + y**2)
        Rw = R - l4 * np.cos(phi)
        Zw = z - l1 - l4 * np.sin(phi)
        
        d_sq = Rw**2 + Zw**2
        cos_th3 = (d_sq - l2**2 - l3**2) / (2.0 * l2 * l3)
        cos_th3 = np.clip(cos_th3, -1.0, 1.0)
        
        q3 = np.arctan2(-np.sqrt(1.0 - cos_th3**2), cos_th3)
        k1 = l2 + l3 * np.cos(q3)
        k2 = l3 * np.sin(q3)
        q2 = np.arctan2(Zw, Rw) - np.arctan2(k2, k1)
        q4 = phi - (q2 + q3)
        
        return [float(np.degrees(q1)), float(np.degrees(q2)), float(np.degrees(q3)), float(np.degrees(q4))]

    def compute_jacobian(self, joint_angles: list) -> np.ndarray:
        q_rad = np.radians(joint_angles)
        l1, l2, l3, l4 = self.link_lengths
        th1, th2, th3, th4 = q_rad[0], q_rad[1], q_rad[2], q_rad[3]
        s1, c1 = np.sin(th1), np.cos(th1)
        s2, c2 = np.sin(th2), np.cos(th2)
        s23, c23 = np.sin(th2 + th3), np.cos(th2 + th3)
        s234, c234 = np.sin(th2 + th3 + th4), np.cos(th2 + th3 + th4)

        r = l2 * c2 + l3 * c23 + l4 * c234
        return np.array([
            [-s1 * r, c1 * (-l2*s2 - l3*s23 - l4*s234), c1 * (-l3*s23 - l4*s234), c1 * (-l4*s234)],
            [ c1 * r, s1 * (-l2*s2 - l3*s23 - l4*s234), s1 * (-l3*s23 - l4*s234), s1 * (-l4*s234)],
            [ 0.0,    l2*c2 + l3*c23 + l4*c234,         l3*c23 + l4*c234,         l4*c234]
        ])