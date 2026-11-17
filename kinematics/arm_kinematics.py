#!/usr/bin/env python3

import numpy as np


class ArmKinematics:
    """
    4-DOF SO-ARM101

    Joint units:
        degrees

    Cartesian units:
        metres

    End-effector pose:
        [x, y, z, pitch_deg]
    """

    def __init__(
        self,
        link_lengths=None,
        joint_limit_deg=60.0
    ):

        self.link_lengths = (
            link_lengths
            if link_lengths is not None
            else [0.08, 0.12, 0.12, 0.06]
        )

        self.joint_limit_deg = float(joint_limit_deg)

        self.lower = np.full(
            4,
            -self.joint_limit_deg
        )

        self.upper = np.full(
            4,
            self.joint_limit_deg
        )

    # --------------------------------------------------
    # limits
    # --------------------------------------------------

    def within_limits(self, q_deg):

        q = np.asarray(q_deg, dtype=float)

        return bool(
            np.all(q >= self.lower)
            and
            np.all(q <= self.upper)
        )

    def clamp(self, q_deg):

        return np.clip(
            np.asarray(q_deg, dtype=float),
            self.lower,
            self.upper
        )

    # --------------------------------------------------
    # FK
    # --------------------------------------------------

    def forward_kinematics(self, joint_angles):

        q = np.radians(joint_angles)

        l1, l2, l3, l4 = self.link_lengths

        q1, q2, q3, q4 = q

        pitch = q2 + q3 + q4

        r = (
            l2 * np.cos(q2)
            +
            l3 * np.cos(q2 + q3)
            +
            l4 * np.cos(pitch)
        )

        x = r * np.cos(q1)

        y = r * np.sin(q1)

        z = (
            l1
            +
            l2 * np.sin(q2)
            +
            l3 * np.sin(q2 + q3)
            +
            l4 * np.sin(pitch)
        )

        return [
            float(x),
            float(y),
            float(z),
            float(np.degrees(pitch))
        ]

    # --------------------------------------------------
    # IK
    # --------------------------------------------------

    def inverse_kinematics(self, target_pose):

        x, y, z = target_pose[:3]

        pitch_deg = (
            target_pose[3]
            if len(target_pose) > 3
            else 0.0
        )

        l1, l2, l3, l4 = self.link_lengths

        pitch = np.radians(pitch_deg)

        q1 = np.arctan2(y, x)

        r = np.hypot(x, y)

        rw = r - l4 * np.cos(pitch)

        zw = z - l1 - l4 * np.sin(pitch)

        c3 = (
            rw**2
            +
            zw**2
            -
            l2**2
            -
            l3**2
        ) / (2 * l2 * l3)

        if abs(c3) > 1.0:
            return None

        c3 = np.clip(
            c3,
            -1.0,
            1.0
        )

        q3 = np.arctan2(
            -np.sqrt(1.0 - c3**2),
            c3
        )

        k1 = l2 + l3 * np.cos(q3)

        k2 = l3 * np.sin(q3)

        q2 = (
            np.arctan2(zw, rw)
            -
            np.arctan2(k2, k1)
        )

        q4 = pitch - q2 - q3

        q_deg = list(
            np.degrees(
                [q1, q2, q3, q4]
            )
        )

        if not self.within_limits(q_deg):
            return None

        return q_deg

    # --------------------------------------------------
    # Jacobian
    # --------------------------------------------------

    def compute_jacobian(self, joint_angles):

        q = np.radians(joint_angles)

        l1, l2, l3, l4 = self.link_lengths

        q1, q2, q3, q4 = q

        s1 = np.sin(q1)
        c1 = np.cos(q1)

        s2 = np.sin(q2)
        c2 = np.cos(q2)

        s23 = np.sin(q2 + q3)
        c23 = np.cos(q2 + q3)

        s234 = np.sin(q2 + q3 + q4)
        c234 = np.cos(q2 + q3 + q4)

        r = (
            l2 * c2
            +
            l3 * c23
            +
            l4 * c234
        )

        J = np.array([

            [
                -s1 * r,
                c1 * (-l2*s2 - l3*s23 - l4*s234),
                c1 * (-l3*s23 - l4*s234),
                c1 * (-l4*s234)
            ],

            [
                c1 * r,
                s1 * (-l2*s2 - l3*s23 - l4*s234),
                s1 * (-l3*s23 - l4*s234),
                s1 * (-l4*s234)
            ],

            [
                0.0,
                l2*c2 + l3*c23 + l4*c234,
                l3*c23 + l4*c234,
                l4*c234
            ]
        ])

        return J

    def compute_jacobian_4x4(self, joint_angles):

        return np.vstack([

            self.compute_jacobian(joint_angles),

            [0.0, 1.0, 1.0, 1.0]

        ])

    # --------------------------------------------------
    # Velocity FK
    # --------------------------------------------------

    def tip_velocity(
        self,
        joint_angles,
        joint_vel_deg
    ):

        J = self.compute_jacobian(
            joint_angles
        )

        qdot = np.radians(
            joint_vel_deg
        )

        return J @ qdot

    # --------------------------------------------------
    # Velocity IK
    # --------------------------------------------------

    def joint_velocity_from_cartesian(
        self,
        joint_angles,
        velocity_xyz,
        pitch_rate=0.0,
        damping=0.003
    ):

        J = self.compute_jacobian_4x4(
            joint_angles
        )

        xdot = np.array([
            velocity_xyz[0],
            velocity_xyz[1],
            velocity_xyz[2],
            pitch_rate
        ])

        JJt = J @ J.T

        manipulability = float(
            np.sqrt(
                max(
                    np.linalg.det(JJt),
                    0.0
                )
            )
        )

        lam2 = damping ** 2

        qdot = (
            J.T
            @
            np.linalg.solve(
                JJt + lam2 * np.eye(4),
                xdot
            )
        )

        return np.degrees(qdot), manipulability