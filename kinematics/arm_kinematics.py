#!/usr/bin/env python3
import numpy as np


class ArmKinematics:
    """Forward, inverse and VELOCITY kinematics for the SO-ARM101.

    Two angle spaces are used:

      * SERVO angles  q  (deg) - what the driver / controller / GUI sliders use.
                                 q = [0, 0, 0, 0]  <=>  all servos at 1024 steps
                                 <=>  the physical L-shape home pose.
      * MODEL angles  th (rad) - what the planar-chain maths below uses.
                                 th_i = SIGN_i * q_i + OFFSET_i      (map_servo_angles)

    Physical home pose (q = 0):
        q1 : base yaw, 0 deg points along +X.
        q2 : shoulder pitch, 0 deg = upper arm HORIZONTAL (0 deg elevation).
        q3 : elbow pitch, 0 deg = forearm bent 90 deg DOWN from the upper arm
             (upside-down "L").  => model offset  th3 = q3 - 90 deg.
        q4 : wrist pitch, 0 deg = continues straight along the forearm axis.

    Sign convention of the model (right-handed, +Y is the left of the +X axis):
        +q1 : CCW seen from above (+Z)
        +q2 : raises the upper arm      (pitch about the shoulder axis)
        +q3 : opens the elbow, forearm swings UP towards the upper-arm line
        +q4 : continues the same sense as q3
    If a physical joint turns the opposite way, flip that joint in JOINT_SIGN
    (visual / maths side) or in STS3215Driver.servo_directions (motor side) -
    change it in ONE place only, otherwise the two flips cancel each other.

    DH table (a, d in cm in the datasheet -> metres here), joint variable theta_i = th_i:

        i |  a_i (cm) | alpha_i | d_i (cm) | theta_i
        1 |   4.2     |  +90    |   3.0    | th1
        2 |  12.0     |    0    |   0      | th2
        3 |  16.4     |    0    |   0      | th3 (= q3 - 90 deg)
        4 |   0       |    0    |   0      | th4 (fixed tool frame, pitch only)

    q4 is an ORIENTATION-ONLY joint: since a4 = d4 = 0 it never changes the tip
    position, it only sets the tool pitch phi = th2 + th3 + th4.
    """

    # Model angle at servo angle 0 (deg).  Elbow forearm hangs 90 deg below the upper arm.
    JOINT_OFFSET_DEG = (0.0, 90.0, -90.0, 0.0)
    # Direction of the model rotation for a positive servo angle (+1 / -1).
    JOINT_SIGN = (1.0, 1.0, 1.0, 1.0)

    def __init__(self, a1=0.042, d1=0.030, a2=0.120, a3=0.164, a4=0.0,
                 joint_limit_deg: float = 60.0,
                 joint_offset_deg=None, joint_sign=None):
        self.a1, self.d1, self.a2, self.a3, self.a4 = a1, d1, a2, a3, a4

        self.offset_deg = np.array(
            self.JOINT_OFFSET_DEG if joint_offset_deg is None else joint_offset_deg, dtype=float)
        self.sign = np.array(
            self.JOINT_SIGN if joint_sign is None else joint_sign, dtype=float)
        if self.offset_deg.shape != (4,) or self.sign.shape != (4,) \
                or not np.all(np.abs(self.sign) == 1.0):
            raise ValueError("joint_offset_deg needs 4 values, joint_sign needs 4 values of +1/-1")

        # DH rows: [a, alpha, d]   (theta of every row = mapped model angle)
        self.dh_params = [
            [a1, np.pi / 2, d1],
            [a2, 0.0, 0.0],
            [a3, 0.0, 0.0],
            [a4, 0.0, 0.0],           # fixed tool frame (+ q4 rotation)
        ]
        # Joint freedom (SERVO angles): -limit ... +limit for every joint
        self.joint_limit_deg = float(joint_limit_deg)
        self.lower = np.full(4, -self.joint_limit_deg)
        self.upper = np.full(4, self.joint_limit_deg)

    @property
    def link_lengths(self):
        """Backward-compatible [height, upper arm, forearm, wrist] = [d1, a2, a3, a4]."""
        return [self.d1, self.a2, self.a3, self.a4]

    # ------------------------------------------------------- servo <-> model
    def map_servo_angles(self, q_deg) -> np.ndarray:
        """Servo angles (deg) -> model angles (rad):  th = sign * q + offset."""
        q = np.asarray(q_deg, dtype=float)
        return np.radians(self.sign * q + self.offset_deg)

    def unmap_model_angles(self, th_rad) -> np.ndarray:
        """Model angles (rad) -> servo angles (deg).  Inverse of map_servo_angles."""
        th = np.degrees(np.asarray(th_rad, dtype=float))
        return self.sign * (th - self.offset_deg)

    # ------------------------------------------------------------------ limits
    def within_limits(self, q_deg) -> bool:
        q = np.asarray(q_deg, dtype=float)
        return bool(np.all(q >= self.lower - 1e-9) and np.all(q <= self.upper + 1e-9))

    def clamp(self, q_deg) -> np.ndarray:
        return np.clip(np.asarray(q_deg, dtype=float), self.lower, self.upper)

    # ------------------------------------------------------------ DH utilities
    @staticmethod
    def dh_matrix(a, alpha, d, theta) -> np.ndarray:
        """Standard DH transform  Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
        ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st * ca,  st * sa, a * ct],
                         [st,  ct * ca, -ct * sa, a * st],
                         [0.0,      sa,       ca,      d],
                         [0.0,     0.0,      0.0,    1.0]])

    def dh_chain(self, joint_angles_deg) -> np.ndarray:
        """Full 4x4 base->tool transform built straight from the DH table."""
        th = self.map_servo_angles(joint_angles_deg)
        T = np.eye(4)
        for (a, alpha, d), theta in zip(self.dh_params, th):
            T = T @ self.dh_matrix(a, alpha, d, theta)
        return T

    # ----------------------------------------------------------------- position
    def forward_kinematics(self, joint_angles: list) -> list:
        """Returns [x, y, z, phi_deg] (metres, degrees). phi = absolute tool pitch."""
        th1, th2, th3, th4 = self.map_servo_angles(joint_angles)
        phi = th2 + th3 + th4

        r = (self.a1 + self.a2 * np.cos(th2) + self.a3 * np.cos(th2 + th3)
             + self.a4 * np.cos(phi))
        x = r * np.cos(th1)
        y = r * np.sin(th1)
        z = (self.d1 + self.a2 * np.sin(th2) + self.a3 * np.sin(th2 + th3)
             + self.a4 * np.sin(phi))
        return [float(x), float(y), float(z), float(np.degrees(phi))]

    def joint_positions(self, joint_angles: list) -> np.ndarray:
        """5x3 points for drawing: base, top of base column, shoulder, elbow, tip."""
        th1, th2, th3, th4 = self.map_servo_angles(joint_angles)
        c1, s1 = np.cos(th1), np.sin(th1)
        r2 = self.a1
        r3 = r2 + self.a2 * np.cos(th2)
        r4 = r3 + self.a3 * np.cos(th2 + th3) + self.a4 * np.cos(th2 + th3 + th4)
        z3 = self.d1 + self.a2 * np.sin(th2)
        z4 = z3 + self.a3 * np.sin(th2 + th3) + self.a4 * np.sin(th2 + th3 + th4)
        return np.array([[0, 0, 0],
                         [0, 0, self.d1],
                         [r2 * c1, r2 * s1, self.d1],
                         [r3 * c1, r3 * s1, z3],
                         [r4 * c1, r4 * s1, z4]])

    def inverse_kinematics(self, target_pose: list):
        """Elbow-down IK. Takes [x, y, z, phi_deg]; returns SERVO angles [q1..q4] (deg),
        or None if unreachable or outside the +/-60 deg window.

        Solved in MODEL angles (th), then converted back with the joint offsets/signs.
        Elbow-down = th3 < 0, which is the branch the L-shape home pose lives on.
        """
        x, y, z = target_pose[0], target_pose[1], target_pose[2]
        phi = np.radians(target_pose[3] if len(target_pose) > 3 else 0.0)
        a1, d1, a2, a3, a4 = self.a1, self.d1, self.a2, self.a3, self.a4

        th1 = np.arctan2(y, x)
        R = np.sqrt(x ** 2 + y ** 2)
        Rw = R - a1 - a4 * np.cos(phi)          # horizontal distance shoulder -> wrist
        Zw = z - d1 - a4 * np.sin(phi)          # vertical distance shoulder -> wrist

        cos_th3 = (Rw ** 2 + Zw ** 2 - a2 ** 2 - a3 ** 2) / (2.0 * a2 * a3)
        if abs(cos_th3) > 1.0 + 1e-9:
            return None
        cos_th3 = np.clip(cos_th3, -1.0, 1.0)

        th3 = np.arctan2(-np.sqrt(1.0 - cos_th3 ** 2), cos_th3)
        k1 = a2 + a3 * np.cos(th3)
        k2 = a3 * np.sin(th3)
        th2 = np.arctan2(Zw, Rw) - np.arctan2(k2, k1)
        th4 = phi - (th2 + th3)

        q = [float(v) for v in self.unmap_model_angles([th1, th2, th3, th4])]
        return q if self.within_limits(q) else None

    # ----------------------------------------------------------------- velocity
    def compute_jacobian(self, joint_angles: list) -> np.ndarray:
        """3x4 linear-velocity Jacobian  [vx vy vz]^T = J * [dq1..dq4]^T  (rad/s -> m/s).

        Built from the model angles, then each column is multiplied by sign_i
        (d th_i / d q_i = sign_i) so it is the Jacobian w.r.t. the SERVO angles.
        """
        th1, th2, th3, th4 = self.map_servo_angles(joint_angles)
        a1, a2, a3, a4 = self.a1, self.a2, self.a3, self.a4
        s1, c1 = np.sin(th1), np.cos(th1)
        s2, c2 = np.sin(th2), np.cos(th2)
        s23, c23 = np.sin(th2 + th3), np.cos(th2 + th3)
        s234, c234 = np.sin(th2 + th3 + th4), np.cos(th2 + th3 + th4)

        r = a1 + a2 * c2 + a3 * c23 + a4 * c234          # horizontal reach (incl. shoulder offset a1)
        dr2 = -a2 * s2 - a3 * s23 - a4 * s234            # dr/dth2
        dr3 = -a3 * s23 - a4 * s234                      # dr/dth3
        dr4 = -a4 * s234                                 # dr/dth4
        dz2 = a2 * c2 + a3 * c23 + a4 * c234
        dz3 = a3 * c23 + a4 * c234
        dz4 = a4 * c234
        J_model = np.array([
            [-s1 * r, c1 * dr2, c1 * dr3, c1 * dr4],
            [c1 * r, s1 * dr2, s1 * dr3, s1 * dr4],
            [0.0, dz2, dz3, dz4],
        ])
        return J_model * self.sign                       # scale column i by sign_i

    def compute_jacobian_4x4(self, joint_angles: list) -> np.ndarray:
        """Square Jacobian: adds the tool-pitch row (dphi = s2 dq2 + s3 dq3 + s4 dq4)."""
        pitch_row = [0.0, self.sign[1], self.sign[2], self.sign[3]]
        return np.vstack([self.compute_jacobian(joint_angles), pitch_row])

    def tip_velocity(self, joint_angles, joint_vel_deg) -> np.ndarray:
        """Forward velocity kinematics: joint speeds (deg/s) -> tip velocity (m/s)."""
        return self.compute_jacobian(joint_angles) @ np.radians(joint_vel_deg)

    def joint_velocity_from_cartesian(self, joint_angles, v_cart, pitch_rate: float = 0.0,
                                      damping: float = 0.003):
        """Inverse velocity kinematics (resolved-rate) with singularity-robust damped least squares.

        v_cart in m/s, returns (joint speeds deg/s, manipulability).
        """
        J = self.compute_jacobian_4x4(joint_angles)
        xdot = np.array([v_cart[0], v_cart[1], v_cart[2], pitch_rate], dtype=float)
        JJt = J @ J.T
        w = float(np.sqrt(max(np.linalg.det(JJt), 0.0)))   # manipulability
        w0 = 3e-4                                           # below this we are "near singular"
        lam2 = damping ** 2 + (0.01 * (1.0 - w / w0) ** 2 if w < w0 else 0.0)
        qdot = J.T @ np.linalg.solve(JJt + lam2 * np.eye(4), xdot)
        return np.degrees(qdot), w