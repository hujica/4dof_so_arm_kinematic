#!/usr/bin/env python3
import numpy as np
from hardware.sts3215_driver import STS3215Driver
from kinematics.arm_kinematics import ArmKinematics


class ArmController:
    """Velocity-level controller: Cartesian jogging (resolved-rate) and joint moves,
    both with a user-defined speed and a +/-60 deg joint window."""

    HOME_POSE = [0.0, 0.0, 0.0, 0.0]     # physical L-shape: all servos at 1024 steps
    JOINT_LIMIT_DEG = 60.0
    COMM_FAIL_LIMIT = 5            # consecutive failed servo writes before auto E-stop

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 1000000):
        self.hardware = STS3215Driver(port, baudrate)
        self.kinematics = ArmKinematics(joint_limit_deg=self.JOINT_LIMIT_DEG)

        self.current_joint_angles = list(self.HOME_POSE)
        self.joint_targets = list(self.HOME_POSE)
        self.current_joint_velocities = [0.0] * 4      # deg/s
        self.current_tip_velocity = [0.0] * 3          # m/s
        self.current_end_effector_pose = self.kinematics.forward_kinematics(self.current_joint_angles)

        self.cart_cmd = np.zeros(3)                    # commanded tip velocity (m/s)
        self.max_linear_speed = 0.04                   # m/s
        self.max_joint_speed = 30.0                    # deg/s
        self.estopped = False
        self.estop_reason = ""
        self.mode = "IDLE"
        self.status = "Ready"
        self.status_level = "ok"                       # ok | warn | error
        self.sim_mode = True                           # True until real servos are detected
        self.comm_fail = 0
        self._servo_speed_set = None

    # ----------------------------------------------------------------- setup
    def initialize_arm(self) -> bool:
        """Connect to the servos. Returns True if physical hardware was found."""
        connected = self.hardware.connect()
        self.sim_mode = not connected

        if connected:
            q = [self.hardware.get_servo_angle(i) for i in range(1, 5)]
            self.current_joint_angles = q
            self.joint_targets = list(q)
            self.current_end_effector_pose = self.kinematics.forward_kinematics(q)
            if not self.kinematics.within_limits(q):
                print("[WARN] Arm is outside the +/-60 deg window at start-up.")
        else:
            print("[WARN] Arm not connected - running in SIMULATION mode.")

        for i in range(1, 5):
            self.hardware.enable_torque(i, True)
        self._apply_servo_speed()
        return connected

    def shutdown(self) -> None:
        self.cart_cmd[:] = 0.0
        self.hardware.disconnect()

    # -------------------------------------------------------------- commands
    def set_speed_limits(self, linear_m_s: float, joint_deg_s: float) -> None:
        self.max_linear_speed = max(1e-3, float(linear_m_s))
        joint_deg_s = max(1.0, float(joint_deg_s))
        if joint_deg_s != self.max_joint_speed:
            self.max_joint_speed = joint_deg_s
            self._apply_servo_speed()

    def _apply_servo_speed(self) -> None:
        ceiling = self.max_joint_speed * 1.5   # servo may run a bit faster than our commanded ramp
        if ceiling != self._servo_speed_set:
            for i in range(1, 5):
                self.hardware.set_servo_speed(i, ceiling)
            self._servo_speed_set = ceiling

    def set_cartesian_velocity(self, vx: float, vy: float, vz: float) -> None:
        self.cart_cmd = np.array([vx, vy, vz], dtype=float)

    def set_joint_target(self, idx: int, angle_deg: float) -> None:
        lo, hi = self.kinematics.lower[idx], self.kinematics.upper[idx]
        self.joint_targets[idx] = float(np.clip(angle_deg, lo, hi))

    def move_cartesian(self, dx: float, dy: float, dz: float) -> bool:
        """Step to a nearby Cartesian pose (IK) at the current joint speed."""
        p = self.current_end_effector_pose
        q = self.kinematics.inverse_kinematics([p[0] + dx, p[1] + dy, p[2] + dz, p[3]])
        if q is None:
            return False
        self.joint_targets = q
        return True

    def home_arm(self) -> None:
        self.cart_cmd[:] = 0.0
        self.joint_targets = list(self.HOME_POSE)

    def emergency_stop(self, reason: str = "manual") -> None:
        print(f"[ArmController] EMERGENCY STOP ({reason})")
        self.estopped = True
        self.estop_reason = reason
        self.cart_cmd[:] = 0.0
        self.joint_targets = list(self.current_joint_angles)
        self.current_joint_velocities = [0.0] * 4
        self.current_tip_velocity = [0.0] * 3
        for i in range(1, 5):
            self.hardware.enable_torque(i, False)

    def reset_estop(self) -> None:
        for i in range(1, 5):
            self.hardware.enable_torque(i, True)
        self.joint_targets = list(self.current_joint_angles)
        self.estopped = False
        self.estop_reason = ""
        self.comm_fail = 0
        self.status, self.status_level = "Ready", "ok"

    # ------------------------------------------------------------ main update
    def update(self, dt: float) -> None:
        """Advance the arm by dt seconds (call at a fixed rate, e.g. 50 Hz)."""
        if self.estopped:
            self.mode = "E-STOP"
            self.status = f"EMERGENCY STOP ({self.estop_reason})"
            self.status_level = "error"
            return

        kin = self.kinematics
        q = np.array(self.current_joint_angles, dtype=float)
        vmax = self.max_joint_speed
        blocked = False

        if np.any(self.cart_cmd):
            # Resolved-rate: tip velocity -> joint velocity (Jacobian inverse, DLS)
            self.mode = "CARTESIAN VELOCITY"
            qdot, _ = kin.joint_velocity_from_cartesian(q, self.cart_cmd)
            peak = float(np.max(np.abs(qdot)))
            if peak > vmax:
                qdot *= vmax / peak            # keep direction, respect joint speed limit
            step = qdot * dt
            new_q = q + step
            if np.any(new_q < kin.lower) or np.any(new_q > kin.upper):
                step = np.zeros(4)
                blocked = True
            self.joint_targets = (q + step).tolist()
        else:
            err = kin.clamp(self.joint_targets) - q
            m = float(np.max(np.abs(err)))
            if m < 1e-3:
                step = np.zeros(4)
                self.mode = "IDLE"
            else:
                self.mode = "JOINT MOVE"
                max_step = vmax * dt
                step = err if m <= max_step else err * (max_step / m)

        self._apply(q + step, step / dt)

        # _apply may have triggered an automatic E-stop (servo comm lost)
        if self.estopped:
            return

        if blocked:
            self.status, self.status_level = "Joint limit (+/-60 deg) - motion blocked", "warn"
        elif self.mode == "IDLE":
            self.status, self.status_level = "Ready", "ok"
        else:
            self.status, self.status_level = "Moving", "ok"

        # Persistent warnings (only when nothing more urgent is showing)
        if self.status_level == "ok":
            if self.sim_mode:
                self.status, self.status_level = "SIMULATION - arm not connected", "warn"
            elif not kin.within_limits(self.current_joint_angles):
                self.status, self.status_level = "Arm outside +/-60 deg window", "warn"

    def _apply(self, new_q: np.ndarray, qdot: np.ndarray) -> None:
        moved = bool(np.any(np.abs(new_q - np.array(self.current_joint_angles)) > 1e-9))
        self.current_joint_angles = new_q.tolist()
        self.current_joint_velocities = qdot.tolist()
        self.current_end_effector_pose = self.kinematics.forward_kinematics(self.current_joint_angles)
        self.current_tip_velocity = self.kinematics.tip_velocity(new_q, qdot).tolist()
        if moved:
            # list (not generator) so every servo gets its command even if one fails
            results = [self.hardware.set_servo_angle(i + 1, a)
                       for i, a in enumerate(self.current_joint_angles)]
            if not self.sim_mode:
                self.comm_fail = 0 if all(results) else self.comm_fail + 1
                if self.comm_fail >= self.COMM_FAIL_LIMIT:
                    self.emergency_stop("servo communication lost")