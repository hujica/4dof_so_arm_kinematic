#!/usr/bin/env python3
"""
STS3215 Serial Bus Servo Hardware Driver & Mock Simulation Mode Module.
Designed for 4-DOF SO-ARM101 Robotic Manipulator.
"""


class STS3215Driver:
    """Feetech STS3215 bus-servo driver with per-servo direction inversion and simulation fallback."""

    # ---- Control-table addresses
    ADDR_TORQUE_ENABLE = 40        # 1 byte
    ADDR_GOAL_POSITION = 42        # 2 bytes
    ADDR_GOAL_SPEED = 46           # 2 bytes (steps/s)
    ADDR_PRESENT_POSITION = 56     # 2 bytes
    ADDR_PRESENT_SPEED = 58        # 2 bytes
    ADDR_PRESENT_LOAD = 60         # 2 bytes
    ADDR_PRESENT_VOLTAGE = 62      # 1 byte
    ADDR_PRESENT_TEMPERATURE = 63  # 1 byte

    SERVO_IDS = (1, 2, 3, 4)

    # ---- POSITION scaling: active range 180 deg = 2047 steps, 0 deg = 1024 steps (L-shape home)
    ACTIVE_RANGE_DEG = 180.0
    ACTIVE_RANGE_STEPS = 2047
    CENTER_COUNT = 1024
    STEPS_PER_DEG = ACTIVE_RANGE_STEPS / ACTIVE_RANGE_DEG      # ~11.372 steps/deg

    # ---- Safe joint window: -60 .. +60 deg  ->  342 .. 1706 steps
    JOINT_LIMIT_DEG = 60.0
    MIN_COUNT = int(round(CENTER_COUNT - JOINT_LIMIT_DEG * STEPS_PER_DEG))   # 342
    MAX_COUNT = int(round(CENTER_COUNT + JOINT_LIMIT_DEG * STEPS_PER_DEG))   # 1706

    # ---- SPEED scaling: a full turn is 4096 steps (independent of the position scaling above)
    COUNTS_PER_REV = 4096
    MAX_SPEED_STEPS = 3400
    STARTUP_SPEED_DEG_S = 30.0     # slow, safe speed applied right after connecting

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.portHandler = None
        self.packetHandler = None
        self.is_connected = False
        self.simulated_angles = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

        # ------------------------------------------------------------------------------
        # Per-servo direction.  Looking at the horn/shaft of an STS3215:
        #    1  = standard : +angle -> steps increase (1024 -> 2047) -> Counter-Clockwise (CCW)
        #   -1  = inverted : +angle -> steps decrease (1024 -> 0)    -> Clockwise (CW)
        # If a joint moves the wrong way (mirrored servo mounting), change ONLY that joint's
        # value from 1 to -1, e.g.  {1: 1, 2: -1, 3: 1, 4: 1}  flips the shoulder (ID 2).
        # ------------------------------------------------------------------------------
        self.servo_directions = {1: -1, 2: -1, 3: -1, 4: 1}
        self._clamp_warned = set()

    # ------------------------------------------------------------ conversions
    def _direction(self, servo_id: int) -> int:
        d = self.servo_directions.get(servo_id, 1)
        if d not in (1, -1):
            raise ValueError(f"servo_directions[{servo_id}] must be 1 or -1, got {d!r}")
        return d

    @classmethod
    def clamp_angle(cls, angle_deg: float) -> float:
        return max(-cls.JOINT_LIMIT_DEG, min(cls.JOINT_LIMIT_DEG, float(angle_deg)))

    def _deg_to_count(self, servo_id: int, angle_deg: float) -> int:
        """Joint angle (deg) -> servo steps, clamped to +/-60 deg (342..1706)."""
        angle = self.clamp_angle(angle_deg)
        cnt = int(round(self.CENTER_COUNT + self._direction(servo_id) * angle * self.STEPS_PER_DEG))
        return max(self.MIN_COUNT, min(self.MAX_COUNT, cnt))

    def _count_to_deg(self, servo_id: int, cnt: float) -> float:
        """Servo steps -> joint angle (deg). Not clamped, so out-of-window positions stay visible."""
        return self._direction(servo_id) * (float(cnt) - self.CENTER_COUNT) / self.STEPS_PER_DEG

    # ------------------------------------------------------------- connection
    def connect(self) -> bool:
        try:
            from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

            self.portHandler = PortHandler(self.port)
            self.packetHandler = PacketHandler(0)

            if self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate):
                self.serial_conn = getattr(self.portHandler, 'ser', None)
                _, result, _ = self.packetHandler.ping(self.portHandler, 1)
                if result == COMM_SUCCESS:
                    self.is_connected = True
                    print(f"[STS3215Driver] PHYSICAL HARDWARE DETECTED on {self.port} @ {self.baudrate} bps.")
                    # Safe slow start: limit speed BEFORE any torque / motion command
                    for sid in self.SERVO_IDS:
                        self.set_servo_speed(sid, self.STARTUP_SPEED_DEG_S)
                    print(f"[STS3215Driver] Startup speed limited to {self.STARTUP_SPEED_DEG_S:.0f} deg/s.")
                    return True
        except Exception as e:
            print(f"[STS3215Driver] Connect Error: {e}")

        self.is_connected = False
        print(f"[STS3215Driver] No physical servos detected on {self.port}.")
        print("[STS3215Driver] Running smoothly in SIMULATION (MOCK) MODE.")
        return False

    def disconnect(self) -> None:
        if self.is_connected and self.portHandler:
            self.portHandler.closePort()
            self.is_connected = False
            print("[STS3215Driver] Serial connection closed.")

    # ---------------------------------------------------------------- motion
    def set_servo_angle(self, servo_id: int, angle_deg: float) -> bool:
        """Goal position in degrees (0 deg = 1024 steps), clamped to +/-60 deg, direction-corrected."""
        clamped = self.clamp_angle(angle_deg)
        if clamped != float(angle_deg) and servo_id not in self._clamp_warned:
            self._clamp_warned.add(servo_id)
            print(f"[STS3215Driver] WARNING: servo {servo_id} target {angle_deg:.1f} deg "
                  f"clamped to {clamped:.1f} deg (limit +/-{self.JOINT_LIMIT_DEG:.0f}).")
        self.simulated_angles[servo_id] = clamped          # mock state always follows

        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_POSITION, self._deg_to_count(servo_id, clamped))
            return result == COMM_SUCCESS
        return True

    def set_servo_speed(self, servo_id: int, speed_deg_s: float) -> bool:
        """Servo speed ceiling in deg/s (full turn = 4096 steps)."""
        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            steps = int(speed_deg_s / 360.0 * self.COUNTS_PER_REV)
            steps = max(1, min(self.MAX_SPEED_STEPS, steps))      # never 0 (0 = unlimited on STS)
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_SPEED, steps)
            return result == COMM_SUCCESS
        return True

    def get_servo_angle(self, servo_id: int) -> float:
        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            pos_cnt, result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS:
                angle_deg = self._count_to_deg(servo_id, pos_cnt)
                self.simulated_angles[servo_id] = angle_deg
                return angle_deg
        return self.simulated_angles.get(servo_id, 0.0)

    def read_torque(self, servo_id: int) -> float:
        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            load, result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_PRESENT_LOAD)
            if result == COMM_SUCCESS:
                return float(load)
        return 0.0

    def _hold_position(self, servo_id: int) -> None:
        """Copy present position into the goal register so enabling torque never jumps to an old goal."""
        from scservo_sdk import COMM_SUCCESS
        pos_cnt, result, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_PRESENT_POSITION)
        if result == COMM_SUCCESS:
            self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_POSITION, int(pos_cnt))

    def enable_torque(self, servo_id: int, enable: bool) -> None:
        if self.is_connected and self.portHandler and self.packetHandler:
            if enable:
                self._hold_position(servo_id)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_TORQUE_ENABLE, 1 if enable else 0)
        else:
            print(f"[STS3215Driver Mock] Servo ID {servo_id} torque {'ENABLED' if enable else 'DISABLED'}.")