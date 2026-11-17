#!/usr/bin/env python3
"""
STS3215 Serial Bus Servo Hardware Driver & Mock Simulation Mode Module.
Designed for 4-DOF SO-ARM101 Robotic Manipulator.
"""


class STS3215Driver:
    """Feetech STS3215 bus-servo driver with simulation fallback."""

    ADDR_TORQUE_ENABLE = 40       # 1 byte
    ADDR_GOAL_POSITION = 42       # 2 bytes (0-4095)
    ADDR_GOAL_SPEED = 46          # 2 bytes (steps/s)
    ADDR_PRESENT_POSITION = 56    # 2 bytes
    ADDR_PRESENT_SPEED = 58       # 2 bytes
    ADDR_PRESENT_LOAD = 60        # 2 bytes
    ADDR_PRESENT_VOLTAGE = 62     # 1 byte
    ADDR_PRESENT_TEMPERATURE = 63 # 1 byte

    COUNTS_PER_REV = 4096
    CENTER_COUNT = 2048           # 0 deg = servo centre (so -60..+60 deg is valid)
    MAX_SPEED_STEPS = 3400

    def __init__(self, port: str = "/dev/tty0", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.portHandler = None
        self.packetHandler = None
        self.is_connected = False
        self.simulated_angles = {1: 0.0, 2: 30.0, 3: -45.0, 4: 15.0}

    # ------------------------------------------------------------ conversions
    def _deg_to_count(self, angle_deg: float) -> int:
        cnt = int(round(self.CENTER_COUNT + angle_deg / 360.0 * self.COUNTS_PER_REV))
        return max(0, min(self.COUNTS_PER_REV - 1, cnt))

    def _count_to_deg(self, cnt: float) -> float:
        return (float(cnt) - self.CENTER_COUNT) / self.COUNTS_PER_REV * 360.0

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
        """Goal position in degrees, centred on 0 (servo centre = 2048 counts)."""
        self.simulated_angles[servo_id] = angle_deg

        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_POSITION, self._deg_to_count(angle_deg))
            return result == COMM_SUCCESS
        return True

    def set_servo_speed(self, servo_id: int, speed_deg_s: float) -> bool:
        """Servo speed ceiling in deg/s."""
        if self.is_connected and self.portHandler and self.packetHandler:
            from scservo_sdk import COMM_SUCCESS
            steps = int(speed_deg_s / 360.0 * self.COUNTS_PER_REV)
            steps = max(1, min(self.MAX_SPEED_STEPS, steps))
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
                angle_deg = self._count_to_deg(pos_cnt)
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

    def enable_torque(self, servo_id: int, enable: bool) -> None:
        if self.is_connected and self.portHandler and self.packetHandler:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_TORQUE_ENABLE, 1 if enable else 0)
        else:
            print(f"[STS3215Driver Mock] Servo ID {servo_id} torque {'ENABLED' if enable else 'DISABLED'}.")