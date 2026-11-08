#!/usr/bin/env python3
"""
STS3215 Serial Bus Servo Hardware Driver & Mock Simulation Mode Module.
Designed for 4-DOF SO-ARM101 Robotic Manipulator.
"""

import time

class STS3215Driver:
    """
    Feetech STS3215 Bus Servo Driver.
    Handles low-level TTL Half-Duplex serial packet communication, angle conversions,
    and fallback mock simulation when hardware is disconnected.
    """

    # STS3215 EEPROM / Control Table Register Addresses
    ADDR_TORQUE_ENABLE = 40        # 1 Byte: 0=Disable, 1=Enable
    ADDR_GOAL_POSITION = 42        # 2 Bytes: Goal Position (0 - 4095)
    ADDR_GOAL_SPEED = 46           # 2 Bytes: Moving Speed
    ADDR_PRESENT_POSITION = 56     # 2 Bytes: Present Position (Read Only)
    ADDR_PRESENT_SPEED = 58        # 2 Bytes: Present Speed (Read Only)
    ADDR_PRESENT_LOAD = 60         # 2 Bytes: Present Load / Torque Output (Read Only)
    ADDR_PRESENT_VOLTAGE = 62      # 1 Byte: Present Voltage (Read Only)
    ADDR_PRESENT_TEMPERATURE = 63  # 1 Byte: Present Temperature (Read Only)

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 1000000):
        """
        Initialize Driver Parameters.
        :param port: Serial port path (e.g., /dev/ttyUSB0 or /dev/ttyACM0)
        :param baudrate: Communication speed (Default for STS3215 is 1,000,000 bps)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.portHandler = None
        self.packetHandler = None
        self.is_connected = False
        
        # Mock Storage for Hardware Disconnected / Simulation Mode
        # Initial angles matching default initial arm pose (Joints 1 to 4)
        self.simulated_angles = {1: 0.0, 2: 30.0, 3: -45.0, 4: 15.0}

    def connect(self) -> bool:
        """
        Initialize serial connection and attempt hardware ping to STS3215 servos.
        Fallback to Simulation Mode if no physical board is found.
        """
        try:
            # Try importing SDK dynamically (Support feetech_servo_sdk)
            from feetech_servo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
            
            self.portHandler = PortHandler(self.port)
            self.packetHandler = PacketHandler(0)  # STS Protocol ID: 0

            if self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate):
                self.serial_conn = getattr(self.portHandler, 'ser', None)
                
                # Test Ping to Joint 1 Servo
                _, result, _ = self.packetHandler.ping(self.portHandler, 1)
                if result == COMM_SUCCESS:
                    self.is_connected = True
                    print(f"[STS3215Driver] PHYSICAL HARDWARE DETECTED on {self.port} @ {self.baudrate} bps.")
                    return True

        except Exception as e:
            # Pass quietly to fallback
            pass

        # Fallback to Simulation Mode
        self.is_connected = False
        print(f"[STS3215Driver] No physical servos detected on {self.port}.")
        print("[STS3215Driver] Running smoothly in SIMULATION (MOCK) MODE.")
        return False

    def disconnect(self) -> None:
        """Close physical serial connection safely."""
        if self.is_connected and self.portHandler:
            self.portHandler.closePort()
            self.is_connected = False
            print("[STS3215Driver] Serial connection closed.")

    def set_servo_angle(self, servo_id: int, angle_deg: float) -> bool:
        """
        Set target goal position of a specific servo in degrees.
        Maps angle_deg (0° - 360°) to servo step resolution (0 - 4095 cnt).
        """
        self.simulated_angles[servo_id] = angle_deg  # Update mock state always

        if self.is_connected and self.portHandler and self.packetHandler:
            from feetech_servo_sdk import COMM_SUCCESS
            
            # Angle mapping: 0 to 360 deg -> 0 to 4095 count
            position_cnt = int((angle_deg / 360.0) * 4095.0)
            position_cnt = max(0, min(4095, position_cnt))  # Clamp within safe limits

            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_POSITION, position_cnt
            )
            return result == COMM_SUCCESS

        return True  # Always return True in simulation mode

    def get_servo_angle(self, servo_id: int) -> float:
        """Read current present angle position of a specific servo in degrees."""
        if self.is_connected and self.portHandler and self.packetHandler:
            from feetech_servo_sdk import COMM_SUCCESS
            
            pos_cnt, result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                angle_deg = (float(pos_cnt) / 4095.0) * 360.0
                self.simulated_angles[servo_id] = angle_deg
                return angle_deg

        # Return mock internal position if in simulation mode
        return self.simulated_angles.get(servo_id, 0.0)

    def read_torque(self, servo_id: int) -> float:
        """Read present output load/torque value of a specific servo."""
        if self.is_connected and self.portHandler and self.packetHandler:
            from feetech_servo_sdk import COMM_SUCCESS
            
            load, result, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_PRESENT_LOAD
            )
            if result == COMM_SUCCESS:
                return float(load)
                
        return 0.0  # Return zero load in simulation mode

    def enable_torque(self, servo_id: int, enable: bool) -> None:
        """Enable or disable motor power/torque for a target servo."""
        if self.is_connected and self.portHandler and self.packetHandler:
            val = 1 if enable else 0
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_TORQUE_ENABLE, val
            )
        else:
            state_str = "ENABLED" if enable else "DISABLED"
            print(f"[STS3215Driver Mock] Servo ID {servo_id} torque {state_str}.")