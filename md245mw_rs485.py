"""
MD245MW-RS485 Servo Controller

Python class for controlling Hitec MD245MW servo via RS485 protocol.
Based on Hitec CAN Servo Control Protocol Manual (also applicable to RS485 variants).

Protocol Format (Normal Packet):
- Write: Header(0x96) + ID + Address + REG_Length(0x02) + Data_Low + Data_High + CheckSum
- Read Request: Header(0x96) + ID + Address + REG_Length(0x00) + CheckSum
- Read Response: Header(0x69) + ID + Address + REG_Length(0x02) + Data_Low + Data_High + CheckSum
- CheckSum = (ID + Address + REG_Length + Data_Low + Data_High) & 0xFF
"""

import serial
import time
import struct
from typing import Optional, Tuple


class MD245MW_RS485:
    """Controller class for MD245MW RS485 servo."""

    # Protocol constants
    HEADER_WRITE = 0x96
    HEADER_READ = 0x69

    # Register addresses
    REG_POSITION = 0x0C          # Current position (read-only)
    REG_VELOCITY = 0x0E          # Current velocity (read-only)
    REG_TORQUE = 0x10            # Current torque/PWM duty (read-only)
    REG_VOLTAGE = 0x12           # Input voltage (read-only)
    REG_MCU_TEMPER = 0x14        # MCU temperature (read-only)
    REG_TURN_COUNT = 0x18        # Turn count (R/W)
    REG_POSITION_NEW = 0x1E      # Target position (R/W)
    REG_TURN_NEW = 0x24          # Target turn count (R/W)
    REG_ID = 0x32                # Servo ID (R/W)
    REG_RUN_MODE = 0x44          # Run mode (R/W): 0=Multi-Turn, 1=Servo
    REG_EMERGENCY_STOP = 0x48    # Emergency stop status (read-only)
    REG_VELOCITY_MAX = 0x54      # Max velocity setting (R/W)
    REG_TORQUE_MAX = 0x56        # Max torque setting (R/W)
    REG_SPEED_UP = 0xDC          # Acceleration time in ms (R/W)
    REG_SPEED_DN = 0xDE          # Deceleration time in ms (R/W)
    REG_POSITION_MAX_LIMIT = 0xB0  # Max position limit (R/W)
    REG_POSITION_MIN_LIMIT = 0xB2  # Min position limit (R/W)
    REG_CONFIG_SAVE = 0x70       # Save configuration (W)
    REG_FACTORY_DEFAULT = 0x6E   # Reset to defaults (W)
    REG_VERSION = 0xFC           # Firmware version (read-only)

    # Position constants
    POSITION_PER_90_DEG = 4096   # 4096 units = 90 degrees
    POSITION_PER_DEGREE = POSITION_PER_90_DEG / 90.0  # ~45.51 units per degree
    MAX_POSITION = 16383         # Maximum position value (360 degrees)

    # Run modes
    MODE_MULTI_TURN = 0
    MODE_SERVO = 1
    MODE_CR = 2        # Continuous rotation (FW >= 2.0)
    MODE_SPEED = 3     # Speed mode (FW >= 2.0)

    def __init__(self, port: str, servo_id: int = 1, baudrate: int = 115200,
                 timeout: float = 1.0):
        """
        Initialize the servo controller.

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0' or 'COM3')
            servo_id: Servo ID (1-254, 0 is broadcast)
            baudrate: Communication baud rate (default 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.servo_id = servo_id
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None

    def connect(self) -> bool:
        """
        Open the serial connection.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            time.sleep(0.1)  # Allow port to initialize

            # Warm up the servo - some servos need to see traffic before responding
            # Send dummy queries to wake up the bus
            self._warmup_bus()

            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def _warmup_bus(self):
        """
        Send warmup queries to wake up the RS485 bus.

        Some servos need to see traffic on the bus before they start responding.
        This sends a few dummy read requests to different IDs to initialize communication.
        """
        if not self.serial or not self.serial.is_open:
            return

        # Send queries to common IDs to wake up the bus
        warmup_ids = [1, 0, self.servo_id] if self.servo_id not in [0, 1] else [1, 0]

        for sid in warmup_ids:
            checksum = self._calculate_checksum(sid, self.REG_POSITION, 0x00)
            packet = bytes([
                self.HEADER_WRITE,
                sid,
                self.REG_POSITION,
                0x00,
                checksum
            ])
            self.serial.reset_input_buffer()
            self.serial.write(packet)
            self.serial.flush()
            time.sleep(0.05)  # Wait for response
            # Read and discard any response/echo
            time.sleep(0.05)
            if self.serial.in_waiting > 0:
                self.serial.read(self.serial.in_waiting)

        # Final delay to ensure bus is ready
        time.sleep(0.1)

    def disconnect(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False

    def _calculate_checksum(self, servo_id: int, address: int,
                            reg_length: int, data_low: int = 0,
                            data_high: int = 0) -> int:
        """
        Calculate packet checksum.

        Args:
            servo_id: Servo ID
            address: Register address
            reg_length: Register length (0 for read, 2 for write)
            data_low: Low byte of data
            data_high: High byte of data

        Returns:
            Checksum byte
        """
        if reg_length == 0:
            return (servo_id + address + reg_length) & 0xFF
        return (servo_id + address + reg_length + data_low + data_high) & 0xFF

    def _write_register(self, address: int, value: int,
                        servo_id: Optional[int] = None) -> bool:
        """
        Write a value to a register.

        Args:
            address: Register address
            value: 16-bit value to write
            servo_id: Override default servo ID (optional)

        Returns:
            True if write successful
        """
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port not connected")

        sid = servo_id if servo_id is not None else self.servo_id

        # Convert value to little-endian bytes
        data_low = value & 0xFF
        data_high = (value >> 8) & 0xFF

        # Calculate checksum
        checksum = self._calculate_checksum(sid, address, 0x02, data_low, data_high)

        # Build packet
        packet = bytes([
            self.HEADER_WRITE,
            sid,
            address,
            0x02,  # REG_Length for write
            data_low,
            data_high,
            checksum
        ])

        # Clear input buffer and send
        self.serial.reset_input_buffer()
        self.serial.write(packet)
        self.serial.flush()

        return True

    def _read_register(self, address: int,
                       servo_id: Optional[int] = None) -> Optional[int]:
        """
        Read a value from a register.

        Args:
            address: Register address
            servo_id: Override default servo ID (optional)

        Returns:
            16-bit register value or None if read failed
        """
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port not connected")

        sid = servo_id if servo_id is not None else self.servo_id

        # Calculate checksum for read request
        checksum = self._calculate_checksum(sid, address, 0x00)

        # Build read request packet
        packet = bytes([
            self.HEADER_WRITE,
            sid,
            address,
            0x00,  # REG_Length = 0 for read request
            checksum
        ])

        # Clear buffers and send request
        self.serial.reset_input_buffer()
        self.serial.write(packet)
        self.serial.flush()

        # Wait for response
        # Many RS485 adapters echo TX data, so we need to read: echo (5 bytes) + response (7 bytes)
        time.sleep(0.02)  # Initial delay

        # Read in a loop until we have enough data or timeout
        raw_response = b''
        for _ in range(10):  # Max 10 attempts
            available = self.serial.in_waiting
            if available > 0:
                raw_response += self.serial.read(available)
            if len(raw_response) >= 12:  # echo (5) + response (7)
                break
            time.sleep(0.01)  # Small delay between checks

        # Handle TX echo: if response starts with our packet, skip it
        if len(raw_response) >= len(packet) and raw_response[:len(packet)] == packet:
            response = raw_response[len(packet):]
        else:
            response = raw_response

        if len(response) < 7:
            return None

        # Take first 7 bytes of response
        response = response[:7]

        # Validate response
        if response[0] != self.HEADER_READ:
            return None
        # Note: Some servos respond with ID=0 regardless of query ID
        if response[2] != address:
            return None
        if response[3] != 0x02:
            return None

        # Verify checksum
        expected_checksum = self._calculate_checksum(
            response[1], response[2], response[3], response[4], response[5]
        )
        if response[6] != expected_checksum:
            return None

        # Extract value (little-endian)
        value = response[4] | (response[5] << 8)
        return value

    # ========== Cover Control ==========

    # Default positions for cover open/close
    COVER_OPEN_POSITION = 5      # Command angle for open position (~114° actual)
    COVER_CLOSE_POSITION = 78.5  # Command angle for close position (~194° actual)
    COVER_OPEN_SPEED = 300       # Speed for opening
    COVER_CLOSE_SPEED = 5        # Slower speed for closing

    def open_cover(self, speed: int = None) -> bool:
        """
        Open the cover by moving servo to open position.

        Args:
            speed: Optional speed override (default: COVER_OPEN_SPEED)

        Returns:
            True if command sent successfully
        """
        self.set_speed(speed if speed is not None else self.COVER_OPEN_SPEED)
        result = self.set_position(self.COVER_OPEN_POSITION)
        time.sleep(2)
        return result

    def close_cover(self, speed: int = None) -> bool:
        """
        Close the cover by moving servo to close position.

        Args:
            speed: Optional speed override (default: COVER_CLOSE_SPEED)

        Returns:
            True if command sent successfully
        """
        self.set_speed(speed if speed is not None else self.COVER_CLOSE_SPEED)
        result = self.set_position(self.COVER_CLOSE_POSITION)
        time.sleep(2)
        return result

    # ========== Position Control ==========

    def set_position(self, angle_degrees: float) -> bool:
        """
        Move servo to specified angle.

        Args:
            angle_degrees: Target angle in degrees (0-360 in servo mode)

        Returns:
            True if command sent successfully
        """
        # Convert degrees to position units
        position = int(angle_degrees * self.POSITION_PER_DEGREE)

        # Clamp to valid range
        position = max(0, min(position, self.MAX_POSITION))

        return self._write_register(self.REG_POSITION_NEW, position)

    def set_position_raw(self, position: int) -> bool:
        """
        Set servo position using raw position value.

        Args:
            position: Raw position value (0-16383, where 4096 = 90°)

        Returns:
            True if command sent successfully
        """
        position = max(0, min(position, self.MAX_POSITION))
        return self._write_register(self.REG_POSITION_NEW, position)

    def get_position(self) -> Optional[float]:
        """
        Read current servo position in degrees.

        Returns:
            Current angle in degrees or None if read failed
        """
        position = self._read_register(self.REG_POSITION)
        if position is not None:
            return position / self.POSITION_PER_DEGREE
        return None

    def get_position_raw(self) -> Optional[int]:
        """
        Read current servo position as raw value.

        Returns:
            Raw position value (0-16383) or None if read failed
        """
        return self._read_register(self.REG_POSITION)

    # ========== Speed Control ==========

    def set_speed(self, speed: int) -> bool:
        """
        Set maximum servo speed.

        The speed value is in pos/100msec units.
        Higher values = faster movement.

        Args:
            speed: Speed value (0-4095)

        Returns:
            True if command sent successfully
        """
        speed = max(0, min(speed, 4095))
        return self._write_register(self.REG_VELOCITY_MAX, speed)

    def get_speed(self) -> Optional[int]:
        """
        Get current max speed setting.

        Returns:
            Speed value (0-4095) or None if read failed
        """
        return self._read_register(self.REG_VELOCITY_MAX)

    def set_acceleration(self, accel_time_ms: int) -> bool:
        """
        Set acceleration time (ramp up time from 0 to max speed).

        Args:
            accel_time_ms: Acceleration time in milliseconds

        Returns:
            True if command sent successfully
        """
        return self._write_register(self.REG_SPEED_UP, accel_time_ms)

    def set_deceleration(self, decel_time_ms: int) -> bool:
        """
        Set deceleration time (ramp down time from max speed to 0).

        Args:
            decel_time_ms: Deceleration time in milliseconds

        Returns:
            True if command sent successfully
        """
        return self._write_register(self.REG_SPEED_DN, decel_time_ms)

    def get_velocity(self) -> Optional[int]:
        """
        Read current velocity.

        Returns:
            Current velocity in pos/100msec or None if read failed
        """
        return self._read_register(self.REG_VELOCITY)

    # ========== Torque Control ==========

    def set_torque_max(self, torque: int) -> bool:
        """
        Set maximum torque (PWM duty).

        Args:
            torque: Torque value (0-4095, where 4095 = 100%)

        Returns:
            True if command sent successfully
        """
        torque = max(0, min(torque, 4095))
        return self._write_register(self.REG_TORQUE_MAX, torque)

    def get_torque(self) -> Optional[int]:
        """
        Read current torque (PWM duty).

        Returns:
            Current torque value (0-4095) or None if read failed
        """
        return self._read_register(self.REG_TORQUE)

    # ========== Status Readouts ==========

    def get_voltage(self) -> Optional[float]:
        """
        Read input voltage.

        Returns:
            Voltage in volts or None if read failed
        """
        value = self._read_register(self.REG_VOLTAGE)
        if value is not None:
            return value / 100.0  # Convert from 0.01V units
        return None

    def get_temperature(self) -> Optional[int]:
        """
        Read MCU temperature.

        Returns:
            Temperature in Celsius or None if read failed
        """
        return self._read_register(self.REG_MCU_TEMPER)

    def get_turn_count(self) -> Optional[int]:
        """
        Read accumulated turn count (for multi-turn mode).

        Returns:
            Turn count or None if read failed
        """
        value = self._read_register(self.REG_TURN_COUNT)
        if value is not None:
            # Convert to signed 16-bit
            if value > 32767:
                value -= 65536
        return value

    def get_firmware_version(self) -> Optional[int]:
        """
        Read firmware version.

        Returns:
            Firmware version or None if read failed
        """
        return self._read_register(self.REG_VERSION)

    # ========== Mode Control ==========

    def set_run_mode(self, mode: int) -> bool:
        """
        Set servo run mode.

        Args:
            mode: Run mode (0=Multi-Turn, 1=Servo, 2=CR, 3=Speed)

        Returns:
            True if command sent successfully

        Note:
            Requires save and reset to take effect.
        """
        return self._write_register(self.REG_RUN_MODE, mode)

    def get_run_mode(self) -> Optional[int]:
        """
        Get current run mode.

        Returns:
            Run mode (0=Multi-Turn, 1=Servo, 2=CR, 3=Speed) or None
        """
        return self._read_register(self.REG_RUN_MODE)

    # ========== Position Limits ==========

    def set_position_limits(self, min_degrees: float, max_degrees: float) -> bool:
        """
        Set position limits for servo mode.

        Args:
            min_degrees: Minimum angle in degrees
            max_degrees: Maximum angle in degrees

        Returns:
            True if both limits set successfully
        """
        min_pos = int(min_degrees * self.POSITION_PER_DEGREE)
        max_pos = int(max_degrees * self.POSITION_PER_DEGREE)

        min_pos = max(0, min(min_pos, self.MAX_POSITION))
        max_pos = max(0, min(max_pos, self.MAX_POSITION))

        result1 = self._write_register(self.REG_POSITION_MIN_LIMIT, min_pos)
        result2 = self._write_register(self.REG_POSITION_MAX_LIMIT, max_pos)

        return result1 and result2

    # ========== Configuration ==========

    def save_config(self) -> bool:
        """
        Save current configuration to non-volatile memory.

        Returns:
            True if command sent successfully

        Note:
            Wait 1 second after save, then reset servo.
        """
        return self._write_register(self.REG_CONFIG_SAVE, 0xFFFF)

    def factory_reset(self) -> bool:
        """
        Reset all settings to factory defaults.

        Returns:
            True if command sent successfully

        Warning:
            This will erase all custom settings!
        """
        return self._write_register(self.REG_FACTORY_DEFAULT, 0x0F0F)

    # ========== Utility Methods ==========

    @staticmethod
    def degrees_to_position(degrees: float) -> int:
        """Convert degrees to raw position value."""
        return int(degrees * MD245MW_RS485.POSITION_PER_DEGREE)

    @staticmethod
    def position_to_degrees(position: int) -> float:
        """Convert raw position value to degrees."""
        return position / MD245MW_RS485.POSITION_PER_DEGREE

    def ping(self) -> bool:
        """
        Check if servo is responding.

        Returns:
            True if servo responds, False otherwise
        """
        try:
            result = self._read_register(self.REG_POSITION)
            return result is not None
        except Exception:
            return False

    def get_status(self) -> dict:
        """
        Read comprehensive servo status.

        Returns:
            Dictionary with all status values
        """
        status = {
            'connected': self.ping(),
            'position_deg': self.get_position(),
            'position_raw': self.get_position_raw(),
            'velocity': self.get_velocity(),
            'torque': self.get_torque(),
            'voltage': self.get_voltage(),
            'temperature': self.get_temperature(),
            'turn_count': self.get_turn_count(),
            'run_mode': self.get_run_mode(),
            'max_speed': self.get_speed(),
        }
        return status


# Convenience function for quick testing
def create_servo(port: str, servo_id: int = 1, baudrate: int = 115200) -> MD245MW_RS485:
    """
    Create and connect to a servo.

    Args:
        port: Serial port name
        servo_id: Servo ID (default 1)
        baudrate: Baud rate (default 115200)

    Returns:
        Connected MD245MW_RS485 instance
    """
    servo = MD245MW_RS485(port, servo_id, baudrate)
    if not servo.connect():
        raise RuntimeError(f"Failed to connect to servo on {port}")
    return servo


if __name__ == "__main__":
    # Basic usage example
    print("MD245MW-RS485 Servo Controller")
    print("Usage:")
    print("  from md245mw_rs485 import MD245MW_RS485")
    print("  ")
    print("  # Using context manager:")
    print("  with MD245MW_RS485('/dev/ttyUSB0', servo_id=1) as servo:")
    print("      servo.set_position(90)  # Move to 90 degrees")
    print("      print(servo.get_position())  # Read current position")
    print("  ")
    print("  # Or manual connection:")
    print("  servo = MD245MW_RS485('/dev/ttyUSB0', servo_id=1)")
    print("  servo.connect()")
    print("  servo.set_speed(500)  # Set speed")
    print("  servo.set_position(180)  # Move to 180 degrees")
    print("  servo.disconnect()")
