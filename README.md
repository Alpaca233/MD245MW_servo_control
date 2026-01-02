# MD245MW-RS485 Servo Controller

Python class for controlling Hitec MD245MW servo via RS485 protocol.

## Files

- `md245mw_rs485.py` - Main controller class
- `test_servo.py` - Test script with interactive mode

## Requirements

```bash
pip install pyserial
```

## Quick Start

```python
from md245mw_rs485 import MD245MW_RS485

# Using context manager (recommended)
with MD245MW_RS485('/dev/ttyUSB0', servo_id=1) as servo:
    servo.set_speed(500)        # Set speed
    servo.set_position(90)      # Move to 90 degrees
    print(servo.get_position()) # Read current position

# Manual connection
servo = MD245MW_RS485('/dev/ttyUSB0', servo_id=1)
servo.connect()
servo.set_position(180)
servo.disconnect()
```

Run `python3 demo_cover.py` for demoing opening and closing the cover/LED arm.

## API Reference

### Constructor

```python
MD245MW_RS485(port, servo_id=1, baudrate=115200, timeout=1.0)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| port | str | required | Serial port (e.g., '/dev/ttyUSB0' or 'COM3') |
| servo_id | int | 1 | Servo ID (1-254, 0 is broadcast) |
| baudrate | int | 115200 | Communication baud rate |
| timeout | float | 1.0 | Read timeout in seconds |

### Cover Control

| Method | Description |
|--------|-------------|
| `open_cover(speed=None)` | Open cover by moving to open position (default speed: 300) |
| `close_cover(speed=None)` | Close cover by moving to close position (default speed: 5) |

**Cover Position Constants:**
| Constant | Default | Description |
|----------|---------|-------------|
| `COVER_OPEN_POSITION` | 5 | Command angle for open position |
| `COVER_CLOSE_POSITION` | 78.5 | Command angle for close position |
| `COVER_OPEN_SPEED` | 300 | Speed for opening |
| `COVER_CLOSE_SPEED` | 5 | Speed for closing (slower) |

### Position Control

| Method | Description |
|--------|-------------|
| `set_position(angle_degrees)` | Move to angle (0-360°) |
| `set_position_raw(position)` | Set raw position (0-16383, where 4096 = 90°) |
| `get_position()` | Read current position in degrees |
| `get_position_raw()` | Read raw position value |

### Speed Control

| Method | Description |
|--------|-------------|
| `set_speed(speed)` | Set max speed (0-4095) |
| `get_speed()` | Get current max speed setting |
| `set_acceleration(ms)` | Set acceleration ramp time in milliseconds |
| `set_deceleration(ms)` | Set deceleration ramp time in milliseconds |
| `get_velocity()` | Read current velocity |

### Torque Control

| Method | Description |
|--------|-------------|
| `set_torque_max(torque)` | Set max torque/PWM duty (0-4095, 4095=100%) |
| `get_torque()` | Read current torque value |

### Status Reading

| Method | Description |
|--------|-------------|
| `get_voltage()` | Read input voltage (volts) |
| `get_temperature()` | Read MCU temperature (°C) |
| `get_turn_count()` | Read accumulated turn count |
| `get_firmware_version()` | Read firmware version |
| `get_status()` | Read comprehensive status dictionary |
| `ping()` | Check if servo is responding |

### Configuration

| Method | Description |
|--------|-------------|
| `set_run_mode(mode)` | Set run mode (0=Multi-Turn, 1=Servo, 2=CR, 3=Speed) |
| `get_run_mode()` | Get current run mode |
| `set_position_limits(min_deg, max_deg)` | Set position limits for servo mode |
| `save_config()` | Save configuration to non-volatile memory |
| `factory_reset()` | Reset all settings to factory defaults |

**Run Mode Constants:**
| Constant | Value | Description |
|----------|-------|-------------|
| `MODE_MULTI_TURN` | 0 | Multi-turn mode |
| `MODE_SERVO` | 1 | Servo mode |
| `MODE_CR` | 2 | Continuous rotation (FW >= 2.0) |
| `MODE_SPEED` | 3 | Speed mode (FW >= 2.0) |

### Utility Methods

| Method | Description |
|--------|-------------|
| `degrees_to_position(degrees)` | Static method: convert degrees to raw position value |
| `position_to_degrees(position)` | Static method: convert raw position value to degrees |

### Convenience Function

```python
from md245mw_rs485 import create_servo

# Create and connect in one call
servo = create_servo('/dev/ttyUSB0', servo_id=1, baudrate=115200)
servo.set_position(90)
servo.disconnect()
```

## Test Script Usage

```bash
# Run full test suite
python test_servo.py --port /dev/ttyUSB0 --id 1

# Quick test (connection and status only)
python test_servo.py --port /dev/ttyUSB0 --quick

# Interactive mode after tests
python test_servo.py --port /dev/ttyUSB0 --interactive

# Windows example
python test_servo.py --port COM3 --id 1 --baudrate 115200
```

### Interactive Mode Commands

| Command | Description |
|---------|-------------|
| `p <angle>` | Move to angle (degrees) |
| `s <speed>` | Set max speed (0-4095) |
| `r` | Read current status |
| `c` | Center servo (90°) |
| `q` | Quit |

## Protocol Details

The servo uses a serial packet format over RS485:

### Write Packet
```
Header(0x96) + ID + Address + REG_Length(0x02) + DataLow + DataHigh + CheckSum
```

### Read Request
```
Header(0x96) + ID + Address + REG_Length(0x00) + CheckSum
```

### Read Response
```
Header(0x69) + ID + Address + REG_Length(0x02) + DataLow + DataHigh + CheckSum
```

### Checksum Calculation
```
CheckSum = (ID + Address + REG_Length + DataLow + DataHigh) & 0xFF
```

### Position Resolution
- 4096 units = 90°
- 16384 units = 360°
- ~45.51 units per degree

### Default Communication Settings
- Baud Rate: 115200
- Data Bits: 8
- Parity: None
- Stop Bits: 1

## Key Register Addresses

| Register | Address | Description |
|----------|---------|-------------|
| REG_POSITION | 0x0C | Current position (read-only) |
| REG_VELOCITY | 0x0E | Current velocity (read-only) |
| REG_POSITION_NEW | 0x1E | Target position (read/write) |
| REG_VELOCITY_MAX | 0x54 | Max velocity setting |
| REG_RUN_MODE | 0x44 | Run mode setting |
| REG_SPEED_UP | 0xDC | Acceleration time (ms) |
| REG_SPEED_DN | 0xDE | Deceleration time (ms) |

## Troubleshooting

### Permission Denied on Linux
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Servo Not Responding
1. Check power supply to servo
2. Verify correct serial port
3. Check RS485 A/B wiring polarity
4. Confirm servo ID matches (default is usually 1)
5. Try different baud rates (common: 115200, 57600, 9600)

### Position Not Accurate
- Servo may need calibration
- Check position limits with `set_position_limits()`
- Verify run mode is set correctly (1 for servo mode)
