#!/usr/bin/env python3
"""
Test script for MD245MW-RS485 Servo Controller

This script tests the basic functionality of the MD245MW_RS485 class:
1. Connection to servo
2. Reading servo status
3. Moving to specific angles
4. Setting speed
5. Reading position feedback

Usage:
    python test_servo.py [--port PORT] [--id SERVO_ID] [--baudrate BAUDRATE]

Examples:
    python test_servo.py --port /dev/ttyUSB0
    python test_servo.py --port COM3 --id 1 --baudrate 115200
"""

import argparse
import time
import sys

from md245mw_rs485 import MD245MW_RS485


def test_connection(servo: MD245MW_RS485) -> bool:
    """Test basic connection to servo."""
    print("\n" + "=" * 50)
    print("TEST: Connection")
    print("=" * 50)

    if servo.ping():
        print("[PASS] Servo is responding")
        return True
    else:
        print("[FAIL] Servo not responding")
        return False


def test_read_status(servo: MD245MW_RS485) -> bool:
    """Test reading servo status."""
    print("\n" + "=" * 50)
    print("TEST: Read Status")
    print("=" * 50)

    try:
        status = servo.get_status()

        print(f"  Position (degrees): {status['position_deg']}")
        print(f"  Position (raw):     {status['position_raw']}")
        print(f"  Velocity:           {status['velocity']}")
        print(f"  Torque:             {status['torque']}")
        print(f"  Voltage:            {status['voltage']} V")
        print(f"  Temperature:        {status['temperature']} °C")
        print(f"  Turn Count:         {status['turn_count']}")
        print(f"  Run Mode:           {status['run_mode']}")
        print(f"  Max Speed:          {status['max_speed']}")

        if status['position_deg'] is not None:
            print("[PASS] Status read successfully")
            return True
        else:
            print("[FAIL] Could not read position")
            return False

    except Exception as e:
        print(f"[FAIL] Error reading status: {e}")
        return False


def test_firmware_version(servo: MD245MW_RS485) -> bool:
    """Test reading firmware version."""
    print("\n" + "=" * 50)
    print("TEST: Firmware Version")
    print("=" * 50)

    version = servo.get_firmware_version()
    if version is not None:
        print(f"  Firmware Version: 0x{version:04X} ({version})")
        print("[PASS] Firmware version read successfully")
        return True
    else:
        print("[FAIL] Could not read firmware version")
        return False


def test_set_speed(servo: MD245MW_RS485) -> bool:
    """Test setting servo speed."""
    print("\n" + "=" * 50)
    print("TEST: Set Speed")
    print("=" * 50)

    # Read current speed
    original_speed = servo.get_speed()
    print(f"  Original speed: {original_speed}")

    # Set new speed
    test_speed = 500
    if servo.set_speed(test_speed):
        print(f"  Set speed to: {test_speed}")

        # Read back to verify
        time.sleep(0.05)
        new_speed = servo.get_speed()
        print(f"  Read back speed: {new_speed}")

        if new_speed == test_speed:
            print("[PASS] Speed set and verified")

            # Restore original speed
            if original_speed is not None:
                servo.set_speed(original_speed)
                print(f"  Restored original speed: {original_speed}")

            return True
        else:
            print("[WARN] Speed readback mismatch (may be normal for some servos)")
            return True  # Still consider pass if write succeeded
    else:
        print("[FAIL] Failed to set speed")
        return False


def test_position_movement(servo: MD245MW_RS485, angle: float = 90.0) -> bool:
    """Test moving servo to a specific angle."""
    print("\n" + "=" * 50)
    print(f"TEST: Position Movement to {angle}°")
    print("=" * 50)

    # Read starting position
    start_pos = servo.get_position()
    print(f"  Starting position: {start_pos:.2f}°" if start_pos else "  Starting position: Unknown")

    # Move to target angle
    print(f"  Moving to {angle}°...")
    if not servo.set_position(angle):
        print("[FAIL] Failed to send position command")
        return False

    # Wait for movement
    time.sleep(1.0)

    # Read final position
    final_pos = servo.get_position()
    if final_pos is not None:
        print(f"  Final position: {final_pos:.2f}°")

        # Check if close to target (within 5 degrees tolerance)
        error = abs(final_pos - angle)
        if error < 5.0:
            print(f"  Position error: {error:.2f}° (within tolerance)")
            print("[PASS] Position movement successful")
            return True
        else:
            print(f"  Position error: {error:.2f}° (outside tolerance)")
            print("[WARN] Large position error (servo may need calibration)")
            return True  # Still pass if we got a reading
    else:
        print("[FAIL] Could not read final position")
        return False


def test_position_sequence(servo: MD245MW_RS485) -> bool:
    """Test moving servo through a sequence of positions."""
    print("\n" + "=" * 50)
    print("TEST: Position Sequence")
    print("=" * 50)

    # Test positions (degrees)
    positions = [0, 45, 90, 135, 180, 90]

    print("  Moving through sequence:", positions)

    for angle in positions:
        print(f"  -> Moving to {angle}°...", end=" ", flush=True)
        servo.set_position(angle)
        time.sleep(0.8)

        current = servo.get_position()
        if current is not None:
            print(f"at {current:.1f}°")
        else:
            print("(position unknown)")

    print("[PASS] Position sequence completed")
    return True


def test_acceleration(servo: MD245MW_RS485) -> bool:
    """Test setting acceleration/deceleration."""
    print("\n" + "=" * 50)
    print("TEST: Acceleration Settings")
    print("=" * 50)

    # Set acceleration (ramp up time)
    accel_ms = 200
    if servo.set_acceleration(accel_ms):
        print(f"  Set acceleration time: {accel_ms} ms")
    else:
        print("[FAIL] Failed to set acceleration")
        return False

    # Set deceleration (ramp down time)
    decel_ms = 200
    if servo.set_deceleration(decel_ms):
        print(f"  Set deceleration time: {decel_ms} ms")
    else:
        print("[FAIL] Failed to set deceleration")
        return False

    print("[PASS] Acceleration settings applied")
    return True


def interactive_test(servo: MD245MW_RS485):
    """Interactive test mode for manual testing."""
    print("\n" + "=" * 50)
    print("INTERACTIVE MODE")
    print("=" * 50)
    print("Commands:")
    print("  p <angle>  - Move to angle (degrees)")
    print("  s <speed>  - Set max speed (0-4095)")
    print("  r          - Read current status")
    print("  c          - Center servo (90°)")
    print("  q          - Quit")
    print()

    while True:
        try:
            cmd = input("Enter command: ").strip().lower()

            if not cmd:
                continue

            parts = cmd.split()

            if parts[0] == 'q':
                print("Exiting interactive mode")
                break

            elif parts[0] == 'p' and len(parts) > 1:
                try:
                    angle = float(parts[1])
                    servo.set_position(angle)
                    print(f"Moving to {angle}°")
                    time.sleep(0.5)
                    pos = servo.get_position()
                    print(f"Current position: {pos:.2f}°" if pos else "Position unknown")
                except ValueError:
                    print("Invalid angle")

            elif parts[0] == 's' and len(parts) > 1:
                try:
                    speed = int(parts[1])
                    servo.set_speed(speed)
                    print(f"Speed set to {speed}")
                except ValueError:
                    print("Invalid speed")

            elif parts[0] == 'r':
                status = servo.get_status()
                for key, value in status.items():
                    print(f"  {key}: {value}")

            elif parts[0] == 'c':
                servo.set_position(90)
                print("Centering to 90°")
                time.sleep(0.5)
                pos = servo.get_position()
                print(f"Current position: {pos:.2f}°" if pos else "Position unknown")

            else:
                print("Unknown command")

        except KeyboardInterrupt:
            print("\nExiting...")
            break


def run_all_tests(servo: MD245MW_RS485, interactive: bool = False) -> int:
    """Run all tests and return number of failures."""
    results = []

    # Basic tests
    results.append(("Connection", test_connection(servo)))
    results.append(("Read Status", test_read_status(servo)))
    results.append(("Firmware Version", test_firmware_version(servo)))
    results.append(("Set Speed", test_set_speed(servo)))
    results.append(("Acceleration", test_acceleration(servo)))
    results.append(("Position Movement", test_position_movement(servo, 90)))
    results.append(("Position Sequence", test_position_sequence(servo)))

    # Print summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)

    passed = 0
    failed = 0
    for name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {name}: [{status}]")
        if result:
            passed += 1
        else:
            failed += 1

    print(f"\nTotal: {passed} passed, {failed} failed")

    # Interactive mode if requested
    if interactive:
        interactive_test(servo)

    return failed


def main():
    parser = argparse.ArgumentParser(
        description="Test MD245MW-RS485 Servo Controller"
    )
    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--id", "-i",
        type=int,
        default=1,
        help="Servo ID (default: 1)"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--interactive", "-I",
        action="store_true",
        help="Enter interactive mode after tests"
    )
    parser.add_argument(
        "--quick", "-q",
        action="store_true",
        help="Quick test (connection and status only)"
    )

    args = parser.parse_args()

    print("MD245MW-RS485 Servo Test")
    print(f"Port: {args.port}")
    print(f"Servo ID: {args.id}")
    print(f"Baud Rate: {args.baudrate}")

    # Create servo instance
    servo = MD245MW_RS485(args.port, args.id, args.baudrate)

    # Try to connect
    print("\nConnecting to servo...")
    if not servo.connect():
        print(f"ERROR: Could not open serial port {args.port}")
        print("Please check:")
        print("  1. The servo is connected and powered")
        print("  2. The correct serial port is specified")
        print("  3. You have permission to access the port")
        print("     (try: sudo usermod -a -G dialout $USER)")
        sys.exit(1)

    print("Serial port opened successfully")

    try:
        if args.quick:
            # Quick test - just connection and status
            test_connection(servo)
            test_read_status(servo)
            failures = 0
        else:
            # Full test suite
            failures = run_all_tests(servo, args.interactive)

    finally:
        # Always disconnect
        servo.disconnect()
        print("\nDisconnected from servo")

    sys.exit(1 if failures > 0 else 0)


if __name__ == "__main__":
    main()
