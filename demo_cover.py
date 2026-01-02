#!/usr/bin/env python3
"""Demo script for cover open/close control."""

from md245mw_rs485 import MD245MW_RS485
import time

# Connect to servo
servo = MD245MW_RS485('/dev/ttyUSB0', servo_id=0, baudrate=115200)
servo.connect()

print('Cover Demo')
print('=' * 30)

# Open cover
print('Opening cover...')
servo.open_cover()
print(f'Position: {servo.get_position():.1f}°')

# Close cover
print('Closing cover...')
servo.close_cover()
print(f'Position: {servo.get_position():.1f}°')

servo.disconnect()
print('Done')
