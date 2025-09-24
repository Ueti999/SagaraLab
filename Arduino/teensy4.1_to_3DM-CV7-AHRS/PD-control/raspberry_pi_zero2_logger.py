#!/usr/bin/env python3
"""
Raspberry Pi Zero 2 Data Logger for Teensy 4.1
Receive data from Teensy via USB serial and save to SD card
"""

import serial
import csv
import time
from datetime import datetime
import os
import sys

# Serial port settings
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Log file settings
LOG_DIR = '/home/pi/teensy_logs'
os.makedirs(LOG_DIR, exist_ok=True)

def main():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(LOG_DIR, f'teensy_log_{timestamp}.csv')

    print(f"Data logger started")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"Log file: {log_file}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)

        with open(log_file, 'w', newline='') as csvfile:
            csvfile.write("timestamp,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,theta_motor,theta_leg,dtheta_leg,walk_count,target,torque\n")
            csvfile.flush()

            print("Connection established. Receiving data... (Ctrl+C to stop)")

            while True:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8').strip()

                    if line.startswith('#CONNECTED:'):
                        print(f"Teensy connected: {line}")
                        ser.write(b"#CMD:START\n")
                    elif line and not line.startswith('#'):
                        csvfile.write(line + '\n')
                        csvfile.flush()

                        if csvfile.tell() % 100 == 0:
                            print(f"Receiving data... {line}")

    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nLogging stopped")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print(f"Log file saved: {log_file}")

if __name__ == "__main__":
    main()