# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Teensy 4.1 Arduino project for communicating with a 3DM-CV7-AHRS sensor via USB host interface. The sensor uses the MIP (MicroStrain Inertial Protocol) for data communication.

## Hardware Requirements

- **Microcontroller**: Teensy 4.1 with USB Host capability
- **Sensor**: 3DM-CV7-AHRS (LORD/MicroStrain IMU/AHRS sensor)
- **Connection**: USB connection between Teensy's USB Host port and the sensor

## Development Environment

### Required Tools
- Arduino IDE with Teensyduino support
- USBHost_t36 library (included with Teensyduino)

### Build and Upload Commands
```bash
# Using Arduino CLI (if installed)
arduino-cli compile --fqbn teensy:avr:teensy41 teensy4.1_to_3DM-CV7-AHRS.ino
arduino-cli upload --fqbn teensy:avr:teensy41 --port COMx teensy4.1_to_3DM-CV7-AHRS.ino

# Note: Replace COMx with actual port number
# Most development is done through Arduino IDE with Teensy Loader
```

### Serial Monitor Settings
- Baud rate: 115200
- Line ending: Newline

## Code Architecture

### MIP Protocol Implementation
The code implements the MicroStrain Inertial Protocol (MIP) with:
- **Sync bytes**: 0x75, 0x65 for packet detection
- **Packet structure**: Sync(2) + Descriptor(1) + Length(1) + Payload(n) + Checksum(2)
- **Data sets**: 
  - 0x80: IMU data (accelerometer, gyroscope, magnetometer)
  - 0x82: AHRS data (Euler angles, quaternions)
  - 0x01: Command responses

### Key Components

1. **Connection Management**
   - Automatic detection of sensor connection/disconnection
   - State tracking via `sensorConnected` and `sensorConfigured` flags
   - Reconnection handling without manual intervention

2. **Buffer Management**
   - 1024-byte circular buffer for incoming data
   - `processBuffer()`: Handles multiple consecutive packets
   - `shiftBuffer()`: Prevents buffer overflow while preserving partial packets

3. **Data Parsing**
   - `parseFloat()`: Converts big-endian sensor data to little-endian Teensy format
   - Checksum verification for data integrity
   - Field-based parsing for IMU and AHRS data

4. **Debug Control**
   - `DEBUG_MODE` flag (line 12) controls verbose output
   - Set to `true` for troubleshooting, `false` for production

## Common Tasks

### Changing Sensor Update Rate
Modify the rate divisor in `configureSensor()` function (line 105):
```cpp
0x00, 0x00, 0x0A,  // Rate divisor (10Hz) - Change last byte for different rates
```

### Adding New Data Fields
To parse additional sensor data fields:
1. Add new field descriptor check in `parseIMUData()` or `parseAHRSData()`
2. Use `parseFloat()` for proper endian conversion
3. Follow existing pattern for data extraction

### Debugging Connection Issues
1. Set `DEBUG_MODE = true` (line 12)
2. Monitor serial output for packet details
3. Check "チェックサムエラー" messages for corrupted data

## Important Notes

- The Teensy 4.1 USB Host port provides power to the sensor
- Default sensor baud rate is 115200
- All floating-point data from sensor is big-endian
- Buffer size can be adjusted if receiving larger packets (line 15)