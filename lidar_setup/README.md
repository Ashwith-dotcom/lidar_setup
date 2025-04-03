# RPLiDAR A1M8 Setup for Autonomous Delivery Robot

This guide outlines the setup and implementation of the SLAMTEC RPLiDAR A1M8 on a Raspberry Pi 4 for obstacle detection in an autonomous delivery robot.

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Wiring](#wiring)
- [Software Setup](#software-setup)
- [Available Tools](#available-tools)
- [Running the Tests](#running-the-tests)
- [Troubleshooting](#troubleshooting)
- [Integration with Robot](#integration-with-robot)
- [Resources](#resources)

## Hardware Requirements
- Raspberry Pi 4
- SLAMTEC RPLiDAR A1M8 360° Laser Range Finder (6m radius range)
- USB-to-UART adapter (CP2102 or FTDI recommended for good DTR support)
- Power supply for RPLiDAR A1M8 (5V/1A recommended)

## Wiring

Connect the RPLiDAR A1M8 to the Raspberry Pi through the USB-to-UART adapter:

| RPLiDAR Pin | Connect to           |
|-------------|----------------------|
| TX          | RX on USB Adapter    |
| RX          | TX on USB Adapter    |
| GND         | GND on USB Adapter   |
| MOTCTL      | DTR on USB Adapter   |
| 5V          | 5V Power Supply (1A) |

> **Important**: Make sure your USB-to-UART adapter supports DTR pin control for the motor. Some CH340 adapters may not work properly.

## Software Setup

1. Install required packages:
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-numpy
   pip3 install pyserial matplotlib
   ```

2. Clone this repository or copy the files to your Raspberry Pi:
   ```bash
   git clone https://github.com/yourusername/rplidar-setup
   cd rplidar-setup
   ```

3. Make the scripts executable:
   ```bash
   chmod +x *.py test_all.sh
   ```

4. Run the USB adapter test to ensure your adapter supports DTR control:
   ```bash
   python3 usb_adapter_test.py
   ```

5. If you have permission issues with the serial port:
   ```bash
   sudo chmod a+rw /dev/ttyUSB0  # Replace with your port
   ```
   
   For permanent access, add your user to the dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Then log out and log back in.

## Available Tools

This package includes multiple tools for different testing and usage scenarios:

| File                   | Description                                              |
|------------------------|----------------------------------------------------------|
| `rplidar_driver.py`    | Main driver with full functionality                      |
| `rplidar_visualization.py` | Real-time visualization tool                         |
| `robot_integration.py` | Example for autonomous robot navigation                  |
| `simple_lidar.py`      | Simplified driver focused on reliability                 |
| `force_scan_test.py`   | Test using FORCE_SCAN command (often more reliable)      |
| `serial_test.py`       | Direct communication test for troubleshooting            |
| `usb_adapter_test.py`  | Tests if the USB adapter supports DTR control            |
| `ros_compatible.py`    | ROS-compatible driver (standalone or with ROS)           |
| `test_all.sh`          | Shell script to run all test tools in sequence           |

## Running the Tests

The easiest way to start is with the test script:

```bash
./test_all.sh
```

This will run all the test tools in sequence, helping you identify which one works best with your hardware.

### Individual Tools

To run specific tools:

1. Basic functionality test:
   ```bash
   python3 rplidar_driver.py
   ```

2. Visualization (requires GUI or X forwarding):
   ```bash
   python3 rplidar_visualization.py
   ```

3. Robot integration simulation:
   ```bash
   python3 robot_integration.py
   ```

4. Force scan test (if normal scan doesn't work):
   ```bash
   python3 force_scan_test.py
   ```

## Troubleshooting

For detailed troubleshooting steps, refer to [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

Common issues:
- Serial port not found
- Permission denied on port
- Motor not spinning
- No data points detected
- Inconsistent or noisy data

## Integration with Robot

The `robot_integration.py` file provides a simple example of how to use RPLiDAR data for obstacle avoidance in an autonomous robot.

To integrate with your own robot:

1. **Navigation Control**: Modify the `RobotController._set_direction()` method to interface with your motor controllers.

2. **Tune Parameters**: Adjust these values in `robot_integration.py`:
   - `MIN_DISTANCE`: Minimum safe distance (mm)
   - `CAUTION_DISTANCE`: Distance to start avoiding obstacles (mm)
   - `FORWARD_ANGLE_RANGE`: Angle range to check for forward obstacles (degrees)
   - `SIDE_ANGLE_RANGE`: Angle range to check for side obstacles (degrees)

3. **ROS Integration**: For ROS-based robots, use `ros_compatible.py` which outputs scan data in a format compatible with ROS LaserScan messages.

## Resources

- [SLAMTEC RPLiDAR A1M8 User Manual](https://bucket-download.slamtec.com/af084741a46129dfcf2b516110be558561d55767/LM108_SLAMTEC_rplidarkit_usermanual_A1M8_v2.2_en.pdf)
- [RPLiDAR SDK](https://github.com/Slamtec/rplidar_sdk)
- [RPLiDAR SDK Documentation](https://bucket-download.slamtec.com/6957283725b66750890024d1f0d12940fa079e06/LR002_SLAMTEC_rplidar_sdk_v2.0_en.pdf)

## Important Updates & Fixes

### Critical Timing Fix

We've discovered that the most critical issue with the RPLiDAR A1M8 is proper timing between starting the motor and sending scan commands. **The motor requires approximately 2.5 seconds to reach proper speed before it can provide valid scan data.**

All scripts in this folder have been updated to include this critical delay. If you were experiencing issues with receiving scan data (even though the motor was spinning), this timing fix should resolve the problem. 