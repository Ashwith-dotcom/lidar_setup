# RPLiDAR A1M8 Setup for Autonomous Delivery Robot

This guide outlines the setup and implementation of the SLAMTEC RPLiDAR A1M8 on a Raspberry Pi 4 for obstacle detection in an autonomous delivery robot.

## Hardware Requirements
- Raspberry Pi 4
- SLAMTEC RPLiDAR A1M8 360° Laser Range Finder (6m radius range)
- USB-to-UART adapter (if not connecting directly via GPIO)
- Power supply for RPLiDAR A1M8 (5V/1A recommended)

## Software Requirements
- Raspberry Pi OS (Bullseye or newer recommended)
- Python 3.7+
- Required Python packages:
  - pyserial
  - numpy
  - matplotlib (for visualization, optional)

## Connection Setup
1. Connect the RPLiDAR A1M8 to the Raspberry Pi:
   - Connect RPLiDAR's TX to RX on Raspberry Pi (or USB adapter)
   - Connect RPLiDAR's RX to TX on Raspberry Pi (or USB adapter)
   - Connect RPLiDAR's GND to GND on Raspberry Pi
   - Connect RPLiDAR's +5V to 5V power supply

## Software Setup
1. Install required packages:
   ```
   sudo apt update
   sudo apt install python3-pip python3-numpy
   pip3 install pyserial matplotlib
   ```

2. Clone the RPLiDAR SDK (optional, for reference):
   ```
   git clone https://github.com/Slamtec/rplidar_sdk
   ```

3. Run the implementation script:
   ```
   python3 rplidar_driver.py
   ```

## Troubleshooting
- If you encounter permission issues with the serial port, run:
  ```
  sudo chmod a+rw /dev/ttyUSB0
  ```
  (Replace ttyUSB0 with your actual port)

- Make sure the motor is spinning when taking measurements

- Verify correct power supply voltage and current 