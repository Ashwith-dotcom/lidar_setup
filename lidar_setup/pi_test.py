#!/usr/bin/env python3
"""
RPLIDAR A1M8 Raspberry Pi Test Tool
Based on Windows improved_test.py with Raspberry Pi specific enhancements
"""

import serial
import time
import struct
import sys
import binascii

# RPLIDAR Commands
CMD_STOP = b'\xA5\x25'
CMD_RESET = b'\xA5\x40'
CMD_SCAN = b'\xA5\x20'
CMD_FORCE_SCAN = b'\xA5\x21'
CMD_GET_INFO = b'\xA5\x50'
CMD_GET_HEALTH = b'\xA5\x52'

def print_info(msg):
    print(f"[INFO] {msg}")

def print_success(msg):
    print(f"[SUCCESS] {msg}")

def print_warning(msg):
    print(f"[WARNING] {msg}")

def print_error(msg):
    print(f"[ERROR] {msg}")

def get_device_port():
    port = input("Enter device port for RPLiDAR (default: /dev/ttyUSB0): ").strip()
    if not port:
        port = "/dev/ttyUSB0"
    return port

def test_connection(port, baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, 
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS,
                           timeout=1)
        print_success(f"Serial port {port} opened successfully")
        
        # Check initial DTR state
        dtr_state = "HIGH" if ser.dtr else "LOW"
        print_info(f"Initial DTR state: {dtr_state}")
        
        return ser
    except serial.SerialException as e:
        print_error(f"Failed to open serial port: {e}")
        return None

def send_command(ser, cmd, payload=None):
    if payload:
        size = len(payload)
        if size > 255:
            print_error("Payload size exceeds maximum length")
            return False
        
        cmd_packet = cmd + struct.pack("B", size) + payload
        checksum = 0
        for b in payload:
            checksum ^= b if isinstance(b, int) else ord(b)
        cmd_packet += struct.pack("B", checksum)
    else:
        cmd_packet = cmd
    
    # First clear any existing data
    ser.reset_input_buffer()
    
    # Then send the command
    ser.write(cmd_packet)
    time.sleep(0.1)  # Allow time for command processing
    return True

def test_motor_control(ser):
    print("\nTesting motor control - setting DTR LOW to start motor...")
    ser.dtr = False  # Set DTR LOW to start motor
    time.sleep(0.5)  # Give motor time to respond
    
    response = input("Is the RPLiDAR motor spinning? (y/n)\nLook at the top part of the LIDAR - it should be rotating\n")
    if response.lower() == 'y':
        print_success("Motor started successfully!")
    else:
        print_error("Motor failed to start!")
        return False
    
    print("\nTesting motor control - setting DTR HIGH to stop motor...")
    ser.dtr = True  # Set DTR HIGH to stop motor
    time.sleep(0.5)  # Give motor time to respond
    
    response = input("Has the RPLiDAR motor stopped? (y/n)\n")
    if response.lower() == 'y':
        print_success("Motor stopped successfully!")
    else:
        print_error("Motor failed to stop!")
        return False
    
    return True

def get_device_info(ser):
    print("\nGetting device information...")
    send_command(ser, CMD_GET_INFO)
    
    time.sleep(0.1)  # Wait for response
    
    descriptor_len = 7
    descriptor = ser.read(descriptor_len)
    if len(descriptor) != descriptor_len:
        print_error(f"Failed to get response descriptor (got {len(descriptor)} bytes, expected {descriptor_len})")
        return False
    
    # Check response header
    if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
        print_error(f"Invalid response header: {binascii.hexlify(descriptor[:2])}")
        return False
    
    print_success("Valid response descriptor received")
    
    # Parse response data
    response_len = struct.unpack("<I", descriptor[2:6])[0]
    if response_len != 20:  # Expected length for device info
        print_error(f"Invalid response length: {response_len}")
        return False
    
    # Read actual response data
    response = ser.read(response_len)
    if len(response) != response_len:
        print_error(f"Incomplete response data: got {len(response)} bytes, expected {response_len}")
        return False
    
    # Extract device info
    model = response[0]
    firmware_minor = response[1]
    firmware_major = response[2]
    hardware = response[3]
    serialnum = binascii.hexlify(response[4:20]).upper().decode()
    
    print(f"Model: {model}")
    print(f"Firmware: {firmware_major}.{firmware_minor}")
    print(f"Hardware: {hardware}")
    print(f"Serial Number: {serialnum}")
    
    return True

def get_health_status(ser):
    print("\nGetting health status...")
    send_command(ser, CMD_GET_HEALTH)
    
    time.sleep(0.1)  # Wait for response
    
    descriptor_len = 7
    descriptor = ser.read(descriptor_len)
    if len(descriptor) != descriptor_len:
        print_error(f"Failed to get health response descriptor (got {len(descriptor)} bytes, expected {descriptor_len})")
        return False
    
    # Check response header
    if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
        print_error(f"Invalid health response header: {binascii.hexlify(descriptor[:2])}")
        return False
    
    print_success("Valid health response descriptor received")
    
    # Parse response data
    response_len = struct.unpack("<I", descriptor[2:6])[0]
    if response_len != 3:  # Expected length for health response
        print_error(f"Invalid health response length: {response_len}")
        return False
    
    # Read actual health data
    response = ser.read(response_len)
    if len(response) != response_len:
        print_error(f"Incomplete health data: got {len(response)} bytes, expected {response_len}")
        return False
    
    # Extract health info
    status = response[0]
    error_code = struct.unpack("<H", response[1:3])[0]
    
    status_text = "Good" if status == 0 else "Warning" if status == 1 else "Error"
    print(f"Health Status: {status_text} (code: {status})")
    print(f"Error Code: {error_code}")
    
    return status == 0

def test_scan(ser, use_force_scan=False):
    print(f"\nTesting {'force ' if use_force_scan else ''}scan mode...\n")
    
    # First stop any existing scan
    print("Stopping any existing scan...")
    send_command(ser, CMD_STOP)
    time.sleep(0.1)  # Wait for stop to complete
    
    # Start the motor and wait for it to reach proper speed
    print("Starting motor for scan test...")
    ser.dtr = False  # Start motor
    time.sleep(2.5)  # Wait 2.5 seconds for motor to reach proper speed - CRITICAL!
    
    # Start scan
    cmd = CMD_FORCE_SCAN if use_force_scan else CMD_SCAN
    cmd_name = "FORCE_SCAN" if use_force_scan else "SCAN"
    print(f"Starting {cmd_name}...")
    send_command(ser, cmd)
    
    # Wait for scan response
    time.sleep(0.1)  # Wait for response
    
    descriptor_len = 7
    descriptor = ser.read(descriptor_len)
    if len(descriptor) != descriptor_len:
        print_error(f"Failed to get {cmd_name} response descriptor (got {len(descriptor)} bytes, expected {descriptor_len})")
        print_error(f"Raw response: {binascii.hexlify(descriptor)}")
        return False
    
    # Check response header
    if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
        print_error(f"Invalid {cmd_name} response header: {binascii.hexlify(descriptor[:2])}")
        return False
    
    print_success(f"Valid {cmd_name} response descriptor received")
    
    print("\nReading scan data packets...")
    
    # Try to read scan data
    valid_data_points = 0
    timeout_counter = 0
    retry_count = 0
    
    # Read scan data for a few seconds
    start_time = time.time()
    while time.time() - start_time < 10:  # Try for 10 seconds (longer for Pi)
        try:
            # First byte should be a response, either 0xA or quality
            response_byte = ser.read(1)
            if not response_byte:
                timeout_counter += 1
                if timeout_counter > 10:
                    print_warning("Multiple timeouts when reading data")
                    retry_count += 1
                    if retry_count >= 3:
                        break
                continue
            
            timeout_counter = 0  # Reset timeout counter
            
            # Check if it's a data packet
            if len(response_byte) < 1:
                continue
            
            first_byte = response_byte[0]
            
            # Dump first few bytes to debug
            if valid_data_points == 0:
                print(f"First byte: 0x{first_byte:02X} (bin: {bin(first_byte)[2:].zfill(8)})")
            
            # Read the rest of the packet based on the protocol
            if (first_byte & 0x1) == 0x1:  # If LSB is 1, it's the beginning of a new scan
                # For the A1, each data packet should have 5 bytes
                remaining_data = ser.read(4)  # Already read 1 byte
                if len(remaining_data) < 4:
                    print_warning(f"Incomplete data packet: got {len(remaining_data)+1} bytes, expected 5")
                    continue
                
                # Parse angle and distance
                packet = response_byte + remaining_data
                
                # Dump first data packet for debugging
                if valid_data_points == 0:
                    print(f"First packet: {binascii.hexlify(packet)}")
                    print(f"Byte 0: 0x{packet[0]:02X} (Check: {bool(packet[0] & 0x1)})")
                    print(f"Byte 1: 0x{packet[1]:02X} (Check: {bool(packet[1] & 0x1)})")
                
                # Check if it's a valid data packet
                if packet[0] & 0x1 and packet[1] & 0x1:
                    # Proper data packet
                    angle_q6 = ((packet[1] >> 1) + (packet[2] << 7)) & 0x7FFF
                    angle = angle_q6 / 64.0
                    
                    distance_q2 = ((packet[3] >> 2) + (packet[4] << 6)) & 0x3FFF
                    distance = distance_q2 / 4.0
                    
                    if distance > 0:
                        valid_data_points += 1
                        if valid_data_points <= 10 or valid_data_points % 100 == 0:  # Show the first 10 points and every 100th after
                            print(f"Point: Angle={angle:.2f}°, Distance={distance:.2f}mm")
            else:
                # Skip any unexpected bytes
                if valid_data_points == 0:
                    print(f"Skipping unexpected byte: 0x{first_byte:02X}")
                continue
        
        except Exception as e:
            print_error(f"Error reading scan data: {e}")
            break
    
    print(f"\nReceived {valid_data_points} valid data points")
    
    # Stop the scan
    print("Stopping scan...")
    send_command(ser, CMD_STOP)
    time.sleep(0.1)
    
    # Stop the motor
    print("Stopping motor...")
    ser.dtr = True
    time.sleep(0.5)
    
    return valid_data_points > 0

def analyze_power_issues(ser, scan_successful):
    print("\n========================================")
    print("POWER ANALYSIS")
    print("========================================\n")
    
    if scan_successful:
        print_success("No power issues detected.")
        return
    
    print_warning("Potential power issues detected:\n")
    
    # Test power stability with rapid motor control cycles
    print("Testing power stability with rapid motor control...")
    stability_issues = False
    
    for i in range(3):
        print(f"Test cycle {i+1}/3:")
        
        # Start motor
        ser.dtr = False
        time.sleep(1.0)
        
        # Try to get health
        send_command(ser, CMD_GET_HEALTH)
        time.sleep(0.1)
        descriptor = ser.read(7)
        
        if len(descriptor) != 7 or descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            print_warning("Communication unstable when motor is running")
            stability_issues = True
        
        # Stop motor
        ser.dtr = True
        time.sleep(0.5)
    
    if stability_issues:
        print_warning("1. Power stability issue detected - motor reset doesn't recover")
    
    print("\nRECOMMENDED POWER SOLUTIONS:")
    print("1. Raspberry Pi: use a 5V/3A power supply for the Pi")
    print("2. Connect LIDAR directly to Pi (not through a USB hub)")
    print("3. Use a powered USB hub with its own power supply")
    print("4. Use a separate 5V/1A power supply for the RPLiDAR")
    print("5. Try a higher-quality USB cable (thicker gauge, shorter length)")
    print("6. Make sure no other high-power devices share the same power supply")

def main():
    print("\n============================================================")
    print("RPLiDAR A1M8 Raspberry Pi Test Tool")
    print("============================================================\n")
    
    print("This tool will test your RPLiDAR A1M8 on Raspberry Pi")
    print("and help diagnose any power or connection issues.")
    
    port = get_device_port()
    
    # Connect to LIDAR
    ser = test_connection(port)
    if not ser:
        print_error("Failed to connect to RPLiDAR")
        return
    
    try:
        # Reset the device first
        print("\nResetting RPLiDAR...")
        send_command(ser, CMD_RESET)
        time.sleep(2)  # Wait for device to reset
        
        # Test motor control
        motor_ok = test_motor_control(ser)
        if not motor_ok:
            print_warning("Motor control test failed - continuing with other tests")
        
        # Get device info
        info_ok = get_device_info(ser)
        if not info_ok:
            print_warning("Failed to get device information - continuing with other tests")
        
        # Get health status
        health_ok = get_health_status(ser)
        if not health_ok:
            print_warning("Device reported health issues - continuing with other tests")
        
        # Test standard scan mode
        scan_ok = test_scan(ser, use_force_scan=False)
        
        # If standard scan failed, try force scan
        if not scan_ok:
            print_warning("Standard scan failed, trying force scan...")
            scan_ok = test_scan(ser, use_force_scan=True)
        
        # Analyze power issues if scan failed
        if not scan_ok:
            analyze_power_issues(ser, scan_ok)
        
    finally:
        # Clean up
        print("\nClosing serial port...")
        ser.close()
        print_info("Serial port closed")
    
    print("\nTest completed.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print_error(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nTest completed.") 