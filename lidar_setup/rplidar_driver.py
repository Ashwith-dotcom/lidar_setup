#!/usr/bin/env python3
# RPLiDAR A1M8 Driver for Autonomous Delivery Robot
# Based on SLAMTEC RPLiDAR SDK and protocol documentation

import serial
import time
import struct
import numpy as np
import threading
import math
from collections import deque

class RPLidar:
    """
    Class for interfacing with RPLiDAR A1M8
    """
    # Constants from RPLiDAR protocol
    START_FLAG = b'\xA5'
    STOP_BYTE = b'\x25'
    RESET_BYTE = b'\x40'
    SCAN_BYTE = b'\x20'
    FORCE_SCAN_BYTE = b'\x21'
    GET_INFO_BYTE = b'\x50'
    GET_HEALTH_BYTE = b'\x52'
    
    # Response descriptor length
    DESCRIPTOR_LEN = 7
    
    # Health status
    HEALTH_STATUS = {
        0: "Good",
        1: "Warning",
        2: "Error"
    }
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        """
        Initialize RPLidar object
        
        Args:
            port (str): Serial port device
            baudrate (int): Baud rate for serial communication
            timeout (int): Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.is_scanning = False
        self.motor_running = False
        self.scan_thread = None
        self.points = deque(maxlen=360)  # Store a full 360° scan
        self.distances = np.zeros(360)  # Distance data in mm for each degree
        self._init_empty_scan()
    
    def _init_empty_scan(self):
        """Initialize the distances array with zeros"""
        for i in range(360):
            self.distances[i] = 0
    
    def connect(self):
        """
        Connect to the RPLiDAR device
        
        Returns:
            bool: True if connected successfully, False otherwise
        """
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            if self.serial.is_open:
                print(f"Connected to RPLiDAR on {self.port}")
                time.sleep(1)  # Give device time to initialize
                return True
        except serial.SerialException as e:
            print(f"Failed to connect to RPLiDAR: {e}")
            return False
        return False
    
    def disconnect(self):
        """
        Disconnect from the RPLiDAR device
        """
        if self.is_scanning:
            self.stop_scan()
        
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from RPLiDAR")
    
    def _send_cmd(self, cmd, payload=None):
        """
        Send command to RPLiDAR
        
        Args:
            cmd (bytes): Command byte
            payload (bytes): Optional payload data
        """
        if not self.serial or not self.serial.is_open:
            raise Exception("Device not connected")
            
        # Clear input buffer before sending new command
        self.serial.reset_input_buffer()
        
        # Construct command packet
        packet = self.START_FLAG + cmd
        
        if payload:
            size = len(payload)
            packet += struct.pack('B', size)  # Size byte
            packet += payload
        else:
            packet += b'\x00'  # Empty payload
            
        # Send command
        self.serial.write(packet)
        time.sleep(0.01)  # Short delay to ensure command is sent
    
    def reset(self):
        """
        Reset the RPLiDAR device
        
        Returns:
            bool: True if reset successful, False otherwise
        """
        try:
            self._send_cmd(self.RESET_BYTE)
            time.sleep(2)  # Wait for device to reset
            return True
        except Exception as e:
            print(f"Failed to reset RPLiDAR: {e}")
            return False
    
    def get_health(self):
        """
        Get health status of RPLiDAR
        
        Returns:
            tuple: (status, error_code) where status is a string description
        """
        try:
            self._send_cmd(self.GET_HEALTH_BYTE)
            
            # Read descriptor
            descriptor = self.serial.read(self.DESCRIPTOR_LEN)
            if len(descriptor) < self.DESCRIPTOR_LEN:
                raise Exception("Failed to get descriptor")
            
            # Check if it's a valid health response
            if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
                raise Exception("Invalid health descriptor")
                
            # Read response data (3 bytes: status, error_code low, error_code high)
            response = self.serial.read(3)
            if len(response) < 3:
                raise Exception("Failed to get health data")
                
            status = response[0]
            error_code = response[1] | (response[2] << 8)
            
            return (self.HEALTH_STATUS.get(status, "Unknown"), error_code)
        except Exception as e:
            print(f"Failed to get health: {e}")
            return ("Unknown", 0)
    
    def get_info(self):
        """
        Get device information
        
        Returns:
            dict: Device information
        """
        try:
            self._send_cmd(self.GET_INFO_BYTE)
            
            # Read descriptor
            descriptor = self.serial.read(self.DESCRIPTOR_LEN)
            if len(descriptor) < self.DESCRIPTOR_LEN:
                raise Exception("Failed to get descriptor")
            
            # Validate descriptor
            if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
                raise Exception("Invalid info descriptor")
                
            # Get data length from descriptor
            data_length = descriptor[2]
            
            # Read response data (20 bytes for A1M8)
            response = self.serial.read(data_length)
            if len(response) < data_length:
                raise Exception(f"Failed to get info data, got {len(response)} bytes, expected {data_length}")
                
            model = response[0]
            firmware_minor = response[1]
            firmware_major = response[2]
            hardware = response[3]
            serial_number = "".join([f"{b:02X}" for b in response[4:20]])
            
            return {
                "model": model,
                "firmware": f"{firmware_major}.{firmware_minor}",
                "hardware": hardware,
                "serial_number": serial_number
            }
        except Exception as e:
            print(f"Failed to get info: {e}")
            return {}
    
    def start_motor(self):
        """
        Start the RPLiDAR motor
        
        Note: For RPLiDAR A1, the motor is controlled through the DTR pin
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not self.serial or not self.serial.is_open:
                return False
            
            # Set DTR to LOW to start the motor
            self.serial.dtr = False
            self.motor_running = True
            print("Motor started, waiting for it to reach proper speed...")
            time.sleep(2.5)  # CRITICAL: Wait 2.5s for motor to reach proper speed
            print("Motor should now be at proper speed")
            return True
        except Exception as e:
            print(f"Failed to start motor: {e}")
            return False
    
    def stop_motor(self):
        """
        Stop the RPLiDAR motor
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not self.serial or not self.serial.is_open:
                return False
                
            # Set DTR to HIGH to stop the motor
            self.serial.dtr = True
            self.motor_running = False
            time.sleep(0.5)  # Wait for motor to stop
            return True
        except Exception as e:
            print(f"Failed to stop motor: {e}")
            return False
    
    def start_scan(self):
        """
        Start RPLiDAR scanning
        
        Returns:
            bool: True if scan started successfully, False otherwise
        """
        if self.is_scanning:
            return True
            
        try:
            # Clear existing scan
            self._init_empty_scan()
            
            # Make sure the motor is running
            if not self.motor_running:
                self.start_motor()
            else:
                # If motor is already running, still need to ensure it's at proper speed
                print("Motor already running, ensuring it's at proper speed...")
                time.sleep(1.0)  # Give motor time to stabilize
            
            # Stop any existing scan first
            self._send_cmd(self.STOP_BYTE)
            time.sleep(0.1)  # Wait for stop to complete
            
            # Clear input buffer before starting scan
            self.serial.reset_input_buffer()
            
            # Start scan command
            print("Starting scan...")
            self._send_cmd(self.SCAN_BYTE)
            
            # Read response descriptor
            descriptor = self.serial.read(self.DESCRIPTOR_LEN)
            if len(descriptor) < self.DESCRIPTOR_LEN:
                raise Exception(f"Failed to get scan descriptor, received {len(descriptor)} bytes")
            
            # Validate descriptor
            if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
                raise Exception(f"Invalid scan descriptor: {descriptor.hex()}")
            
            print("Scan started successfully")
            self.is_scanning = True
            
            # Start scanning thread
            self.scan_thread = threading.Thread(target=self._scan_thread)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            
            return True
        except Exception as e:
            print(f"Failed to start scan: {e}")
            # Try force scan as fallback
            return self.start_force_scan()
    
    def start_force_scan(self):
        """
        Start RPLiDAR scanning using FORCE_SCAN command
        This is more reliable in some situations
        
        Returns:
            bool: True if force scan started successfully, False otherwise
        """
        if self.is_scanning:
            return True
            
        try:
            # Clear existing scan
            self._init_empty_scan()
            
            # Make sure the motor is running
            if not self.motor_running:
                self.start_motor()
            else:
                # If motor is already running, still need to ensure it's at proper speed
                print("Motor already running, ensuring it's at proper speed...")
                time.sleep(1.0)  # Give motor time to stabilize
            
            # Stop any existing scan first
            self._send_cmd(self.STOP_BYTE)
            time.sleep(0.1)  # Wait for stop to complete
            
            # Clear input buffer before starting scan
            self.serial.reset_input_buffer()
            
            # Start force scan command
            print("Starting force scan...")
            self._send_cmd(self.FORCE_SCAN_BYTE)
            
            # Read response descriptor
            descriptor = self.serial.read(self.DESCRIPTOR_LEN)
            if len(descriptor) < self.DESCRIPTOR_LEN:
                raise Exception(f"Failed to get force scan descriptor, received {len(descriptor)} bytes")
            
            # Validate descriptor
            if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
                raise Exception(f"Invalid force scan descriptor: {descriptor.hex()}")
            
            print("Force scan started successfully")
            self.is_scanning = True
            
            # Start scanning thread
            self.scan_thread = threading.Thread(target=self._scan_thread)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            
            return True
        except Exception as e:
            print(f"Failed to start force scan: {e}")
            return False
    
    def stop_scan(self):
        """
        Stop the scanning process
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_scanning:
            return True
            
        try:
            self.is_scanning = False
            
            # Send stop command
            self._send_cmd(self.STOP_BYTE)
            
            # Wait for scan thread to complete
            if self.scan_thread and self.scan_thread.is_alive():
                self.scan_thread.join(2)
                
            # Stop motor
            self.stop_motor()
            
            return True
        except Exception as e:
            print(f"Failed to stop scan: {e}")
            return False
    
    def _scan_thread(self):
        """
        Thread that continuously reads scan data
        """
        while self.is_scanning:
            try:
                # Read one data packet
                # Data packet format (5 bytes): quality, angle_low, angle_high, dist_low, dist_high
                data = self.serial.read(5)
                
                if len(data) != 5:
                    continue
                    
                # Check if first byte has the check bit (bit 7) set
                if not (data[0] & 0x01):
                    # Invalid data, skip this packet
                    continue
                    
                # Parse packet
                quality = data[0] >> 2  # Remove check bits
                angle = ((data[2] << 8) | data[1]) / 64.0  # Convert to degrees
                distance = ((data[4] << 8) | data[3]) / 4.0  # Convert to mm
                
                # Convert angle to nearest degree and use as index (0-359)
                angle_index = int(angle) % 360
                
                # Store the point data
                point = (angle, distance, quality)
                self.points.append(point)
                
                # Update distance array - 0 means no data
                if distance > 0:
                    self.distances[angle_index] = distance
                
            except Exception as e:
                print(f"Error in scan thread: {e}")
                time.sleep(0.01)
    
    def get_obstacle_distances(self, min_distance=0, max_distance=6000):
        """
        Get filtered obstacle distances
        
        Args:
            min_distance (float): Minimum valid distance in mm
            max_distance (float): Maximum valid distance in mm
            
        Returns:
            numpy.ndarray: Array of distances in mm for each angle (0-359)
        """
        # Create a copy of the distances array
        filtered_distances = self.distances.copy()
        
        # Filter out distances outside the valid range
        filtered_distances[(filtered_distances < min_distance) | (filtered_distances > max_distance)] = 0
        
        return filtered_distances
    
    def detect_obstacles(self, threshold_distance=500, min_angle=0, max_angle=360, 
                         min_points=3, filter_range=True, filter_window=3):
        """
        Detect obstacles in the current scan
        
        Args:
            threshold_distance (float): Distance threshold for obstacle detection in mm
            min_angle (int): Minimum angle to check (0-359)
            max_angle (int): Maximum angle to check (0-359)
            min_points (int): Minimum consecutive points to consider as obstacle
            filter_range (bool): Whether to filter out spurious points
            filter_window (int): Window size for median filtering
            
        Returns:
            list: List of tuples (start_angle, end_angle, min_distance, center_angle)
        """
        # Get filtered distances
        distances = self.get_obstacle_distances()
        
        # Apply median filter if requested
        if filter_range and filter_window > 0:
            filtered = []
            half_window = filter_window // 2
            
            for i in range(360):
                window = []
                for j in range(-half_window, half_window + 1):
                    idx = (i + j) % 360
                    if distances[idx] > 0:  # Only include valid distances
                        window.append(distances[idx])
                
                if window:
                    filtered.append(np.median(window))
                else:
                    filtered.append(0)
            
            distances = np.array(filtered)
        
        # Normalize angles to the 0-359 range
        min_angle = min_angle % 360
        max_angle = max_angle % 360
        
        # Convert to range array to analyze
        angle_range = list(range(min_angle, max_angle if max_angle > min_angle else max_angle + 360))
        angle_range = [a % 360 for a in angle_range]
        
        # Detect obstacle clusters
        obstacles = []
        in_obstacle = False
        start_angle = None
        obstacle_points = []
        
        for angle in angle_range:
            distance = distances[angle]
            
            # Check if this point is an obstacle
            is_obstacle_point = (0 < distance <= threshold_distance)
            
            if is_obstacle_point and not in_obstacle:
                # Start of new obstacle
                in_obstacle = True
                start_angle = angle
                obstacle_points = [distance]
            elif is_obstacle_point and in_obstacle:
                # Continuation of obstacle
                obstacle_points.append(distance)
            elif not is_obstacle_point and in_obstacle:
                # End of obstacle
                in_obstacle = False
                
                # Only add if it meets minimum size
                if len(obstacle_points) >= min_points:
                    end_angle = (angle - 1) % 360
                    min_distance = min(obstacle_points)
                    center_angle = (start_angle + ((end_angle - start_angle) % 360) // 2) % 360
                    
                    obstacles.append((start_angle, end_angle, min_distance, center_angle))
                
                # Reset for next obstacle
                start_angle = None
                obstacle_points = []
        
        # Handle case where obstacle extends to the end of the range
        if in_obstacle and len(obstacle_points) >= min_points:
            end_angle = angle_range[-1]
            min_distance = min(obstacle_points)
            center_angle = (start_angle + ((end_angle - start_angle) % 360) // 2) % 360
            
            obstacles.append((start_angle, end_angle, min_distance, center_angle))
        
        return obstacles
    
    def get_closest_obstacle(self, min_angle=0, max_angle=360, min_distance=100):
        """
        Get the closest obstacle in the specified angle range
        
        Args:
            min_angle (int): Minimum angle to check (0-359)
            max_angle (int): Maximum angle to check (0-359)
            min_distance (float): Minimum valid distance in mm
            
        Returns:
            tuple: (angle, distance) of closest obstacle, or (None, None) if no obstacle found
        """
        # Get filtered distances
        distances = self.get_obstacle_distances(min_distance=min_distance)
        
        # Normalize angles to the 0-359 range
        min_angle = min_angle % 360
        max_angle = max_angle % 360
        
        # Convert to range array to analyze
        angle_range = list(range(min_angle, max_angle if max_angle > min_angle else max_angle + 360))
        angle_range = [a % 360 for a in angle_range]
        
        # Find closest valid distance
        closest_distance = float('inf')
        closest_angle = None
        
        for angle in angle_range:
            distance = distances[angle]
            
            if distance > 0 and distance < closest_distance:
                closest_distance = distance
                closest_angle = angle
        
        if closest_angle is not None:
            return (closest_angle, closest_distance)
        else:
            return (None, None)

# Example usage
def main():
    # Initialize RPLidar object - adjust port as needed
    lidar = RPLidar(port='/dev/ttyUSB0')
    
    try:
        # Connect to the device
        if not lidar.connect():
            print("Failed to connect to RPLiDAR")
            return
        
        # Get device info
        info = lidar.get_info()
        print(f"RPLiDAR Info: {info}")
        
        # Get health status
        health, error_code = lidar.get_health()
        print(f"RPLiDAR Health: {health}, Error Code: {error_code}")
        
        # Reset the device
        lidar.reset()
        time.sleep(1)
        
        # Start scanning
        if not lidar.start_scan():
            print("Failed to start scan")
            lidar.disconnect()
            return
        
        print("Scanning started, press Ctrl+C to stop")
        
        # Show obstacle information for a while
        for _ in range(30):  # Run for 30 iterations
            obstacles = lidar.detect_obstacles(threshold_distance=1000)
            closest_angle, closest_distance = lidar.get_closest_obstacle()
            
            # Print obstacles
            print(f"\nDetected {len(obstacles)} obstacles:")
            for i, (start, end, dist, center) in enumerate(obstacles):
                print(f"  Obstacle {i+1}: Angle {start}° to {end}° (center: {center}°), Distance: {dist:.0f}mm")
            
            # Print closest obstacle
            if closest_angle is not None:
                print(f"Closest obstacle: Angle {closest_angle}°, Distance: {closest_distance:.0f}mm")
            else:
                print("No obstacles detected")
                
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Clean up
        lidar.stop_scan()
        lidar.disconnect()
        print("Done")

if __name__ == "__main__":
    main() 