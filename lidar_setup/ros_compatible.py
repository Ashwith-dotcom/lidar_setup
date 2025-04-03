#!/usr/bin/env python3
# ROS-compatible RPLiDAR driver
# Can be used standalone or with ROS

import serial
import time
import math
import json
import numpy as np
import threading
from datetime import datetime
import signal
import sys

# RPLiDAR Constants
LIDAR_START_FLAG = b'\xA5'
LIDAR_STOP_BYTE = b'\x25'
LIDAR_RESET_BYTE = b'\x40'
LIDAR_SCAN_BYTE = b'\x20'
LIDAR_FORCE_SCAN_BYTE = b'\x21'
LIDAR_INFO_BYTE = b'\x50'
LIDAR_HEALTH_BYTE = b'\x52'

class ROSLidar:
    """
    RPLiDAR driver that can be used with or without ROS
    """
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, use_force_scan=True, frame_id="lidar"):
        self.port = port
        self.baudrate = baudrate
        self.use_force_scan = use_force_scan  # Force scan tends to be more reliable
        self.frame_id = frame_id
        
        # Serial connection
        self.serial = None
        
        # Scan state
        self.running = False
        self.scan_thread = None
        self.stop_event = threading.Event()
        
        # Latest scan data
        self.scan_data = {
            'angles': [],
            'distances': [],
            'intensities': [],
            'timestamp': 0
        }
        
        # Full 360° data
        self.distances = np.zeros(360)
        self.intensities = np.zeros(360)
        
        # Data lock
        self.data_lock = threading.Lock()
        
        # For statistics
        self.points_per_scan = 0
        self.scan_count = 0
        self.last_scan_time = 0
        self.scan_duration = 0
    
    def connect(self):
        """Connect to LIDAR device"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            
            if self.serial.is_open:
                print(f"Connected to RPLiDAR on {self.port}")
                return True
                
            return False
        except Exception as e:
            print(f"Failed to connect to RPLiDAR: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from LIDAR device"""
        if self.running:
            self.stop_scan()
            
        if self.serial and self.serial.is_open:
            # Stop motor
            self.serial.dtr = True
            time.sleep(0.5)
            
            # Close port
            self.serial.close()
            print("Disconnected from RPLiDAR")
    
    def start_motor(self):
        """Start LIDAR motor"""
        if not self.serial or not self.serial.is_open:
            return False
            
        try:
            # Set DTR to LOW to start motor
            self.serial.dtr = False
            time.sleep(0.5)  # Wait for motor to start
            return True
        except Exception as e:
            print(f"Failed to start motor: {e}")
            return False
    
    def stop_motor(self):
        """Stop LIDAR motor"""
        if not self.serial or not self.serial.is_open:
            return False
            
        try:
            # Set DTR to HIGH to stop motor
            self.serial.dtr = True
            time.sleep(0.5)  # Wait for motor to stop
            return True
        except Exception as e:
            print(f"Failed to stop motor: {e}")
            return False
    
    def _send_cmd(self, cmd, payload=None):
        """Send command to LIDAR"""
        if not self.serial or not self.serial.is_open:
            return False
            
        try:
            # Reset input buffer
            self.serial.reset_input_buffer()
            
            # Construct command
            packet = LIDAR_START_FLAG + cmd
            
            if payload:
                size = len(payload)
                packet += bytes([size]) + payload
            else:
                packet += b'\x00'  # Empty payload
                
            # Send command
            self.serial.write(packet)
            time.sleep(0.1)  # Short delay
            
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def start_scan(self):
        """Start LIDAR scanning"""
        if self.running:
            return True
            
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False
        
        # Reset the stop event
        self.stop_event.clear()
        
        # Start motor
        if not self.start_motor():
            return False
            
        # Stop any existing scan
        self._send_cmd(LIDAR_STOP_BYTE)
        time.sleep(0.5)
            
        # Choose scan command based on configuration
        scan_command = LIDAR_FORCE_SCAN_BYTE if self.use_force_scan else LIDAR_SCAN_BYTE
            
        # Send scan command
        if not self._send_cmd(scan_command):
            self.stop_motor()
            return False
            
        # Read response descriptor
        descriptor = self.serial.read(7)
        
        if len(descriptor) < 7:
            print("Failed to get scan response descriptor")
            self.stop_motor()
            return False
            
        # Check descriptor
        if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            print("Invalid scan response descriptor")
            self.stop_motor()
            return False
            
        # Start scan thread
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_thread)
        self.scan_thread.daemon = True
        self.scan_thread.start()
        
        print("RPLiDAR scan started")
        return True
    
    def stop_scan(self):
        """Stop LIDAR scanning"""
        if not self.running:
            return True
            
        # Signal thread to stop
        self.stop_event.set()
        
        # Wait for thread to end
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(2)
            
        # Send stop command
        self._send_cmd(LIDAR_STOP_BYTE)
        
        # Stop motor
        self.stop_motor()
        
        self.running = False
        print("RPLiDAR scan stopped")
        
        return True
    
    def reset(self):
        """Reset the LIDAR device"""
        if self.running:
            self.stop_scan()
            
        success = self._send_cmd(LIDAR_RESET_BYTE)
        time.sleep(2)  # Wait for device to reset
        
        return success
    
    def _scan_thread(self):
        """Thread function for continuous scanning"""
        # For measuring scan rate
        scan_start_time = time.time()
        points_in_scan = 0
        
        # Main scan loop
        while self.running and not self.stop_event.is_set():
            try:
                # Read one data packet (5 bytes)
                data = self.serial.read(5)
                
                if len(data) != 5:
                    continue
                    
                # Check if first byte has check bit set
                if not (data[0] & 0x01):
                    continue
                    
                # Parse data
                quality = data[0] >> 2  # First 6 bits
                angle = ((data[2] << 8) | data[1]) / 64.0  # Next 16 bits
                distance = ((data[4] << 8) | data[3]) / 4.0  # Last 16 bits
                
                # Skip invalid data
                if distance == 0:
                    continue
                    
                # Update data arrays
                with self.data_lock:
                    # Add to raw scan data
                    self.scan_data['angles'].append(angle)
                    self.scan_data['distances'].append(distance)
                    self.scan_data['intensities'].append(quality)
                    
                    # Update timestamp
                    self.scan_data['timestamp'] = time.time()
                    
                    # Update indexed arrays
                    angle_index = int(angle) % 360
                    self.distances[angle_index] = distance
                    self.intensities[angle_index] = quality
                    
                    points_in_scan += 1
                
                # Check if we've completed a full scan (360 degrees)
                # This is an approximation - a better approach would track start/end angles
                if points_in_scan >= 360:
                    # Calculate scan statistics
                    now = time.time()
                    self.scan_duration = now - scan_start_time
                    self.points_per_scan = points_in_scan
                    self.scan_count += 1
                    self.last_scan_time = now
                    
                    # Start new scan count
                    scan_start_time = now
                    points_in_scan = 0
                    
                    # Clear scan data for next scan while preserving arrays
                    with self.data_lock:
                        self.scan_data['angles'] = []
                        self.scan_data['distances'] = []
                        self.scan_data['intensities'] = []
                
            except Exception as e:
                print(f"Error in scan thread: {e}")
                time.sleep(0.01)
    
    def get_scan_msg(self):
        """
        Get scan data in a format similar to ROS LaserScan message
        This can be directly converted to ROS message or used standalone
        """
        with self.data_lock:
            # Create a copy of distances array to avoid thread conflicts
            distances = self.distances.copy()
            
            # Basic metadata
            scan_msg = {
                "header": {
                    "frame_id": self.frame_id,
                    "stamp": time.time()
                },
                "angle_min": 0.0,
                "angle_max": 2*math.pi,
                "angle_increment": (2*math.pi) / 360,
                "time_increment": self.scan_duration / 360 if self.scan_duration > 0 else 0.0,
                "scan_time": self.scan_duration,
                "range_min": 0.0,
                "range_max": 6000.0,  # 6 meters max range for A1M8
                "ranges": distances.tolist(),
                "intensities": self.intensities.tolist()
            }
            
            return scan_msg
    
    def get_closest_obstacle(self, min_distance=100, max_distance=6000, min_angle=0, max_angle=360):
        """Get the closest obstacle in the specified angle range"""
        with self.data_lock:
            # Get distance data within angle range
            if min_angle < max_angle:
                angle_range = range(min_angle, max_angle)
            else:
                angle_range = list(range(min_angle, 360)) + list(range(0, max_angle))
                
            # Find closest valid point
            closest_distance = float('inf')
            closest_angle = None
            
            for angle in angle_range:
                distance = self.distances[angle]
                
                if min_distance < distance < max_distance and distance < closest_distance:
                    closest_distance = distance
                    closest_angle = angle
            
            if closest_angle is not None:
                return (closest_angle, closest_distance)
            else:
                return (None, None)
    
    def get_scan_as_json(self):
        """Get scan data as JSON string"""
        scan_msg = self.get_scan_msg()
        return json.dumps(scan_msg)
    
    def get_stats(self):
        """Get scanning statistics"""
        return {
            "points_per_scan": self.points_per_scan,
            "scan_count": self.scan_count,
            "scan_duration": self.scan_duration,
            "scan_rate": 1.0 / self.scan_duration if self.scan_duration > 0 else 0.0,
            "points_rate": self.points_per_scan / self.scan_duration if self.scan_duration > 0 else 0.0,
            "last_scan_time": self.last_scan_time
        }

def print_stats(lidar):
    """Print scan statistics periodically"""
    while lidar.running:
        stats = lidar.get_stats()
        closest_angle, closest_distance = lidar.get_closest_obstacle()
        
        print("\033[H\033[J")  # Clear terminal
        print("RPLiDAR Statistics:")
        print(f"Points per scan: {stats['points_per_scan']}")
        print(f"Scan rate: {stats['scan_rate']:.2f} Hz")
        print(f"Point rate: {stats['points_rate']:.2f} points/sec")
        print(f"Total scans: {stats['scan_count']}")
        
        if closest_angle is not None:
            print(f"\nClosest obstacle: {closest_distance:.0f}mm at {closest_angle}°")
        else:
            print("\nNo obstacles detected")
            
        # Print some sample range data
        print("\nSample distances (degrees: mm):")
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            print(f"  {angle}°: {lidar.distances[angle]:.0f}mm")
            
        time.sleep(1)

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nStopping...")
    if 'lidar' in globals():
        lidar.stop_scan()
        lidar.disconnect()
    sys.exit(0)

def main():
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Flush stdin to avoid buffered input
    sys.stdin.flush()
    
    # Create LIDAR object
    global lidar
    lidar = ROSLidar(port='/dev/ttyUSB0', use_force_scan=True)
    
    try:
        # Connect to LIDAR
        if not lidar.connect():
            print("Failed to connect to LIDAR")
            return
            
        print("Connected to LIDAR")
        
        # Start scanning
        if not lidar.start_scan():
            print("Failed to start scan")
            lidar.disconnect()
            return
            
        print("Scan started, collecting data...")
        
        # Wait for initial data
        time.sleep(2)
        
        # Print statistics in a loop until interrupted
        print_stats(lidar)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up
        if 'lidar' in locals():
            lidar.stop_scan()
            lidar.disconnect()
        print("Done")

if __name__ == "__main__":
    main() 