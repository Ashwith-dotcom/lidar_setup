#!/usr/bin/env python3
# Simplified RPLiDAR A1M8 Driver
# Focuses only on basic scanning functionality

import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimpleLidar:
    """
    Simplified class for RPLiDAR A1M8
    """
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.distances = np.zeros(360)  # Store distances by angle (0-359)
        self.angles = []  # Raw angles from measurements
        self.raw_distances = []  # Raw distances from measurements
    
    def connect(self):
        """Connect to the LIDAR device"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            return self.serial.is_open
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the LIDAR device"""
        if self.running:
            self.stop()
        
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def start_motor(self):
        """Start the LIDAR motor"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            # Set DTR to LOW to start the motor
            self.serial.dtr = False
            time.sleep(0.5)  # Give motor time to start
            return True
        except Exception as e:
            print(f"Failed to start motor: {e}")
            return False
    
    def stop_motor(self):
        """Stop the LIDAR motor"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            # Set DTR to HIGH to stop the motor
            self.serial.dtr = True
            time.sleep(0.5)  # Give motor time to stop
            return True
        except Exception as e:
            print(f"Failed to stop motor: {e}")
            return False
    
    def _send_cmd(self, cmd):
        """Send a command byte to LIDAR"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            self.serial.write(b'\xA5' + cmd + b'\x00')
            time.sleep(0.1)  # Short delay after command
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def start(self):
        """Start LIDAR scanning"""
        if self.running:
            return True
        
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False
        
        # Start motor
        if not self.start_motor():
            return False
        
        # Reset input buffer
        self.serial.reset_input_buffer()
        
        # Send stop command to ensure no scan is running
        self._send_cmd(b'\x25')
        time.sleep(0.5)
        
        # Send scan command
        if not self._send_cmd(b'\x20'):
            self.stop_motor()
            return False
        
        # Read response descriptor
        descriptor = self.serial.read(7)
        if len(descriptor) < 7:
            print("Failed to get scan descriptor")
            self.stop_motor()
            return False
        
        # Check if response is valid
        if descriptor[0] != 0xA5 or descriptor[1] != 0x5A or descriptor[2] != 0x05:
            print("Invalid scan descriptor")
            self.stop_motor()
            return False
        
        self.running = True
        print("Scan started successfully")
        return True
    
    def stop(self):
        """Stop LIDAR scanning"""
        if not self.running:
            return True
        
        self.running = False
        
        # Send stop command
        success = self._send_cmd(b'\x25')
        
        # Stop motor
        self.stop_motor()
        
        return success
    
    def get_data(self, num_samples=360, timeout=5):
        """
        Get LIDAR scan data
        
        Args:
            num_samples (int): Number of data samples to collect
            timeout (int): Timeout in seconds
            
        Returns:
            bool: True if data collected successfully
        """
        if not self.running:
            if not self.start():
                return False
        
        # Clear previous data
        self.angles = []
        self.raw_distances = []
        self.distances = np.zeros(360)
        
        samples = 0
        start_time = time.time()
        
        # Read data packets
        while samples < num_samples and (time.time() - start_time) < timeout:
            try:
                # Read one data packet (5 bytes)
                data = self.serial.read(5)
                
                if len(data) != 5:
                    continue
                
                # Check if first byte has check bit (bit 0) set
                if not (data[0] & 0x01):
                    continue
                
                # Parse data
                quality = data[0] >> 2  # First 6 bits
                angle = ((data[2] << 8) | data[1]) / 64.0  # Next 16 bits
                distance = ((data[4] << 8) | data[3]) / 4.0  # Last 16 bits
                
                # Skip invalid data
                if distance == 0:
                    continue
                
                # Store raw data
                self.angles.append(angle)
                self.raw_distances.append(distance)
                
                # Update the distances array
                angle_index = int(angle) % 360
                self.distances[angle_index] = distance
                
                samples += 1
                
            except Exception as e:
                print(f"Error reading data: {e}")
        
        print(f"Collected {samples} samples in {time.time() - start_time:.1f} seconds")
        return samples > 0
    
    def plot_data(self):
        """Plot the LIDAR data as a polar chart"""
        if len(self.angles) == 0:
            print("No data to plot")
            return
        
        # Convert to radians for plotting
        angles_rad = np.radians(self.angles)
        
        # Create figure
        plt.figure(figsize=(10, 8))
        ax = plt.subplot(111, projection='polar')
        
        # Plot the data points
        ax.scatter(angles_rad, self.raw_distances, s=5)
        
        # Set limits and labels
        ax.set_theta_zero_location('N')  # 0 degrees at top (North)
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_title('RPLiDAR A1M8 Scan')
        ax.set_rlabel_position(0)  # Move radial labels to 0 degrees
        
        # Add range rings
        max_range = max(self.raw_distances) * 1.1
        ax.set_rticks(np.linspace(0, max_range, 5))
        
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    
    def plot_realtime(self, max_range=6000, interval=100, duration=60):
        """
        Show real-time plot of LIDAR data
        
        Args:
            max_range (int): Maximum range to display in mm
            interval (int): Update interval in ms
            duration (int): Duration in seconds (approximate)
        """
        if not self.running:
            if not self.start():
                return
        
        # Create figure
        fig = plt.figure(figsize=(10, 8))
        ax = plt.subplot(111, projection='polar')
        
        # Initialize empty scatter plot
        scatter = ax.scatter([], [], s=5)
        
        # Set limits and labels
        ax.set_theta_zero_location('N')  # 0 degrees at top (North)
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_rlim(0, max_range)
        ax.set_title('RPLiDAR A1M8 Real-Time Scan')
        ax.grid(True)
        
        # Animation update function
        def update(frame):
            # Read some data
            self.get_data(num_samples=90, timeout=interval/1000)
            
            if len(self.angles) > 0:
                # Convert to radians for plotting
                angles_rad = np.radians(self.angles)
                
                # Update scatter plot
                scatter.set_offsets(np.column_stack([angles_rad, self.raw_distances]))
                
                # Update title with point count
                ax.set_title(f'RPLiDAR A1M8 - {len(self.angles)} points')
            
            return scatter,
        
        # Create animation
        ani = FuncAnimation(fig, update, frames=range(int(duration*1000/interval)), 
                           interval=interval, blit=True)
        
        plt.tight_layout()
        plt.show()

def main():
    # Create lidar object
    lidar = SimpleLidar('/dev/ttyUSB0')
    
    try:
        # Connect to LIDAR
        if not lidar.connect():
            print("Failed to connect to LIDAR")
            return
        
        print("Connected to LIDAR")
        
        # Test basic scan
        print("\nCollecting scan data...")
        if lidar.get_data(num_samples=720, timeout=10):
            print(f"Got {len(lidar.angles)} data points")
            
            # Check if we have enough data
            if len(lidar.angles) > 100:
                # Plot the data
                print("Plotting data...")
                lidar.plot_data()
            else:
                print("Not enough data points for plotting")
            
            # Show real-time plot
            print("\nStarting real-time display (ctrl+c to stop)...")
            lidar.plot_realtime(duration=30)  # Run for 30 seconds
            
        else:
            print("Failed to get data")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up
        lidar.stop()
        lidar.disconnect()
        print("Done")

if __name__ == "__main__":
    main() 