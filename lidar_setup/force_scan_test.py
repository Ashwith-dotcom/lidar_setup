#!/usr/bin/env python3
# Force Scan Test for RPLiDAR A1M8
# Uses FORCE_SCAN command which can be more reliable

import serial
import time
import struct
import binascii
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Connection settings
    port = '/dev/ttyUSB0'  # Change this if needed
    baudrate = 115200
    
    # Measurement settings
    timeout = 10  # seconds to collect data
    min_samples = 100  # minimum samples for a useful scan
    
    try:
        print(f"Connecting to RPLiDAR on {port}...")
        
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        if not ser.is_open:
            print("Failed to open port")
            return
        
        print("Serial port opened successfully")
        
        # Start motor
        print("Starting motor...")
        ser.dtr = False
        time.sleep(1)
        print("Motor should be spinning now")
        
        # Clear any existing commands
        print("Stopping any existing scan...")
        ser.write(b'\xA5\x25')
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        # Send FORCE_SCAN command
        print("Sending FORCE_SCAN command...")
        ser.write(b'\xA5\x21\x00')
        time.sleep(0.5)
        
        # Read response descriptor
        descriptor = ser.read(7)
        print(f"Read descriptor: {binascii.hexlify(descriptor).decode('utf-8') if descriptor else 'None'}")
        
        if len(descriptor) < 7:
            print("Failed to get response descriptor")
            ser.dtr = True  # Stop motor
            ser.close()
            return
        
        # Check descriptor
        if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            print("Invalid response descriptor")
            ser.dtr = True  # Stop motor
            ser.close()
            return
        
        print("Valid response descriptor received, starting data collection...")
        
        # Storage for scan data
        angles = []
        distances = []
        qualities = []
        
        # Collect data for some time
        start_time = time.time()
        read_errors = 0
        max_errors = 10
        
        while (time.time() - start_time) < timeout and read_errors < max_errors:
            try:
                # Each data point is 5 bytes: [quality, angle_low, angle_high, dist_low, dist_high]
                data = ser.read(5)
                
                if len(data) != 5:
                    read_errors += 1
                    print(f"Incomplete data packet ({len(data)} bytes)")
                    continue
                
                # Check if first byte has check bit (bit 0) set
                if not (data[0] & 0x01):
                    read_errors += 1
                    print("Invalid data packet (check bit not set)")
                    continue
                
                # Parse data
                quality = data[0] >> 2  # First 6 bits
                angle = ((data[2] << 8) | data[1]) / 64.0  # Next 16 bits
                distance = ((data[4] << 8) | data[3]) / 4.0  # Last 16 bits
                
                # Only record valid distances
                if distance > 0:
                    angles.append(angle)
                    distances.append(distance)
                    qualities.append(quality)
                
                # Print sample point
                if len(angles) % 50 == 0:
                    print(f"Got {len(angles)} points. Latest: Angle={angle:.2f}°, Distance={distance:.2f}mm, Quality={quality}")
                
                read_errors = 0  # Reset error count on successful read
                
            except Exception as e:
                read_errors += 1
                print(f"Error reading data: {e}")
        
        # Stop scanning
        print("\nStopping scan...")
        ser.write(b'\xA5\x25')
        time.sleep(0.5)
        
        # Stop motor
        print("Stopping motor...")
        ser.dtr = True
        time.sleep(0.5)
        
        # Close serial port
        ser.close()
        print("Serial port closed")
        
        # Show results
        print(f"\nTotal data points collected: {len(angles)}")
        
        if len(angles) < min_samples:
            print("Not enough data points for visualization")
            return
        
        # Convert to numpy arrays for processing
        angles = np.array(angles)
        distances = np.array(distances)
        
        # Create polar plot
        plt.figure(figsize=(10, 8))
        ax = plt.subplot(111, projection='polar')
        
        # Convert to radians for matplotlib
        angles_rad = np.radians(angles)
        
        # Plot scatter
        sc = ax.scatter(angles_rad, distances, s=5, c=distances, cmap='viridis')
        
        # Set up the polar plot
        ax.set_theta_zero_location('N')  # 0 degrees at top
        ax.set_theta_direction(-1)  # Clockwise
        
        # Add color bar
        plt.colorbar(sc, label='Distance (mm)')
        
        # Set title and labels
        ax.set_title(f'RPLiDAR A1M8 Force Scan ({len(angles)} points)')
        
        # Add grid
        plt.grid(True)
        
        # Show the plot
        plt.tight_layout()
        plt.show()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Operation interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Make sure to close serial port if it's open
        if 'ser' in locals() and ser.is_open:
            # Stop motor before closing
            ser.dtr = True
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main() 