#!/usr/bin/env python3
# Simple serial test for RPLiDAR
# This tests direct serial communication

import serial
import time
import binascii

def main():
    # Connect to the serial port
    port = '/dev/ttyUSB0'  # Change this if needed
    baudrate = 115200
    
    print(f"Attempting to connect to {port} at {baudrate} baud...")
    
    try:
        # Open the serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        if ser.is_open:
            print(f"Serial port {port} opened successfully")
        else:
            print("Failed to open port")
            return
        
        # Start the motor (DTR low)
        print("Starting motor (setting DTR low)...")
        ser.dtr = False
        time.sleep(2)
        print("Motor should be spinning now")
        
        # Send stop command to clear any existing scan operation
        print("\nSending STOP command...")
        ser.write(b'\xA5\x25')
        time.sleep(0.5)
        
        # Flush input buffer
        ser.reset_input_buffer()
        
        # Send command to get device info
        print("\nSending GET_INFO command...")
        ser.write(b'\xA5\x50\x00')
        time.sleep(0.5)
        
        # Read response
        print("Reading response...")
        response = ser.read(100)  # Try to read a lot of bytes
        
        print(f"Received {len(response)} bytes:")
        if len(response) > 0:
            hex_data = binascii.hexlify(response).decode('utf-8')
            print(f"HEX: {hex_data}")
            
            # Check if it looks like a valid response
            if len(response) >= 7 and response[0] == 0xA5 and response[1] == 0x5A:
                print("Valid response descriptor found!")
            else:
                print("No valid response descriptor")
        else:
            print("No data received")
        
        # Send scan command
        print("\nSending SCAN command...")
        ser.write(b'\xA5\x20\x00')
        time.sleep(0.5)
        
        # Read scan response
        print("Reading scan response...")
        scan_resp = ser.read(100)
        
        print(f"Received {len(scan_resp)} bytes:")
        if len(scan_resp) > 0:
            hex_scan = binascii.hexlify(scan_resp).decode('utf-8')
            print(f"HEX: {hex_scan}")
            
            # Check if it looks like a valid scan response
            if len(scan_resp) >= 7 and scan_resp[0] == 0xA5 and scan_resp[1] == 0x5A:
                print("Valid scan response descriptor found!")
                
                # Try to read some actual scan data packets
                print("\nReading scan data packets...")
                for i in range(10):  # Try to read 10 packets
                    packet = ser.read(5)  # Each packet is 5 bytes
                    if len(packet) == 5:
                        hex_packet = binascii.hexlify(packet).decode('utf-8')
                        print(f"Packet {i+1}: {hex_packet}")
                        
                        # Check if first byte has the check bit (bit 0) set
                        if packet[0] & 0x01:
                            # Parse packet
                            quality = packet[0] >> 2  # First 6 bits
                            angle = ((packet[2] << 8) | packet[1]) / 64.0  # Next 16 bits
                            distance = ((packet[4] << 8) | packet[3]) / 4.0  # Last 16 bits
                            print(f"  Quality: {quality}, Angle: {angle:.2f}°, Distance: {distance:.2f}mm")
                        else:
                            print("  Invalid packet (check bit not set)")
                    else:
                        print(f"  Received incomplete packet: {len(packet)} bytes")
            else:
                print("No valid scan response descriptor")
        else:
            print("No scan response received")
            
        # Stop the scan
        print("\nSending STOP command...")
        ser.write(b'\xA5\x25')
        time.sleep(0.5)
        
        # Stop the motor (DTR high)
        print("Stopping motor (setting DTR high)...")
        ser.dtr = True
        time.sleep(1)
        
        # Close serial port
        ser.close()
        print("Serial port closed")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 