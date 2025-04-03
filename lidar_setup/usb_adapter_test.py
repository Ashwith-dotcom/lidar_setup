#!/usr/bin/env python3
# USB-to-UART Adapter Test Script for RPLiDAR
# Tests if your USB adapter properly supports DTR pin control for motor operation

import serial
import time
import sys

def test_dtr_control(port):
    """
    Test if the USB-to-UART adapter supports DTR pin control
    Returns True if successful, False otherwise
    """
    try:
        print(f"Opening serial port {port}...")
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        if not ser.is_open:
            print("Failed to open port")
            return False
            
        print("Serial port opened successfully")
        
        # Get initial DTR state
        initial_dtr = ser.dtr
        print(f"Initial DTR state: {'LOW' if initial_dtr is False else 'HIGH'}")
        
        # Test DTR control
        print("\nTesting DTR control (motor control)...")
        print("Setting DTR LOW (motor ON)...")
        ser.dtr = False
        time.sleep(1)
        
        # Check if DTR state changed
        if ser.dtr is False:
            print("SUCCESS: DTR is LOW")
        else:
            print("ERROR: Failed to set DTR LOW")
            ser.close()
            return False
            
        print("\nMotor should be spinning now.")
        print("Do you see the RPLiDAR motor spinning? (y/n)")
        response = input().strip().lower()
        
        if response != 'y':
            print("Motor control test failed - motor is not spinning")
            print("Possible causes:")
            print("1. USB adapter doesn't properly support DTR control")
            print("2. RPLiDAR is not getting enough power")
            print("3. Connection issue between RPLiDAR motor control and USB adapter DTR")
            motor_spinning = False
        else:
            print("Motor is spinning - DTR LOW control works!")
            motor_spinning = True
            
        # Test turning off motor
        print("\nSetting DTR HIGH (motor OFF)...")
        ser.dtr = True
        time.sleep(1)
        
        # Check if DTR state changed
        if ser.dtr is True:
            print("SUCCESS: DTR is HIGH")
        else:
            print("ERROR: Failed to set DTR HIGH")
            ser.close()
            return False
            
        print("\nMotor should be stopped now.")
        print("Has the RPLiDAR motor stopped spinning? (y/n)")
        response = input().strip().lower()
        
        if response != 'y':
            print("Motor control test failed - motor is still spinning")
            print("Possible causes:")
            print("1. USB adapter doesn't properly support DTR control")
            print("2. Connection issue between RPLiDAR motor control and USB adapter DTR")
            motor_stopping = False
        else:
            print("Motor has stopped - DTR HIGH control works!")
            motor_stopping = True
            
        # Close serial port
        ser.close()
        print("Serial port closed")
        
        # Return overall result
        return motor_spinning and motor_stopping
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("======================================")
    print("RPLiDAR USB-to-UART Adapter Test Tool")
    print("======================================")
    print("\nThis tool tests if your USB-to-UART adapter")
    print("properly supports DTR pin control for the RPLiDAR motor.")
    print("\nMake sure your RPLiDAR is connected and powered.")
    
    # Get port from command line argument or ask user
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        print("\nEnter the serial port your RPLiDAR is connected to")
        print("(e.g., /dev/ttyUSB0 on Linux, COM3 on Windows):")
        port = input().strip()
    
    # Run the test
    print(f"\nTesting adapter on port {port}...")
    result = test_dtr_control(port)
    
    # Show final result
    print("\n======================================")
    if result:
        print("TEST PASSED: Your adapter supports DTR control!")
        print("This adapter should work with RPLiDAR motor control.")
    else:
        print("TEST FAILED: DTR control issues detected.")
        print("\nPossible solutions:")
        print("1. Try a different USB-to-UART adapter that supports DTR")
        print("2. Check connections between RPLiDAR and adapter")
        print("3. Ensure RPLiDAR has adequate power (5V/1A)")
        print("4. Some CH340 adapters have issues with DTR control - try a CP2102 or FTDI adapter")
    print("======================================")

if __name__ == "__main__":
    main() 