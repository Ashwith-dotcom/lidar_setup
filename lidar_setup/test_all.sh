#!/bin/bash
# RPLiDAR A1M8 Test Script
# Runs various test programs to identify which works best with your hardware

# ANSI color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}RPLiDAR A1M8 Testing Script${NC}"
echo -e "${BLUE}====================================${NC}"
echo

# Check for required packages
echo -e "${YELLOW}Checking required packages...${NC}"
if ! pip3 list | grep -q "pyserial"; then
    echo -e "${RED}pyserial not found. Installing...${NC}"
    pip3 install pyserial
fi

if ! pip3 list | grep -q "numpy"; then
    echo -e "${RED}numpy not found. Installing...${NC}"
    pip3 install numpy
fi

if ! pip3 list | grep -q "matplotlib"; then
    echo -e "${RED}matplotlib not found. Installing...${NC}"
    pip3 install matplotlib
fi

echo -e "${GREEN}All required packages are installed.${NC}"
echo

# Make all Python scripts executable
chmod +x *.py

# Function to test serial port
test_serial_port() {
    local port=$1
    echo -e "${YELLOW}Testing connection to port $port...${NC}"
    
    # Try to open the port
    if [ -e "$port" ]; then
        echo -e "${GREEN}Port $port exists.${NC}"
        
        # Check if port is accessible
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo -e "${GREEN}Port $port is readable and writable.${NC}"
            return 0
        else
            echo -e "${RED}Port $port exists but permission denied.${NC}"
            echo -e "${YELLOW}Trying to fix permissions...${NC}"
            sudo chmod a+rw $port
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}Permissions fixed for $port.${NC}"
                return 0
            else
                echo -e "${RED}Failed to fix permissions for $port.${NC}"
                return 1
            fi
        fi
    else
        echo -e "${RED}Port $port does not exist.${NC}"
        return 1
    fi
}

# Try to find RPLiDAR port
echo -e "${YELLOW}Looking for RPLiDAR device...${NC}"
RPLIDAR_PORT=""

# List of ports to try
PORTS=("/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyACM0" "/dev/ttyACM1" "/dev/ttyS0")

for port in "${PORTS[@]}"; do
    if [ -e "$port" ]; then
        echo -e "${GREEN}Found potential device at $port${NC}"
        if test_serial_port "$port"; then
            RPLIDAR_PORT=$port
            break
        fi
    fi
done

if [ -z "$RPLIDAR_PORT" ]; then
    echo -e "${RED}No RPLiDAR device found. Please connect the device and try again.${NC}"
    echo -e "${YELLOW}Available ports:${NC}"
    ls -l /dev/tty*
    
    echo
    echo -e "${YELLOW}Would you like to manually specify a port? (y/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        echo -e "${YELLOW}Enter the port (e.g., /dev/ttyUSB0):${NC}"
        read -r RPLIDAR_PORT
        if [ ! -e "$RPLIDAR_PORT" ]; then
            echo -e "${RED}Port $RPLIDAR_PORT does not exist. Exiting.${NC}"
            exit 1
        fi
        test_serial_port "$RPLIDAR_PORT"
    else
        echo -e "${RED}Exiting.${NC}"
        exit 1
    fi
fi

echo
echo -e "${GREEN}Using port: $RPLIDAR_PORT${NC}"
echo

# Update port in Python scripts
echo -e "${YELLOW}Updating port in test scripts...${NC}"
find . -name "*.py" -type f -exec sed -i "s|/dev/ttyUSB0|$RPLIDAR_PORT|g" {} \;
echo -e "${GREEN}Updated port in all Python scripts.${NC}"
echo

cat << "EOF"
################################################
#                                              #
#    RPLiDAR A1M8 IMPLEMENTATION TESTS         #
#                                              #
################################################

IMPORTANT: We've discovered that the most critical 
timing issue with the RPLiDAR A1M8 is waiting for 
the motor to reach proper speed (about 2.5 seconds)
before attempting to scan. All implementations 
have been updated with this fix.

EOF

# Run the basic serial test first
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Running Basic Serial Test${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This test will check basic communication with the RPLiDAR.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test and continue to the next one.${NC}"
echo
read -p "Press Enter to start the test..."
python3 serial_test.py

# Add timing check
echo
echo "Checking for critical timing in implementations..."
echo

# Check rplidar_driver.py for proper timing
TIMING_CHECK=$(grep -n "time.sleep(2.5)" rplidar_driver.py | wc -l)
if [ $TIMING_CHECK -ge 1 ]; then
    echo "✓ rplidar_driver.py includes proper motor timing"
else
    echo "⚠ rplidar_driver.py may have timing issues - not using 2.5s delay"
fi

# Check simple_lidar.py for proper timing
TIMING_CHECK=$(grep -n "time.sleep(2.5)" simple_lidar.py | wc -l)
if [ $TIMING_CHECK -ge 1 ]; then
    echo "✓ simple_lidar.py includes proper motor timing"
else
    echo "⚠ simple_lidar.py may have timing issues - not using 2.5s delay"
fi

# Check force_scan_test.py for proper timing
TIMING_CHECK=$(grep -n "time.sleep(2.5)" force_scan_test.py | wc -l)
if [ $TIMING_CHECK -ge 1 ]; then
    echo "✓ force_scan_test.py includes proper motor timing"
else
    echo "⚠ force_scan_test.py may have timing issues - not using 2.5s delay"
fi

echo
echo "If tests fail, check TROUBLESHOOTING.md for solutions"
echo "The most common issue is insufficient time between motor start and scan"

# Run the force scan test
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Running Force Scan Test${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This test uses the FORCE_SCAN command which often works better.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test and continue to the next one.${NC}"
echo
read -p "Press Enter to start the test..."
python3 force_scan_test.py
echo

# Run the simple lidar test
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Running Simple LIDAR Test${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This is a simplified driver that focuses on reliability.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test and continue to the next one.${NC}"
echo
read -p "Press Enter to start the test..."
python3 simple_lidar.py
echo

# Run the ROS-compatible test
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Running ROS-Compatible Test${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This test uses a ROS-compatible driver that can be integrated with ROS.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test and continue to the next one.${NC}"
echo
read -p "Press Enter to start the test..."
python3 ros_compatible.py
echo

# Run the full lidar driver
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}Running Full LIDAR Driver${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This is the complete RPLiDAR driver implementation.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test.${NC}"
echo
read -p "Press Enter to start the test..."
python3 rplidar_driver.py
echo

echo -e "${BLUE}====================================${NC}"
echo -e "${GREEN}All tests completed!${NC}"
echo -e "${BLUE}====================================${NC}"
echo
echo -e "${YELLOW}Which test worked best for you?${NC}"
echo "1. Basic Serial Test (serial_test.py)"
echo "2. Force Scan Test (force_scan_test.py)"
echo "3. Simple LIDAR Test (simple_lidar.py)"
echo "4. ROS-Compatible Test (ros_compatible.py)"
echo "5. Full LIDAR Driver (rplidar_driver.py)"
echo

echo -e "${YELLOW}To run a specific test again, use:${NC}"
echo "python3 [script_name.py]"
echo
echo -e "${YELLOW}For visualization, try:${NC}"
echo "python3 rplidar_visualization.py"
echo
echo -e "${YELLOW}For robot integration example, try:${NC}"
echo "python3 robot_integration.py"
echo

echo -e "${BLUE}Happy scanning!${NC}" 