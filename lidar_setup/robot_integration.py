#!/usr/bin/env python3
# Sample Robot Integration for RPLiDAR A1M8
# Demonstrates how to use RPLiDAR for obstacle avoidance in an autonomous robot

import time
import math
import threading
from enum import Enum
from rplidar_driver import RPLidar

# Navigation constants
MIN_DISTANCE = 500  # Minimum safe distance in mm
CAUTION_DISTANCE = 1000  # Caution distance in mm
FORWARD_ANGLE_RANGE = 45  # Angle range to check for forward obstacles (degrees to each side)
SIDE_ANGLE_RANGE = 60  # Angle range to check for side obstacles (degrees)

class Direction(Enum):
    """Enum for robot movement directions"""
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    STOP = 4

class RobotController:
    """
    Class for integrating RPLiDAR with a robot's navigation system
    This is a simplified example - in a real robot, you would connect this
    to your motor controllers and other hardware
    """
    def __init__(self, lidar, enable_debug=True):
        """
        Initialize the robot controller
        
        Args:
            lidar (RPLidar): The initialized lidar object
            enable_debug (bool): Whether to print debug messages
        """
        self.lidar = lidar
        self.enable_debug = enable_debug
        self.running = False
        self.control_thread = None
        self.current_direction = Direction.STOP
        self.obstacle_detected = False
        self.obstacle_directions = set()
        self.closest_obstacle = (None, float('inf'))  # (angle, distance)
        
        # Lock for thread safety
        self.state_lock = threading.Lock()
    
    def start(self):
        """Start the robot controller"""
        if self.running:
            return
            
        # Make sure LIDAR is connected and scanning
        if not self.lidar.is_scanning:
            if not self.lidar.start_scan():
                print("Error: LIDAR is not scanning")
                return False
                
        # Start control thread
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        if self.enable_debug:
            print("Robot controller started")
            
        return True
    
    def stop(self):
        """Stop the robot controller"""
        if not self.running:
            return
            
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(2)
            
        self._set_direction(Direction.STOP)
        
        if self.enable_debug:
            print("Robot controller stopped")
    
    def _debug_print(self, message):
        """Print debug message if enabled"""
        if self.enable_debug:
            print(f"[Robot] {message}")
    
    def _set_direction(self, direction):
        """
        Set the robot's movement direction
        
        In a real robot implementation, this would interface with your
        motor controllers to actually control the robot's movement
        
        Args:
            direction (Direction): The direction to move
        """
        with self.state_lock:
            if direction != self.current_direction:
                self.current_direction = direction
                self._debug_print(f"Direction changed to: {direction.name}")
                
                # Here you would implement the actual motor control:
                if direction == Direction.FORWARD:
                    # Code to drive motors forward
                    pass
                elif direction == Direction.BACKWARD:
                    # Code to drive motors backward
                    pass
                elif direction == Direction.LEFT:
                    # Code to turn robot left
                    pass
                elif direction == Direction.RIGHT:
                    # Code to turn robot right
                    pass
                elif direction == Direction.STOP:
                    # Code to stop motors
                    pass
    
    def get_direction(self):
        """Get the current movement direction"""
        with self.state_lock:
            return self.current_direction
    
    def _control_loop(self):
        """
        Main control loop that runs in a separate thread
        Gets data from LIDAR and makes navigation decisions
        """
        # Allow some time for initial data collection
        time.sleep(1.5)
        
        while self.running:
            try:
                # Check for obstacles
                self._check_obstacles()
                
                # Decide direction based on obstacle data
                self._decide_direction()
                
                # Short sleep to prevent CPU overuse
                time.sleep(0.1)
                
            except Exception as e:
                self._debug_print(f"Error in control loop: {e}")
                time.sleep(0.1)
    
    def _check_obstacles(self):
        """Check for obstacles in different directions"""
        with self.state_lock:
            # Clear previous obstacle directions
            self.obstacle_directions = set()
            
            # Get filtered distances
            distances = self.lidar.get_obstacle_distances(min_distance=10)
            
            # Check front obstacles
            front_min_angle = (360 - FORWARD_ANGLE_RANGE) % 360
            front_max_angle = FORWARD_ANGLE_RANGE
            front_distance = float('inf')
            
            # Check front obstacle zone
            for angle in range(front_min_angle, 360):
                distance = distances[angle]
                if 0 < distance < front_distance:
                    front_distance = distance
            
            for angle in range(0, front_max_angle + 1):
                distance = distances[angle]
                if 0 < distance < front_distance:
                    front_distance = distance
            
            # Check left obstacles (90 degrees is left)
            left_min_angle = 90 - SIDE_ANGLE_RANGE
            left_max_angle = 90 + SIDE_ANGLE_RANGE
            left_distance = float('inf')
            
            for angle in range(left_min_angle, left_max_angle + 1):
                idx = angle % 360
                distance = distances[idx]
                if 0 < distance < left_distance:
                    left_distance = distance
            
            # Check right obstacles (270 degrees is right)
            right_min_angle = 270 - SIDE_ANGLE_RANGE
            right_max_angle = 270 + SIDE_ANGLE_RANGE
            right_distance = float('inf')
            
            for angle in range(right_min_angle, right_max_angle + 1):
                idx = angle % 360
                distance = distances[idx]
                if 0 < distance < right_distance:
                    right_distance = distance
            
            # Check back obstacles (180 degrees is back)
            back_min_angle = 180 - SIDE_ANGLE_RANGE
            back_max_angle = 180 + SIDE_ANGLE_RANGE
            back_distance = float('inf')
            
            for angle in range(back_min_angle, back_max_angle + 1):
                idx = angle % 360
                distance = distances[idx]
                if 0 < distance < back_distance:
                    back_distance = distance
            
            # Determine if there are obstacles in different directions
            if front_distance < CAUTION_DISTANCE:
                self.obstacle_directions.add(Direction.FORWARD)
                if front_distance < MIN_DISTANCE:
                    self._debug_print(f"Front obstacle detected at {front_distance:.0f}mm!")
            
            if left_distance < CAUTION_DISTANCE:
                self.obstacle_directions.add(Direction.LEFT)
                if left_distance < MIN_DISTANCE:
                    self._debug_print(f"Left obstacle detected at {left_distance:.0f}mm!")
            
            if right_distance < CAUTION_DISTANCE:
                self.obstacle_directions.add(Direction.RIGHT)
                if right_distance < MIN_DISTANCE:
                    self._debug_print(f"Right obstacle detected at {right_distance:.0f}mm!")
            
            if back_distance < CAUTION_DISTANCE:
                self.obstacle_directions.add(Direction.BACKWARD)
                if back_distance < MIN_DISTANCE:
                    self._debug_print(f"Back obstacle detected at {back_distance:.0f}mm!")
            
            # Get the overall closest obstacle
            closest_angle, closest_distance = self.lidar.get_closest_obstacle(min_distance=10)
            if closest_angle is not None:
                self.closest_obstacle = (closest_angle, closest_distance)
    
    def _decide_direction(self):
        """Decide which direction to move based on obstacle data"""
        with self.state_lock:
            current_direction = self.current_direction
            obstacles = self.obstacle_directions
            
            # Very simplified obstacle avoidance logic
            # In a real robot, you'd likely use a more sophisticated algorithm
            
            # If no obstacles, move forward
            if len(obstacles) == 0:
                self._set_direction(Direction.FORWARD)
                return
            
            # If obstacle in front
            if Direction.FORWARD in obstacles:
                # If front obstacle is too close, stop
                angle, distance = self.closest_obstacle
                if angle is not None and (angle < FORWARD_ANGLE_RANGE or angle > (360 - FORWARD_ANGLE_RANGE)):
                    if distance < MIN_DISTANCE:
                        self._set_direction(Direction.STOP)
                        self._debug_print("Stopped due to close front obstacle")
                        return
                
                # Otherwise try to go around - check sides
                if Direction.LEFT not in obstacles:
                    self._set_direction(Direction.LEFT)
                elif Direction.RIGHT not in obstacles:
                    self._set_direction(Direction.RIGHT)
                else:
                    # Both sides blocked, try backward
                    if Direction.BACKWARD not in obstacles:
                        self._set_direction(Direction.BACKWARD)
                    else:
                        # All directions blocked, stop
                        self._set_direction(Direction.STOP)
            else:
                # No front obstacle, continue forward
                self._set_direction(Direction.FORWARD)

def main():
    """
    Main function to demonstrate robot integration with RPLiDAR
    """
    # Initialize RPLidar - adjust port as needed
    lidar = RPLidar(port='/dev/ttyUSB0')
    
    try:
        # Connect to the device
        if not lidar.connect():
            print("Failed to connect to RPLiDAR")
            return
        
        print("Connected to RPLiDAR")
        
        # Reset the device
        lidar.reset()
        time.sleep(1)
        
        # Start scanning
        if not lidar.start_scan():
            print("Failed to start scan")
            lidar.disconnect()
            return
        
        print("LIDAR scanning started")
        
        # Create and start robot controller
        robot = RobotController(lidar)
        robot.start()
        
        # Run for a while
        print("\nRobot running - press Ctrl+C to stop")
        print("This is a simulation - no actual movement will occur")
        
        # Keep running until interrupted
        while True:
            direction = robot.get_direction()
            if direction == Direction.FORWARD:
                print("\rMoving FORWARD      ", end="")
            elif direction == Direction.BACKWARD:
                print("\rMoving BACKWARD     ", end="")
            elif direction == Direction.LEFT:
                print("\rTurning LEFT        ", end="")
            elif direction == Direction.RIGHT:
                print("\rTurning RIGHT       ", end="")
            elif direction == Direction.STOP:
                print("\rSTOPPED             ", end="")
            
            time.sleep(0.2)
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up
        if 'robot' in locals():
            robot.stop()
        lidar.stop_scan()
        lidar.disconnect()
        print("Done")

if __name__ == "__main__":
    main() 