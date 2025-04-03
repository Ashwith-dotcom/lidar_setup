#!/usr/bin/env python3
# RPLiDAR A1M8 Visualization Tool
# Displays real-time LIDAR data using matplotlib

import matplotlib.pyplot as plt
import numpy as np
import math
import time
import threading
from matplotlib.animation import FuncAnimation
from rplidar_driver import RPLidar

class LidarVisualizer:
    """
    Class for visualizing RPLiDAR data in real-time
    """
    def __init__(self, lidar, max_distance=6000):
        """
        Initialize the visualizer
        
        Args:
            lidar (RPLidar): The initialized RPLidar object
            max_distance (int): Maximum display distance in mm
        """
        self.lidar = lidar
        self.max_distance = max_distance
        self.fig = None
        self.ax = None
        self.scatter = None
        self.anim = None
        self.running = False
        
        # Data for visualization
        self.angles = np.arange(0, 360)
        self.distances = np.zeros(360)
        self.x = np.zeros(360)
        self.y = np.zeros(360)
        
        # Lock for thread safety
        self.data_lock = threading.Lock()
    
    def polar_to_cartesian(self):
        """
        Convert polar coordinates (angle, distance) to cartesian (x, y)
        """
        with self.data_lock:
            # Convert to radians for numpy
            angles_rad = np.radians(self.angles)
            
            # Convert to cartesian coordinates
            self.x = self.distances * np.cos(angles_rad)
            self.y = self.distances * np.sin(angles_rad)
    
    def update_plot(self, frame):
        """
        Update function for animation - gets called every frame
        """
        if not self.running:
            return self.scatter,
            
        with self.data_lock:
            # Get the latest distances
            self.distances = self.lidar.get_obstacle_distances(
                max_distance=self.max_distance
            )
            
            # Convert to cartesian
            self.polar_to_cartesian()
            
            # Update scatter plot
            self.scatter.set_offsets(np.column_stack((self.x, self.y)))
            
            # Find obstacles
            obstacles = self.lidar.detect_obstacles(
                threshold_distance=1000,
                min_points=3
            )
            
            # Clear previous obstacle annotations
            for txt in self.ax.texts:
                txt.remove()
                
            # Add new obstacle annotations
            for i, (start, end, dist, center) in enumerate(obstacles):
                center_rad = np.radians(center)
                text_x = (dist + 200) * np.cos(center_rad)  # Offset for label
                text_y = (dist + 200) * np.sin(center_rad)
                self.ax.text(
                    text_x, text_y, f"{i+1}", 
                    fontsize=9, color='red', 
                    ha='center', va='center'
                )
            
            # Update title with closest obstacle
            closest_angle, closest_dist = self.lidar.get_closest_obstacle()
            if closest_angle is not None:
                self.ax.set_title(
                    f"RPLiDAR Scan - Closest Obstacle: {closest_angle}° at {closest_dist:.0f}mm"
                )
            else:
                self.ax.set_title("RPLiDAR Scan - No obstacles detected")
                
        return self.scatter,
    
    def start(self):
        """
        Start the visualization
        """
        # Create figure and axes
        self.fig, self.ax = plt.subplots(figsize=(10, 8), subplot_kw=dict(polar=False))
        
        # Set limits and grid
        max_coord = self.max_distance
        self.ax.set_xlim(-max_coord, max_coord)
        self.ax.set_ylim(-max_coord, max_coord)
        self.ax.grid(True)
        
        # Add LIDAR position marker
        self.ax.plot(0, 0, 'ro', markersize=10)
        self.ax.text(0, 0, "LIDAR", fontsize=10, ha='center', va='bottom', color='red')
        
        # Add reference circles
        for dist in range(1000, self.max_distance + 1, 1000):
            circle = plt.Circle((0, 0), dist, fill=False, color='gray', linestyle='--', alpha=0.5)
            self.ax.add_artist(circle)
            # Add distance label
            self.ax.text(0, dist, f"{dist}mm", fontsize=8, color='gray', ha='center', va='bottom')
        
        # Add cardinal directions
        directions = [
            (0, max_coord*1.05, "FRONT"),
            (max_coord*1.05, 0, "RIGHT"),
            (0, -max_coord*1.05, "BACK"),
            (-max_coord*1.05, 0, "LEFT")
        ]
        
        for x, y, label in directions:
            self.ax.text(x, y, label, fontsize=12, ha='center', va='center', color='blue')
        
        # Set axis labels and title
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_title("RPLiDAR Scan")
        
        # Equal aspect ratio
        self.ax.set_aspect('equal')
        
        # Create scatter plot
        self.scatter = self.ax.scatter(
            self.x, self.y, 
            s=5,  # Point size
            c=np.zeros(360),  # Color array
            cmap='viridis',
            alpha=0.7
        )
        
        # Mark as running and start animation
        self.running = True
        self.anim = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=100,  # Update every 100ms
            blit=True
        )
        
        # Show plot
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        """
        Stop the visualization
        """
        self.running = False
        if self.anim:
            self.anim.event_source.stop()
        if self.fig:
            plt.close(self.fig)

def main():
    """
    Main function to demonstrate LIDAR visualization
    """
    # Initialize RPLidar object - adjust port as needed
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
        
        print("Scanning started")
        
        # Create and start the visualizer
        visualizer = LidarVisualizer(lidar)
        
        # Allow some time for initial data collection
        print("Collecting initial data, please wait...")
        time.sleep(2)
        
        # Start visualization
        print("Starting visualization (close window to exit)")
        visualizer.start()
    
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Clean up
        if 'visualizer' in locals():
            visualizer.stop()
        lidar.stop_scan()
        lidar.disconnect()
        print("Done")

if __name__ == "__main__":
    main() 