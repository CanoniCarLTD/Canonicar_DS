import rclpy
from rclpy.node import Node
import json
import os
import math
import pandas as pd
import numpy as np
from datetime import datetime
import carla
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time



class DataProcessNode(Node):
    def __init__(self):
        super().__init__('data_process')
        self.get_logger().info("DataProcess Node initialized.")

        # Parameters
        self.declare_parameter('lap_threshold', 10.0)  # Distance in meters to consider lap completed
        self.lap_threshold = self.get_parameter('lap_threshold').value
        
        # Track start point and lap status
        self.start_point = None
        self.lap_completed = False
        self.lap_count = 0
        self.data_lock = threading.Lock()
        self.lap_data = []
        self.lap_time = 0.0
        
        self.location_subscriber = self.create_subscription(Float32MultiArray, '/carla/vehicle/location', self.location_callback,10)
        
        # For communicating with data_collector
        self.get_data_publisher = self.create_publisher(
            String, 
            '/carla/data_request', 
            10)
        
        self.data_subscriber = self.create_subscription(
            String,
            '/carla/collector_data',
            self.data_callback,
            10)
            
        self.get_logger().info("DataProcess Node setup complete")
    
    def location_callback(self, msg):
        """Process vehicle location data"""
        # Save vehicle location for future use
        self.vehicle_location = msg.data
        # self.get_logger().info(f"Vehicle location: {self.vehicle_location}")
        if self.start_point is None:
            self.start_point = self.vehicle_location
            self.get_logger().info(f"Start position recorded: {self.start_point}")

            # Initialize lap timer
            self.lap_start_time = self.get_clock().now()
            self.get_logger().info(f"Lap timer started at: {self.lap_start_time}")
            time.sleep(1)  # Wait for a second before starting to avoid false lap completion

            return

        distance = self.calculate_distance(self.start_point, self.vehicle_location)
        self.get_logger().info(f"Distance from start: {distance:.2f} m, start point: {self.start_point} , current position: {self.vehicle_location}")
        if distance < 1:
            if not self.lap_completed:
                self.lap_completed = True
                self.lap_count += 1
                self.lap_end_time = self.get_clock().now()
                self.lap_time = self.lap_end_time - self.lap_start_time
                self.lap_start_time = self.get_clock().now()
                self.get_logger().info(f"Lap {self.lap_count} completed!")
                self.get_logger().info(f"Lap time: {self.lap_time} seconds")
                self.request_collector_data()
        else:
            self.lap_completed = False
    
    def calculate_distance(self, point1, point2):
        if point1 is None or point2 is None:
            return float('inf')
        return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)))
        
    def request_collector_data(self):
        """Send request to data collector for processed data"""
        request_msg = String()
        request_msg.data = "REQUEST_DATA"
        self.get_logger().info("Requesting data from collector...")
        self.get_data_publisher.publish(request_msg)
        
    def data_callback(self, msg):
        """Handle data received from data collector"""
        try:
            with self.data_lock:
                data_dict = json.loads(msg.data)
                self.get_logger().info(f"Received data for lap {self.lap_count}")
                
                # Add lap number to data
                data_dict["lap_number"] = self.lap_count
                self.lap_data.append(data_dict)
                
                # Save data after each lap
                self.save_data_to_excel()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON data: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing collected data: {e}")
    
    def save_data_to_excel(self):
        """Save collected data to an Excel file"""
        if not self.lap_data:
            self.get_logger().warn("No data to save")
            return
            
        try:
            # Create data directory if it doesn't exist
            data_dir = os.path.join(os.path.expanduser('/ros_bridge/src'), 'carla_data')
            os.makedirs(data_dir, exist_ok=True)
            
            # Use a fixed filename instead of timestamp-based name
            filename = os.path.join(data_dir, "lap_data.xlsx")
            
            # Process only the most recent lap data
            lap_entry = self.lap_data[-1]
            
            # Extract metrics from the lap
            entry = {
                'lap': lap_entry['lap_number'],
                'average_speed': lap_entry['average_speed'],
                'fuel_consumption': lap_entry['fuel_consumption'],
                'total_steering': lap_entry['total_steering'],
                'total_throttle': lap_entry['total_throttle'],
                'total_brake': lap_entry['total_brake']
            }
            
            # Add IMU data (acceleration, angular velocity)
            if 'imu' in lap_entry:
                imu = lap_entry['imu']
                entry.update({
                    'accel_x': imu[0],
                    'accel_y': imu[1],
                    'accel_z': imu[2],
                    'ang_vel_x': imu[3],
                    'ang_vel_y': imu[4],
                    'ang_vel_z': imu[5]
                })
            
            # Add GNSS data if available
            if 'gnss' in lap_entry:
                gnss = lap_entry['gnss']
                entry.update({
                    'latitude': gnss[0],
                    'longitude': gnss[1],
                    'altitude': gnss[2],
                    'velocity': gnss[3]
                })
            
            entry['lap time'] = float(self.lap_time.nanoseconds) / 1e9 
            
            # Add timestamp for reference
            entry['timestamp'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Create DataFrame for new lap data
            df_new = pd.DataFrame([entry])  # Just one row for the new lap
            
            # Check if file already exists
            if os.path.exists(filename):
                # Load existing data and append new lap data
                df_existing = pd.read_excel(filename)
                df_combined = pd.concat([df_existing, df_new], ignore_index=True)
                df_combined.to_excel(filename, index=False)
                self.get_logger().info(f"Appended lap {lap_entry['lap_number']} data to {filename}")
            else:
                # Create new file with first lap data
                df_new.to_excel(filename, index=False)
                self.get_logger().info(f"Created new lap data file at {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving data to Excel: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()