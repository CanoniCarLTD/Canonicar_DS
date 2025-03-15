import os
import rclpy
import math
import struct
import json
from carla import Client
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
from std_msgs.msg import Float32MultiArray, String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory


class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")
        try:
            self.prev_gnss = (
                None  # Store previous GNSS position for velocity calculation
            )
            self.prev_time = None  # Store timestamp
            self.speed_records = []  # Store speed values for average calculation

            # Add subscribers and publishers for data requests
            self.data_request_sub = self.create_subscription(
                String, "/carla/data_request", self.handle_data_request, 10
            )

            self.data_publisher = self.create_publisher(
                String, "/carla/collector_data", 10
            )

            # Start to deploy vehicles after map is loaded

            self.start_vehicle_manager = self.create_publisher(
                String, "/start_vehicle_manager", 10
            )
            self.lap_subscription = self.create_subscription(
                String,
                "/lap_completed",  # Topic name for lap completion
                self.lap_ending_callback,  # Callback function
                10,  # QoS
            )

        except Exception as e:
            self.get_logger().error(f"Error connecting to CARLA server: {e}")
            return

        self.data_buffer = []  # List to store synchronized data

        # TODO different cars have different frontal areas, need to adjust per vehicle.
        self.frontal_area = (
            2.2  # Default estimated frontal area in m² (can be adjusted)
        )
        self.fuel_consumption = 0.0  # Total fuel consumption in liters

        self.throttle_sum = 0.0
        self.brake_sum = 0.0
        self.steering_sum = 0.0

        self.setup_subscribers()
        self.get_logger().info("DataCollector Node initialized.")

    def lap_ending_callback(self, msg):
        """Callback function for lap completion."""
        self.get_logger().info("Lap completed.")
        self.throttle_sum = 0.0
        self.brake_sum = 0.0
        self.steering_sum = 0.0
        self.fuel_consumption = 0.0
        request_msg = String()
        request_msg.data = "DataCollector is ready"
        self.start_vehicle_manager.publish(request_msg)
        self.get_logger().info("Data collector is ready to start the next lap.")

    def setup_subscribers(self):
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info("Setting up subscribers...")

        self.imu_sub = Subscriber(self, Imu, "/carla/imu/imu")
        self.gnss_sub = Subscriber(self, NavSatFix, "/carla/gnss/gnss")

        self.physics_sub = Subscriber(self, Float32MultiArray, "/carla/vehicle/physics")
        self.controller_sub = Subscriber(
            self, Float32MultiArray, "/carla/vehicle/control"
        )

        # Synchronizing IMU and GNSS Data
        self.ats = ApproximateTimeSynchronizer(
            [self.imu_sub, self.gnss_sub, self.physics_sub, self.controller_sub],
            queue_size=10,
            slop=0.05,
            allow_headerless=True,  # Enables processing of messages without timestamps
        )
        self.ats.registerCallback(self.sync_callback)
        self.get_logger().info("Subscribers set up successfully.")

    def sync_callback(self, imu_msg, gnss_msg, physics_msg, controller_msg):
        """Callback function for synchronized data."""
        # self.get_logger().info("Synchronized callback triggered.")
        # self.get_logger().info(f"The physics mass {physics_msg.data[0]}, drag coef {physics_msg.data[1]}.")

        processed_data = self.process_data(
            imu_msg, gnss_msg, physics_msg, controller_msg
        )
        self.data_buffer.append(processed_data)
        # self.get_logger().info("Data appended to buffer.")
        # self.get_logger().info(f'Latest data: {self.get_latest_data()}')

    def process_data(self, imu_msg, gnss_msg, physics_msg, controller_msg):
        """Process synchronized data and compute vehicle parameters."""
        self.steering_sum += abs(controller_msg.data[0])
        self.throttle_sum += abs(controller_msg.data[1])
        self.brake_sum += abs(controller_msg.data[2])
        data = {
            "imu": self.process_imu(imu_msg),
            "gnss": self.process_gnss(gnss_msg, physics_msg),
            "average_speed": self.get_average_speed(),
            "fuel_consumption": self.get_total_fuel_consumption(),
            "total_steering": self.steering_sum,
            "total_throttle": self.throttle_sum,
            "total_brake": self.brake_sum,
        }
        # self.get_logger().info(f"Data:{data}")
        return data

    def process_imu(self, imu_msg):
        """Process IMU data."""
        # self.get_logger().info("Processing IMU data...")
        imu_data = [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ]
        return imu_data

    def process_gnss(self, gnss_msg, physics_msg):
        """Process GNSS data and compute velocity."""
        latitude, longitude, altitude = (
            gnss_msg.latitude,
            gnss_msg.longitude,
            gnss_msg.altitude,
        )

        velocity = 0.0
        if self.prev_gnss is not None and self.prev_time is not None:
            delta_time = (
                self.get_clock().now() - self.prev_time
            ).nanoseconds / 1e9  # Convert to seconds
            delta_lat = latitude - self.prev_gnss[0]
            delta_lon = longitude - self.prev_gnss[1]

            delta_x = delta_lon * 111320  # Convert lon to meters
            delta_y = delta_lat * 110540  # Convert lat to meters
            distance = math.sqrt(delta_x**2 + delta_y**2)

            if delta_time > 0:
                velocity = distance / delta_time  # Speed in m/s
                km_per_hour = velocity * 3.6  # Convert to km/h
                self.speed_records.append(km_per_hour)  # Store the computed speed
                self.calculate_fuel_consumption(
                    velocity, delta_time, physics_msg.data[0], physics_msg.data[1]
                )

        self.prev_gnss = (latitude, longitude)
        self.prev_time = self.get_clock().now()

        return np.array([latitude, longitude, altitude, velocity])

    def get_average_speed(self):
        """Calculate the average speed from stored speed values."""
        if not self.speed_records:
            return 0.0  # Return 0 if no speed data available
        return sum(self.speed_records) / len(self.speed_records)

    def calculate_fuel_consumption(self, velocity, delta_time, mass, drag_coefficient):
        """Estimate fuel consumption based on vehicle parameters."""
        fuel_density = 0.75  # kg/L
        fuel_energy = 34.2e6  # J/L

        # Compute aerodynamic drag power
        air_density = 1.225  # kg/m³
        aerodynamic_drag = (
            0.5 * air_density * drag_coefficient * self.frontal_area * (velocity**3)
        )

        # Compute rolling resistance (approximate)
        rolling_resistance_coefficient = 0.015
        rolling_resistance = rolling_resistance_coefficient * mass * 9.81 * velocity

        # Total power required
        total_power = aerodynamic_drag + rolling_resistance

        # Compute fuel consumption
        fuel_consumed = (total_power * delta_time) / (fuel_energy * fuel_density)
        self.fuel_consumption += fuel_consumed

    def get_total_fuel_consumption(self):
        """Return the total fuel consumption in liters."""
        return self.fuel_consumption

    def get_latest_data(self):
        """Retrieve the most recent synchronized data."""
        return self.data_buffer[-1] if self.data_buffer else None

    def clear_buffer(self):
        """Clear the stored data buffer."""
        self.data_buffer.clear()

    def handle_data_request(self, msg):
        """Handle request for data from data_process_node"""
        if msg.data == "REQUEST_DATA":
            self.get_logger().info("Data request received, sending latest data")

            # Get the latest processed data
            latest_data = self.get_latest_data()

            if latest_data:
                # Convert numpy arrays to lists for JSON serialization
                for key, value in latest_data.items():
                    if isinstance(value, np.ndarray):
                        latest_data[key] = value.tolist()

                # Send data as JSON string
                response = String()
                response.data = json.dumps(latest_data)
                self.data_publisher.publish(response)
                self.get_logger().info("Data sent to data_process_node")
            else:
                self.get_logger().warn("No data available to send")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
