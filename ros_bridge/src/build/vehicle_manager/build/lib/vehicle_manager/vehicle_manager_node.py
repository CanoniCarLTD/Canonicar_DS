import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService

class VehicleManagerNode(Node):
    def __init__(self):
        super().__init__('vehicle_manager_node')

        # Get the parameters
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.sensor_config = self.get_parameter('sensor_config').get_parameter_value().string_value

        self.get_logger().info(f"Vehicle Manager started with parameters: {self.host}, {self.port}")

        # Initialize the list of vehicle types (can be extended)
        self.vehicle_types = ['car_type_1', 'car_type_2', 'car_type_3']
        self.current_vehicle_index = 0  # To keep track of which vehicle to spawn next

        self.launch_service = LaunchService()

        # Subscribe to the 'lap_completed' topic to trigger the relaunch
        self.lap_subscription = self.create_subscription(
            String,
            '/lap_completed',  # Topic name for lap completion
            self.lap_callback,  # Callback function
            10  # QoS
        )

        self.get_logger().info("Vehicle Manager Node started, waiting for lap completion.")

    def lap_callback(self, msg):
        """
        Callback triggered when a lap is finished.
        This method will trigger the relaunching of a new vehicle and data collection.
        """
        self.get_logger().info(f"Lap completed! Triggering vehicle spawn and data collection: {msg.data}")
        
        # Get the next vehicle type in the list, looping back if we reach the end
        vehicle_type = self.vehicle_types[self.current_vehicle_index]
        self.current_vehicle_index = (self.current_vehicle_index + 1) % len(self.vehicle_types)

        # Create a new launch description for the vehicle and data collection nodes
        ld = self.create_lap_launch_description(vehicle_type)

        # Include this launch description in the LaunchService
        self.launch_service.include_launch_description(ld)

        # Run the LaunchService to trigger the launch
        self.launch_service.run()

    def create_lap_launch_description(self, vehicle_type):
        """
        This method creates a LaunchDescription for spawning a new vehicle and
        starting the data collector after a lap is completed.
        """
        # Include the spawn vehicle launch, passing the vehicle type as an argument
        vehicle_launch = LaunchNode(
            package='client_node',
            executable='spawn_vehicle_launch',
            name='spawn_vehicle_node',
            output='screen',
            parameters=[{
                'host': self.host,
                'port': self.port,
                'sensor_config': self.sensor_config,
                'vehicle_type': vehicle_type  # Pass the current vehicle type
            }]
        )

        # Define DataCollector node to be relaunched
        data_collector_node = LaunchNode(
            package='data_collector_node',
            executable='data_collector',
            name='data_collector_node',
            output='screen',
        )

        data_process_node = Node(
            package='data_process_node',
            executable='data_process',
            name='data_process_node',
            output='screen'
        )

        # Add the actions (vehicle spawn and data collector) to the launch description
        ld = LaunchDescription()
        ld.add_action(vehicle_launch)
        ld.add_action(data_collector_node)
        ld.add_action(data_process_node)

        return ld

def main(args=None):
    rclpy.init(args=args)

    vehicle_manager_node = VehicleManagerNode()

    rclpy.spin(vehicle_manager_node)

    vehicle_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
