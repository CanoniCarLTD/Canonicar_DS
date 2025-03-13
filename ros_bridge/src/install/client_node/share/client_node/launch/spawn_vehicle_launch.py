import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for host, port, sensor config, and vehicle type
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='Host address for CARLA server'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2000',
        description='Port number for CARLA server'
    )

    sensor_config_arg = DeclareLaunchArgument(
        'sensor_config',
        default_value=os.path.join(
            get_package_share_directory('client_node'),
            'client_node',
            'sensors_config.json'
        ),
        description='Path to the sensors_config.json file'
    )

    vehicle_type_arg = DeclareLaunchArgument(
        'vehicle_type',
        default_value="vehicle.audi.a2",  # Default vehicle type
        description='Type of vehicle to spawn'
    )

    # Define the spawn_vehicle_node with the added vehicle_type parameter
    spawn_vehicle_node = Node(
        package='client_node',
        executable='spawn_vehicle_node',
        name='spawn_vehicle_node',
        output='screen',
        parameters=[
            {'host': LaunchConfiguration('host')},
            {'port': LaunchConfiguration('port')},
            {'sensor_config_file': LaunchConfiguration('sensor_config')},
            {'vehicle_type': LaunchConfiguration('vehicle_type')}  # Pass the vehicle type
        ]
    )

    # Return the LaunchDescription with all the actions
    return LaunchDescription([
        host_arg,
        port_arg,
        sensor_config_arg,
        vehicle_type_arg,  # Add the vehicle_type argument
        spawn_vehicle_node
    ])
