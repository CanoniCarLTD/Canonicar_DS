import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
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


    sensor_config_file = os.path.join(
        get_package_share_directory('client_node'),
        'client_node',
        'sensors_config.json'
    )

    track_line_file = os.path.join(
        get_package_share_directory('map_loader'),
        'map_loader',
        'track.line'
    )

    track_xodr_file = os.path.join(
        get_package_share_directory('map_loader'),
        'map_loader',
        'generatedTrack.xodr'
    )

    run_load_map_node = Node(
        package='map_loader',
        executable='load_map_node',
        name='load_map_node',
        output='screen',
        parameters=[
            {
                'host': LaunchConfiguration('host'),
                'TRACK_LINE': track_line_file,
                'TRACK_XODR': track_xodr_file,
                'port': LaunchConfiguration('port'),
            }
        ]
    )

    spawn_vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('client_node'),
                'launch',
                'spawn_vehicle_launch.py'
            )
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'sensor_config': sensor_config_file
        }.items()
    )


    # Increase delay between map loading and vehicle spawning to ensure map is fully loaded
    delayed_spawn_vehicle = TimerAction(
        period=5.0, 
        actions=[spawn_vehicle_launch]
    )
    
    
        # Add DataCollector node
    data_collector_node = Node(
        package='data_collector_node',
        executable='data_collector',
        name='data_collector_node',
        output='screen',
    )

    delayed_data_collector = TimerAction(
        period=7.0,  # Delay to ensure vehicle is spawned first
        actions=[data_collector_node]
    )

    data_process_node = Node(
        package='data_process_node',
        executable='data_process',
        name='data_process_node',
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(host_arg)
    ld.add_action(port_arg)
    ld.add_action(run_load_map_node)
    ld.add_action(spawn_vehicle_launch)
    ld.add_action(data_collector_node)
    ld.add_action(data_process_node)
    
    return ld