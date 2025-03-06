import math
import rclpy
from rclpy.node import Node
import carla
from carla import Client, Transform, Location, Rotation, TrafficManager
import json
import dotenv
import os
from ament_index_python.packages import get_package_share_directory

# For ROS2 messages
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from std_msgs.msg import Header

import numpy as np

# Import conversion functions from sensors_data
from sensors_data import (
    carla_image_to_ros_image,
    carla_lidar_to_ros_pointcloud2,
    carla_imu_to_ros_imu,
    carla_gnss_to_ros_navsatfix,
)

import threading  # Ensure threading is imported

class SpawnVehicleNode(Node):
    def __init__(self):
        super().__init__('spawn_vehicle_node')

        self.declare_parameter('host', '')
        self.declare_parameter('port', 2000)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value

        try:
            self.client = Client(self.host, self.port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.traffic_manager = self.client.get_trafficmanager(8000)
            self.world.apply_settings(settings)
            self.spawned_sensors = []
        except Exception as e:
            self.get_logger().error(f"Error connecting to CARLA server: {e}")
            return

        self.sensors_publishers = {}

        self.sensor_config_file = os.path.join(
            get_package_share_directory('client_node'), 'client_node', 'sensors_config.json'
        )
        self.vehicle = None
        self.spawn_objects_from_config()

    def spawn_objects_from_config(self):
        """
        Reads the 'objects' array from sensors_config.json
        Spawns the first 'vehicle.*' it finds using the JSON's spawn_point
        Then spawns sensors attached to that vehicle.
        """
        try:
            with open(self.sensor_config_file, 'r') as f:
                config = json.load(f)
                self.get_logger().info("Loaded JSON config from sensors_config.json")

            objects = config.get("objects", [])
            if not objects:
                self.get_logger().error("No objects found in sensors_config.json")
                return

            ego_object = objects[0]
            vehicle_type = ego_object.get("type", "")
            if not vehicle_type.startswith("vehicle."):
                self.get_logger().error("No valid vehicle object found in JSON.")
                return

            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(vehicle_type)
            if not vehicle_bp:
                self.get_logger().error(f"Vehicle blueprint '{vehicle_type}' not found in CARLA.")
                return
            self.get_logger().info(f"Found vehicle blueprint '{vehicle_type}'")

            vehicle_id = ego_object.get("id", "ego_vehicle")
            self.get_logger().info(f"Setting role_name to '{vehicle_id}'")

            # Get the map and find a valid spawn point in section 2 with correct orientation
            carla_map = self.world.get_map()
            
            # Get waypoints on the driving lane
            waypoints = carla_map.generate_waypoints(2.0)  # Generate waypoints every 2 meters
            
            # Find waypoint on road id 1, lane id -1 (left lane), in section 2
            valid_spawn_waypoint = None
            
            # First pass: find all waypoints in the second section (s between 3 and 9)
            candidate_waypoints = []
            for waypoint in waypoints:
                # Search for lane_id=-1 (left lane) in the second segment
                if waypoint.road_id == 1 and waypoint.lane_id == -1 and 3.0 <= waypoint.s <= 9.0:
                    candidate_waypoints.append(waypoint)
            
            if candidate_waypoints:
                # Log all candidate waypoints for debugging
                self.get_logger().info(f"Found {len(candidate_waypoints)} waypoints in section 2 on left lane (id=-1)")
                for i, wp in enumerate(candidate_waypoints):
                    self.get_logger().info(f"Waypoint {i}: s={wp.s}, lane={wp.lane_id}, heading={math.degrees(wp.transform.rotation.yaw)}")
                
                # Select the waypoint with the correct orientation for driving counterclockwise
                valid_spawn_waypoint = candidate_waypoints[0]  # Default to first one
                
                # Find the waypoint that's most aligned with the counterclockwise direction
                # For section 2, the heading should be around 180° (π radians)
                section_2_heading = math.pi  # Approximate heading for counterclockwise at s≈6
                
                closest_angular_diff = float('inf')
                for wp in candidate_waypoints:
                    # Convert heading to be between -pi and pi
                    wp_yaw = math.radians(wp.transform.rotation.yaw)
                    while wp_yaw > math.pi: wp_yaw -= 2 * math.pi
                    while wp_yaw < -math.pi: wp_yaw += 2 * math.pi
                    
                    # Calculate angular difference with expected heading
                    angular_diff = abs(wp_yaw - section_2_heading)
                    if angular_diff > math.pi: 
                        angular_diff = 2 * math.pi - angular_diff
                        
                    if angular_diff < closest_angular_diff:
                        closest_angular_diff = angular_diff
                        valid_spawn_waypoint = wp
                
                self.get_logger().info(f"Selected waypoint with s={valid_spawn_waypoint.s}, "
                                    f"heading={math.degrees(valid_spawn_waypoint.transform.rotation.yaw)} degrees, "
                                    f"at position ({valid_spawn_waypoint.transform.location.x:.2f}, {valid_spawn_waypoint.transform.location.y:.2f})")
                
                # Draw arrows showing the direction of all waypoints
                for wp in candidate_waypoints:
                    # Draw a line in the direction of the waypoint
                    start = wp.transform.location
                    forward_vector = wp.transform.get_forward_vector()
                    end = carla.Location(
                        x=start.x + forward_vector.x * 2.0,
                        y=start.y + forward_vector.y * 2.0,
                        z=start.z + forward_vector.z * 2.0
                    )
                    self.world.debug.draw_arrow(
                        start, end, 
                        thickness=0.2, 
                        arrow_size=0.3,
                        color=carla.Color(255, 255, 0),  # Yellow
                        life_time=30.0
                    )
            
            if valid_spawn_waypoint:
                # Use the waypoint's transform to spawn the vehicle
                vehicle_transform = valid_spawn_waypoint.transform
                # Raise slightly above the road to avoid collision
                vehicle_transform.location.z += 0.3
                
                self.get_logger().info(f"Using waypoint for spawn at {vehicle_transform}, "
                                     f"heading={math.degrees(vehicle_transform.rotation.yaw)} degrees")
                
                # Draw a different color arrow for the selected waypoint
                start = vehicle_transform.location
                forward_vector = valid_spawn_waypoint.transform.get_forward_vector()
                end = carla.Location(
                    x=start.x + forward_vector.x * 3.0,
                    y=start.y + forward_vector.y * 3.0,
                    z=start.z + forward_vector.z * 3.0
                )
                self.world.debug.draw_arrow(
                    start, end, 
                    thickness=0.3, 
                    arrow_size=0.5,
                    color=carla.Color(0, 255, 0),  # Green for selected
                    life_time=30.0
                )
            else:
                # Fall back to the JSON spawn point if no valid waypoint found
                spawn_point_data = ego_object.get("spawn_point", {})
                self.get_logger().info(f"Using JSON spawn point data: {spawn_point_data}")
                vehicle_transform = Transform(
                    Location(
                        x=spawn_point_data.get("x", 0.0),
                        y=spawn_point_data.get("y", 0.0),
                        z=spawn_point_data.get("z", 0.0) + 0.3,  # Add a small offset
                    ),
                    Rotation(
                        roll=spawn_point_data.get("roll", 0.0),
                        pitch=spawn_point_data.get("pitch", 0.0),
                        yaw=spawn_point_data.get("yaw", 0.0),
                    ),
                )
            
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, vehicle_transform)
            if self.vehicle:
                self.get_logger().info(
                    f"Spawned vehicle '{vehicle_id}' with ID={self.vehicle.id} at {vehicle_transform}"
                )
                
                # Configure physics control for better stability
                physics_control = self.vehicle.get_physics_control()
                physics_control.mass = physics_control.mass * 1.5
                self.vehicle.apply_physics_control(physics_control)
                
                # Apply initial control with zero throttle to let the vehicle settle
                control = carla.VehicleControl()
                control.throttle = 0.0
                self.vehicle.apply_control(control)
                
                import time
                time.sleep(0.5)  # Let physics settle
                
                # Configure and enable autopilot
                self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 0)
                self.traffic_manager.auto_lane_change(self.vehicle, False)
                self.vehicle.set_autopilot(True, self.traffic_manager.get_port())
                
                # Start position monitoring AFTER vehicle is configured
                self.stop_monitoring = False
                self.last_known_positions = []
                self.monitor_thread = threading.Thread(target=self._monitor_vehicle_position)
                self.monitor_thread.daemon = True
                self.monitor_thread.start()
                self.get_logger().info("Vehicle position monitoring started")
                
                # Test teleport right after startup to verify functionality
                self.get_logger().info("Testing teleport functionality...")
                time.sleep(2.0)  # Wait a moment for everything to initialize properly
                self._teleport_to_track_start()  # Test teleport once on startup
                
                # Spawn sensors after vehicle and monitor are set up
                sensors = ego_object.get("sensors", [])
                if not sensors:
                    self.get_logger().warn("No sensors found for the vehicle in JSON.")
                else:
                    self.spawn_sensors(sensors)
            else:
                self.get_logger().error("Failed to spawn the vehicle. Check collisions or map issues.")
                return

        except Exception as e:
            self.get_logger().error(f"Error parsing JSON or spawning objects: {e}")

    def spawn_sensors(self, sensors):
        """
        Given a list of sensor definitions (dict), spawn them and attach to self.vehicle.
        Also sets up a listener callback to publish data via ROS2.
        """
        blueprint_library = self.world.get_blueprint_library()

        for sensor_def in sensors:
            try:
                sensor_type = sensor_def.get("type")
                sensor_id = sensor_def.get("id", "unknown_sensor")

                if not sensor_type:
                    self.get_logger().error(
                        f"Sensor definition is missing 'type'. Skipping {sensor_def}."
                    )
                    continue

                sensor_bp = blueprint_library.find(sensor_type)
                if not sensor_bp:
                    self.get_logger().error(f"Sensor blueprint '{sensor_type}' not found. Skipping.")
                    continue

                for attribute, value in sensor_def.items():
                    if attribute in ["type", "id", "spawn_point", "attached_objects"]:
                        continue
                    if sensor_bp.has_attribute(attribute):
                        try:
                            sensor_bp.set_attribute(attribute, str(value))
                        except RuntimeError as e:
                            self.get_logger().error(
                                f"Error setting attribute '{attribute}' to '{value}' for '{sensor_type}': {e}"
                            )
                    else:
                        self.get_logger().warn(
                            f"Blueprint '{sensor_type}' does NOT have an attribute '{attribute}'. Skipping."
                        )

                sp = sensor_def.get("spawn_point", {})
                spawn_transform = Transform(
                    Location(
                        x=sp.get("x", 0.0),
                        y=sp.get("y", 0.0),
                        z=sp.get("z", 0.0),
                    ),
                    Rotation(
                        roll=sp.get("roll", 0.0),
                        pitch=sp.get("pitch", 0.0),
                        yaw=sp.get("yaw", 0.0),
                    ),
                )

                sensor_actor = self.world.try_spawn_actor(sensor_bp, spawn_transform, attach_to=self.vehicle)
                if sensor_actor is None:
                    self.get_logger().error(f"Failed to spawn sensor '{sensor_id}'.")
                    continue
                else:
                    self.spawned_sensors.append(sensor_actor)

                self.get_logger().info(f"Spawned sensor '{sensor_id}' ({sensor_type}) at {spawn_transform}")

                # Create a ROS2 publisher for this sensor
                topic_name, msg_type = self.get_ros_topic_and_type(sensor_type, sensor_id)
                if topic_name and msg_type:
                    publisher = self.create_publisher(msg_type, topic_name, 10)
                    self.sensors_publishers[sensor_actor.id] = {
                        "sensor_id": sensor_id,
                        "sensor_type": sensor_type,
                        "publisher": publisher,
                        "msg_type": msg_type,
                    }

                    # Attach a listener callback to the CARLA sensor
                    def debug_listener(data, actor_id=sensor_actor.id):
                        # self.get_logger().debug(f"Received data from sensor {actor_id}")
                        self.sensor_data_callback(actor_id, data)

                    sensor_actor.listen(debug_listener)
                else:
                    self.get_logger().warn(
                        f"No recognized ROS message type for sensor '{sensor_type}'. Not publishing."
                    )

                # Handle attached pseudo-actors if any
                attached_objects = sensor_def.get("attached_objects", [])
                for ao in attached_objects:
                    self.get_logger().info(f"Detected attached object (pseudo) {ao['type']} with id '{ao['id']}'")
                    # Implement handling of attached objects if necessary.

            except Exception as e:
                self.get_logger().error(f"Error spawning sensor '{sensor_def}': {e}")

    def get_ros_topic_and_type(self, sensor_type, sensor_id):
        """
        Returns a (topic_name, message_type) for the given CARLA sensor type.
        Modify naming as you prefer for your Foxglove setup.
        """
        if sensor_type.startswith("sensor.camera"):
            return (f"/carla/{sensor_id}/image_raw", Image)
        elif sensor_type.startswith("sensor.lidar"):
            return (f"/carla/{sensor_id}/points", PointCloud2)
        elif sensor_type.startswith("sensor.other.imu"):
            return (f"/carla/{sensor_id}/imu", Imu)
        elif sensor_type.startswith("sensor.other.gnss"):
            return (f"/carla/{sensor_id}/gnss", NavSatFix)
        else:
            return (None, None)

    def sensor_data_callback(self, actor_id, data):
        """
        Single callback for all sensors. We look up the sensor_type to figure out how to convert.
        """
        if actor_id not in self.sensors_publishers:
            return

        pub_info = self.sensors_publishers[actor_id]
        sensor_type = pub_info["sensor_type"]
        publisher = pub_info["publisher"]
        msg_type = pub_info["msg_type"]

        # Create a header for the message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = pub_info["sensor_id"] 

        # Convert CARLA data to ROS message using sensors_data module
        if sensor_type.startswith("sensor.camera"):
            msg = carla_image_to_ros_image(data, header)
        elif sensor_type.startswith("sensor.lidar"):
            msg = carla_lidar_to_ros_pointcloud2(data, header)
        elif sensor_type.startswith("sensor.other.imu"):
            msg = carla_imu_to_ros_imu(data, header)
        elif sensor_type.startswith("sensor.other.gnss"):
            msg = carla_gnss_to_ros_navsatfix(data, header)
        else:
            msg = None

        if msg:
            publisher.publish(msg)

    def _monitor_vehicle_position(self):
        """Background thread to monitor the vehicle's position and detect problems"""
        import time
        import rclpy  # Import rclpy for ROS2
        
        # Get the map
        carla_map = self.world.get_map()
        
        # First, let's understand the junction area by finding the start and end waypoints
        start_waypoints = []
        end_waypoints = []
        junction_waypoints = []
        
        all_waypoints = carla_map.generate_waypoints(0.5)  # Generate dense waypoints
        for wp in all_waypoints:
            if wp.road_id == 1 and wp.lane_id == -1:
                if 0 <= wp.s <= 10:  # Start of track
                    start_waypoints.append(wp)
                elif wp.s >= 580:     # End of track
                    end_waypoints.append(wp)
                    
                # Find the junction waypoints
                if wp.is_junction:
                    junction_waypoints.append(wp)
        
        # Draw the junction area with a highly visible markers
        for wp in junction_waypoints:
            # Draw a red sphere at each junction waypoint
            self.world.debug.draw_point(
                wp.transform.location + carla.Location(z=1.0),
                size=0.3,
                color=carla.Color(255, 0, 0),  # Red
                life_time=0.0  # Permanent
            )
            
            # Add label with s-value
            self.world.debug.draw_string(
                wp.transform.location + carla.Location(z=1.5),
                f"JUNCTION s={wp.s:.1f}",
                draw_shadow=True,
                color=carla.Color(255, 0, 0),
                life_time=0.0
            )
        
        self.get_logger().info(f"Found {len(junction_waypoints)} junction waypoints")
        
        # Draw connection line between last waypoint and first waypoint
        if end_waypoints and start_waypoints:
            last_wp = end_waypoints[-1]
            first_wp = start_waypoints[0]
            
            # Draw a line connecting them
            self.world.debug.draw_line(
                last_wp.transform.location + carla.Location(z=0.5),
                first_wp.transform.location + carla.Location(z=0.5),
                thickness=0.2,
                color=carla.Color(255, 0, 255),  # Magenta
                life_time=0.0
            )
            
            self.get_logger().info(f"Last waypoint: s={last_wp.s:.1f} at {last_wp.transform.location}")
            self.get_logger().info(f"First waypoint: s={first_wp.s:.1f} at {first_wp.transform.location}")
        
        # Variables for monitoring the vehicle's road position
        last_known_s = -1
        last_known_waypoint = None
        disappearance_detected = False
        last_valid_transform = None
        
        # Check vehicle position 20 times per second for more responsive detection
        check_interval = 0.05
        log_interval = 1.0  # Log position every second
        last_log_time = time.time()
        
        while not self.stop_monitoring and rclpy.ok():
            try:
                current_time = time.time()
                
                if self.vehicle is None or not self.vehicle.is_alive:
                    if not disappearance_detected:
                        self.get_logger().error("Vehicle no longer exists!")
                        if last_valid_transform:
                            self.get_logger().error(f"Last known position: {last_valid_transform.location}")
                            self.get_logger().error(f"Last known s value: {last_known_s}")
                            
                            # Mark the position where the car disappeared
                            self.world.debug.draw_point(
                                last_valid_transform.location,
                                size=1.0,
                                color=carla.Color(255, 0, 0),  # Red
                                life_time=0.0  # Permanent
                            )
                            
                            # Add label where vehicle disappeared
                            self.world.debug.draw_string(
                                last_valid_transform.location + carla.Location(z=2.0),
                                "VEHICLE DISAPPEARED HERE",
                                draw_shadow=True,
                                color=carla.Color(255, 0, 0),
                                life_time=0.0
                            )
                            
                        disappearance_detected = True
                    
                    # Try to respawn at a safe position
                    if disappearance_detected and last_known_waypoint:
                        self.get_logger().warn("Attempting to respawn the vehicle...")
                        self._teleport_to_track_start()
                        disappearance_detected = False
                        time.sleep(1.0)  # Wait before next check
                    break  # Exit the monitoring loop
                
                # Get current transform
                transform = self.vehicle.get_transform()
                last_valid_transform = transform
                
                # Find nearest waypoint to get s-value
                waypoint = carla_map.get_waypoint(transform.location)
                if waypoint and waypoint.road_id == 1:
                    last_known_waypoint = waypoint
                    last_known_s = waypoint.s
                    
                    # Log position at regular intervals
                    if current_time - last_log_time > log_interval:
                        velocity = self.vehicle.get_velocity()
                        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # km/h
                        
                        self.get_logger().info(
                            f"Vehicle at: s={waypoint.s:.1f}, road={waypoint.road_id}, "
                            f"lane={waypoint.lane_id}, junction={waypoint.is_junction}, "
                            f"pos=({transform.location.x:.2f}, {transform.location.y:.2f}), "
                            f"speed={speed:.1f} km/h"
                        )
                        last_log_time = current_time
                    
                    # IMPORTANT: Check s-value to teleport BEFORE vehicle reaches problematic area
                    # We set the threshold at s=570, which is approximately 15-20m before the end
                    if waypoint.s >= 560:
                        self.get_logger().warn(f"!!! TELEPORT THRESHOLD REACHED at s={waypoint.s:.1f}")
                        
                        # Visual indicator
                        self.world.debug.draw_point(
                            transform.location,
                            size=0.7,
                            color=carla.Color(0, 255, 255),  # Cyan
                            life_time=30.0
                        )
                        
                        # Teleport to start of track
                        self._teleport_to_track_start()
                        time.sleep(1.0)  # Wait before next check
                
                time.sleep(check_interval)
                
            except Exception as e:
                self.get_logger().error(f"Error in position monitoring: {e}")
                time.sleep(1.0)  # Sleep longer on error

    def _teleport_to_track_start(self):
        """Teleport the vehicle back to the start of the track (around s=5)"""
        try:
            self.get_logger().warn("Teleporting to track start")
            
            # Disable autopilot
            self.vehicle.set_autopilot(False)
            
            # Find a waypoint near the start of the track
            carla_map = self.world.get_map()
            start_waypoints = []
            
            # Generate waypoints and find those at the start of the track (s between 3 and 10)
            all_waypoints = carla_map.generate_waypoints(1.0)  # More dense waypoints
            for wp in all_waypoints:
                if wp.road_id == 1 and wp.lane_id == -1 and 5.0 <= wp.s <= 7.0:  # More precise range
                    start_waypoints.append(wp)
            
            if not start_waypoints:
                self.get_logger().error("Could not find valid start waypoint for teleportation")
                return
                
            # Sort them by s-value to get a clean sequence
            start_waypoints.sort(key=lambda wp: wp.s)
            target_wp = start_waypoints[0]  # Use the first one for consistency
            
            # Create the destination transform
            new_transform = target_wp.transform
            new_transform.location.z += 0.5  # Raise slightly above road
            
            self.get_logger().info(f"Teleporting to s={target_wp.s:.1f} at {new_transform.location}")
            
            # Draw teleport destination marker
            self.world.debug.draw_point(
                new_transform.location,
                size=0.7,
                color=carla.Color(0, 255, 0),  # Green
                life_time=30.0
            )
            
            # Perform teleportation
            self.vehicle.set_transform(new_transform)
            
            import time
            time.sleep(0.5)  # Let physics settle
            
            # Apply control to ensure continued movement
            control = carla.VehicleControl()
            control.throttle = 0.7  # Strong throttle
            control.steer = 0.0
            control.brake = 0.0
            control.hand_brake = False
            self.vehicle.apply_control(control)
            
            time.sleep(0.5)  # Apply control briefly
            
            # Re-enable autopilot
            self.get_logger().info("Re-enabling autopilot")
            tm_port = self.traffic_manager.get_port()
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 0)
            self.vehicle.set_autopilot(True, tm_port)
            
        except Exception as e:
            self.get_logger().error(f"ERROR during teleport to track start: {e}")

    def destroy_node(self):
        """Clean up resources when the node is shutting down"""
        if hasattr(self, 'stop_monitoring'):
            self.stop_monitoring = True
        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        # Destroy vehicle and sensors if they exist
        if hasattr(self, 'vehicle') and self.vehicle is not None and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.get_logger().info("Vehicle destroyed")
            
        for sensor in getattr(self, 'spawned_sensors', []):
            if sensor is not None and sensor.is_alive:
                sensor.destroy()
        
        self.get_logger().info("All actors cleaned up")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpawnVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()