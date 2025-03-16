import rclpy
from rclpy.node import Node
import carla
from carla import Client, Transform, Location, Rotation, TrafficManager
import json
import os
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
import time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

# For ROS2 messages
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from std_msgs.msg import Header


# Import conversion functions from sensors_data
from sensors_data import (
    carla_image_to_ros_image,
    carla_lidar_to_ros_pointcloud2,
    carla_imu_to_ros_imu,
    carla_gnss_to_ros_navsatfix,
)


class SpawnVehicleNode(Node):
    def __init__(self):
        super().__init__("spawn_vehicle_node")

        self.declare_parameter("host", "")
        self.declare_parameter("port", 2000)
        self.declare_parameter(
            "vehicle_type", "vehicle.tesla.model3"
        )  # Default vehicle type

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.vehicle_type = self.get_parameter(
            "vehicle_type"
        ).value  # Read vehicle type

        self.get_logger().info(
            f"Connecting to CARLA server at {self.host}:{self.port} with vehicle {self.vehicle_type}"
        )

        try:
            self.client = Client(self.host, 2000)
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
            get_package_share_directory("client_node"),
            "client_node",
            "sensors_config.json",
        )

        self.physics_publisher = self.create_publisher(
            Float32MultiArray, "/carla/vehicle/physics", 10
        )  # To data_collector node
        self.timer = self.create_timer(0.1, self.publish_vehicle_physics)
        self.control_publisher = self.create_publisher(
            Float32MultiArray, "/carla/vehicle/control", 10
        )  # To data_collector node
        self.timer = self.create_timer(0.1, self.publish_vehicle_control)
        self.location_publisher = self.create_publisher(
            Float32MultiArray, "/carla/vehicle/location", 10
        )  # To data_process node
        self.timer = self.create_timer(0.1, self.publish_vehicle_location)
        self.vehicle_type_publisher = self.create_publisher(
            String, "/carla/vehicle/type", 10
        )  # To data_collector node

        self.vehicle = None

        self.lap_subscription = self.create_subscription(
            String,
            "/lap_completed",  # Topic name for lap completion
            self.lap_callback,  # Callback function
            10,  # QoS
        )
        self.start_subscription = self.create_subscription(
            String, "/start_vehicle_manager", self.start_driving, 10  # QoS
        )
        self.data_collector_ready = False
        self.map_loaded = False

        self.vehicle_types = [
            "vehicle.yamaha.yzf",
            "vehicle.carlamotors.carlacola",
            "vehicle.mini.cooper_s_2021",
            "vehicle.mercedes.sprinter",
            "vehicle.ford.ambulance",
            "vehicle.kawasaki.ninja",
            "vehicle.carlamotors.firetruck",
            "vehicle.ford.mustang",
            "vehicle.volkswagen.t2_2021",
            "vehicle.vespa.zx125",
            "vehicle.lincoln.mkz_2020",
            "vehicle.toyota.prius",
            "vehicle.harley-davidson.low_rider",
            "vehicle.audi.tt",
            "vehicle.dodge.charger_2020",
            "vehicle.nissan.patrol_2021",
            "vehicle.tesla.cybertruck",
        ]

        # self.vehicle_types = [
        # "vehicle.toyota.prius",
        # "vehicle.vespa.zx125",
        # "vehicle.vespa.zx125",
        # "vehicle.ford.ambulance",
        # "vehicle.volkswagen.t2_2021",
        # "vehicle.ford.mustang",
        # "vehicle.mercedes.sprinter",
        # "vehicle.mini.cooper_s_2021",
        # "vehicle.carlamotors.carlacola",
        # "vehicle.audi.tt",
        # "vehicle.harley-davidson.low_rider",
        # "vehicle.mercedes.sprinter",
        # "vehicle.mini.cooper_s_2021",
        # "vehicle.lincoln.mkz_2020",
        # "vehicle.yamaha.yzf",
        # "vehicle.dodge.charger_2020",
        # "vehicle.volkswagen.t2_2021",
        # "vehicle.kawasaki.ninja",
        # "vehicle.lincoln.mkz_2020",
        # "vehicle.volkswagen.t2_2021",
        # "vehicle.mercedes.sprinter",
        # "vehicle.carlamotors.carlacola",
        # "vehicle.dodge.charger_2020",
        # "vehicle.carlamotors.carlacola",
        # "vehicle.nissan.patrol_2021",
        # "vehicle.mercedes.sprinter",
        # "vehicle.ford.mustang",
        # "vehicle.volkswagen.t2_2021",
        # "vehicle.dodge.charger_2020",
        # "vehicle.nissan.patrol_2021",
        # "vehicle.carlamotors.firetruck",
        # "vehicle.lincoln.mkz_2020",
        # "vehicle.toyota.prius",
        # "vehicle.mini.cooper_s_2021",
        # "vehicle.audi.tt",
        # "vehicle.mini.cooper_s_2021",
        # "vehicle.lincoln.mkz_2020",
        # "vehicle.audi.tt",
        # "vehicle.vespa.zx125",
        # "vehicle.carlamotors.carlacola",
        # "vehicle.harley-davidson.low_rider",
        # "vehicle.harley-davidson.low_rider",
        # "vehicle.ford.mustang",
        # "vehicle.nissan.patrol_2021",
        # "vehicle.harley-davidson.low_rider",
        # "vehicle.dodge.charger_2020",
        # "vehicle.carlamotors.firetruck",
        # "vehicle.ford.ambulance",
        # "vehicle.ford.ambulance",
        # "vehicle.kawasaki.ninja",
        # "vehicle.vespa.zx125",
        # "vehicle.carlamotors.firetruck",
        # "vehicle.yamaha.yzf",
        # "vehicle.nissan.patrol_2021",
        # "vehicle.audi.tt",
        # "vehicle.toyota.prius",
        # "vehicle.toyota.prius",
        # "vehicle.carlamotors.firetruck",
        # "vehicle.ford.ambulance",
        # "vehicle.ford.mustang",
        # "vehicle.tesle.cybertruck",
        # ]

        # self.vehicle_types = [
        #     "vehicle.audi.a2",
        #     "vehicle.citroen.c3",
        #     "vehicle.micro.microlino",
        #     "vehicle.dodge.charger_police",
        #     "vehicle.audi.tt",
        #     "vehicle.jeep.wrangler_rubicon",
        #     "vehicle.mercedes.coupe",
        #     "vehicle.mercedes.coupe_2020",
        #     "vehicle.harley-davidson.low_rider",
        #     "vehicle.dodge.charger_2020",
        #     "vehicle.ford.ambulance",
        #     "vehicle.lincoln.mkz_2020",
        #     "vehicle.mini.cooper_s_2021",
        #     "vehicle.toyota.prius",
        #     "vehicle.ford.crown",
        #     "vehicle.carlamotors.carlacola",
        #     "vehicle.vespa.zx125",
        #     "vehicle.nissan.patrol_2021",
        #     "vehicle.dodge.charger_police_2020",
        #     "vehicle.mercedes.sprinter",
        #     "vehicle.audi.etron",
        #     "vehicle.seat.leon",
        #     "vehicle.volkswagen.t2_2021",
        #     "vehicle.tesla.cybertruck",
        #     "vehicle.lincoln.mkz_2017",
        #     "vehicle.ford.mustang",
        #     "vehicle.carlamotors.firetruck",
        #     "vehicle.volkswagen.t2",
        #     "vehicle.diamondback.century",
        #     "vehicle.gazelle.omafiets",
        #     "vehicle.bmw.grandtourer",
        #     "vehicle.bh.crossbike",
        #     "vehicle.kawasaki.ninja",
        #     "vehicle.yamaha.yzf",
        #     "vehicle.nissan.patrol",
        #     "vehicle.nissan.micra",
        #     "vehicle.mini.cooper_s"
        #     ]

        self.current_vehicle_index = 0  # To keep track of which vehicle to spawn next
        self.spawn_objects_from_config()

    def start_driving(self, msg):
        self.get_logger().info(f"Received start signal: {msg.data}")
        if msg.data == "Map is loaded":
            self.map_loaded = True
        if msg.data == "DataCollector is ready":
            self.data_collector_ready = True
        if self.map_loaded and self.data_collector_ready:
            self.get_logger().info("Starting to drive")
            self.spawn_objects_from_config()
        self.get_logger().info(
            f"Is map loaded?: {self.map_loaded}, Is data collector ready {self.data_collector_ready}"
        )

    def lap_callback(self, msg):
        self.data_collector_ready = False
        self.get_logger().info(f"Lap completed! Destroy vehicle {self.vehicle_type}")
        self.destroy_actors()
        self.vehicle_type = self.vehicle_types[self.current_vehicle_index]
        self.current_vehicle_index = (self.current_vehicle_index + 1) % len(
            self.vehicle_types
        )

    def publish_vehicle_location(self):
        if self.vehicle is not None and self.vehicle.is_alive:
            location = self.vehicle.get_location()
            msg = Float32MultiArray()
            msg.data = [location.x, location.y, location.z]
            self.location_publisher.publish(msg)

    def publish_vehicle_control(self):
        if self.vehicle is not None and self.vehicle.is_alive:
            control = self.vehicle.get_control()
            msg = Float32MultiArray()
            msg.data = [control.throttle, control.steer, control.brake]
            self.control_publisher.publish(msg)

    def publish_vehicle_physics(self):
        if self.vehicle is not None and self.vehicle.is_alive:
            physics_control = self.vehicle.get_physics_control()

            msg = Float32MultiArray()
            msg.data = [physics_control.mass, physics_control.drag_coefficient]
            self.physics_publisher.publish(msg)

    def spawn_objects_from_config(self):

        # Add delay to ensure map is fully loaded
        self.get_logger().info("Waiting for map to fully load...")
        time.sleep(2.0)
        self.get_logger().info("Spawning vehicle type " + self.vehicle_type)

        try:
            # Load vehicle configuration
            with open(self.sensor_config_file, "r") as f:
                config = json.load(f)
                self.get_logger().info("Loaded JSON config")

            # Get vehicle blueprint
            objects = config.get("objects", [])
            if not objects:
                self.get_logger().error("No objects found in sensors_config.json")
                return

            ego_object = objects[0]
            vehicle_type = self.vehicle_type
            if not vehicle_type.startswith("vehicle."):
                self.get_logger().error("No valid vehicle object found in JSON.")
                return

            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(vehicle_type)

            carla_map = self.world.get_map()
            waypoints = carla_map.generate_waypoints(
                2.0
            )  # 1-meter interval for precision

            road_ids = set(wp.road_id for wp in waypoints)
            if len(road_ids) == 0:
                self.get_logger().error("No roads found in the map!")
                return

            main_road_id = list(road_ids)[0] if len(road_ids) == 1 else min(road_ids)

            # Find lanes on this road (prioritize negative lane IDs for left-side driving)
            lane_ids = set(wp.lane_id for wp in waypoints if wp.road_id == main_road_id)
            driving_lane_id = next(
                (id for id in lane_ids if id < 0),
                next((id for id in lane_ids if id > 0), 0),
            )

            if driving_lane_id == 0:
                self.get_logger().error("No valid driving lane found!")
                return

            # Find waypoints specifically for this road and lane
            road_waypoints = [
                wp
                for wp in waypoints
                if wp.road_id == main_road_id and wp.lane_id == driving_lane_id
            ]
            if not road_waypoints:
                self.get_logger().error(
                    "No waypoints found for the selected road/lane!"
                )
                return

            # Sort waypoints by s-value to ensure they're in order along the road
            road_waypoints.sort(key=lambda wp: wp.s)

            spawn_waypoint = road_waypoints[3]
            self.get_logger().info(
                f"Using waypoint at s={spawn_waypoint.s:.1f} for spawn"
            )

            # Create the transform and raise it slightly to prevent ground collision
            spawn_transform = spawn_waypoint.transform
            spawn_transform.location.z += 0.3

            # Try to spawn the vehicle
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
            if not self.vehicle:
                self.get_logger().error("Failed to spawn at waypoint")
                return

            location = self.vehicle.get_location()
            self.get_logger().info(f"Spawned vehicle at {location}")

            # Add collision sensor for debugging
            collision_bp = blueprint_library.find("sensor.other.collision")
            if collision_bp:
                collision_sensor = self.world.spawn_actor(
                    collision_bp, carla.Transform(), attach_to=self.vehicle
                )
                collision_sensor.listen(lambda event: self._on_collision(event))
                self.spawned_sensors.append(collision_sensor)

            # Configure physics for stability
            physics_control = self.vehicle.get_physics_control()
            physics_control.mass = physics_control.mass * 1.5
            self.vehicle.apply_physics_control(physics_control)
            # Spawn sensors
            sensors = ego_object.get("sensors", [])
            if sensors:
                self.spawn_sensors(sensors)

            # Configure autopilot
            self.traffic_manager.global_percentage_speed_difference(0)
            self.traffic_manager.auto_lane_change(self.vehicle, False)
            self.traffic_manager.random_left_lanechange_percentage(self.vehicle, 0)
            self.traffic_manager.random_right_lanechange_percentage(self.vehicle, 0)
            self.vehicle.set_autopilot(True, self.traffic_manager.get_port())

            # Publish vehicle type for data process
            request_msg = String()
            request_msg.data = self.vehicle_type
            self.vehicle_type_publisher.publish(request_msg)

        except Exception as e:
            self.get_logger().error(f"Error spawning vehicle: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())

    def _on_collision(self, event):
        """Callback for collision sensor to help debug vehicle disappearances"""
        collision_type = event.other_actor.type_id if event.other_actor else "unknown"
        collision_location = event.transform.location

        self.get_logger().warn(
            f"COLLISION: vehicle hit {collision_type} at "
            f"({collision_location.x:.1f}, {collision_location.y:.1f}, {collision_location.z:.1f})"
        )

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
                    self.get_logger().error(
                        f"Sensor blueprint '{sensor_type}' not found. Skipping."
                    )
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

                sensor_actor = self.world.try_spawn_actor(
                    sensor_bp, spawn_transform, attach_to=self.vehicle
                )
                if sensor_actor is None:
                    self.get_logger().error(f"Failed to spawn sensor '{sensor_id}'.")
                    continue
                else:
                    self.spawned_sensors.append(sensor_actor)

                self.get_logger().info(
                    f"Spawned sensor '{sensor_id}' ({sensor_type}) at {spawn_transform}"
                )

                # Create a ROS2 publisher for this sensor
                topic_name, msg_type = self.get_ros_topic_and_type(
                    sensor_type, sensor_id
                )
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
                    self.get_logger().info(
                        f"Detected attached object (pseudo) {ao['type']} with id '{ao['id']}'"
                    )
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

    def destroy_actors(self):
        """
        Clean up all spawned actors (vehicle and sensors) when node shuts down.
        """
        self.get_logger().info("Destroying all spawned actors...")

        try:
            # Destroy all sensors first
            for sensor in self.spawned_sensors:
                if sensor is not None and sensor.is_alive:
                    sensor.stop()
                    sensor.destroy()
                    self.get_logger().debug(f"Destroyed sensor {sensor.id}")
            self.spawned_sensors.clear()

            # Then destroy the vehicle
            if self.vehicle is not None and self.vehicle.is_alive:
                # Disable autopilot before destroying
                try:
                    self.vehicle.set_autopilot(False)
                except:
                    pass

                self.vehicle.destroy()
                self.get_logger().info(f"Destroyed vehicle {self.vehicle.id}")
                self.vehicle = None

            self.get_logger().info("All actors destroyed successfully")

        except Exception as e:
            self.get_logger().error(f"Error during actor cleanup: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SpawnVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_actors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
