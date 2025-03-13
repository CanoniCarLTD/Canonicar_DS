import random
import carla
import json

def get_vehicle_blueprints():
    try:
        # Connect to CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)  # Set a timeout in seconds
        
        # Get the world and blueprint library
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        result = {}
        # Filter vehicle blueprints
        vehicles = [bp.id for bp in blueprint_library.filter('vehicle.*')]
        for vehicle_type in vehicles:
            vehicle_bp = blueprint_library.find(vehicle_type)
            map = world.get_map()
            spawn_points = map.get_spawn_points()
            random.shuffle(spawn_points)
            spawn_point = spawn_points[0]
            vehicle_actor = world.spawn_actor(vehicle_bp, spawn_point)
            vehicle_actor.set_autopilot(False)
            physics_control = vehicle_actor.get_physics_control()
            wheels = physics_control.wheels

            data = {
                "mass": float(physics_control.mass),
                "drag_coefficient": float(physics_control.drag_coefficient),
                "center_of_mass_x": float(physics_control.center_of_mass.x),
                "center_of_mass_y": float(physics_control.center_of_mass.y),
                "center_of_mass_z": float(physics_control.center_of_mass.z),
                "max_rpm": float(physics_control.max_rpm),
                "moi": float(physics_control.moi),
                "clutch_strength": float(physics_control.clutch_strength),
                "gear_switch_time": float(physics_control.gear_switch_time),
                "wheels": [
                    {
                        "tire_friction": float(wheel.tire_friction),
                        "damping_rate": float(wheel.damping_rate),
                        "max_steer_angle": float(wheel.max_steer_angle),
                        "radius": float(wheel.radius),
                        "max_brake_torque": float(wheel.max_brake_torque),
                        "max_handbrake_torque": float(wheel.max_handbrake_torque),
                        "lat_stiff_max_load": float(wheel.lat_stiff_max_load),
                        "lat_stiff_value": float(wheel.lat_stiff_value),
                        "long_stiff_value": float(wheel.long_stiff_value),
                        "position": {
                            "x": float(wheel.position.x),
                            "y": float(wheel.position.y),
                            "z": float(wheel.position.z)
                        }
                    } for wheel in wheels
                ]
            }
            vehicle_actor.destroy()
            result[vehicle_type] = data
        
        # Ensure all values are actually float type
        return result
    
    except Exception as e:
        print(f"Error: {e}")
        return []

if __name__ == "__main__":
    vehicle_blueprints = get_vehicle_blueprints()

    # Save to JSON file
    with open("vehicle_blueprints2.json", "w") as json_file:
        json.dump(vehicle_blueprints, json_file, indent=4)
    
    print("Available vehicle blueprints saved to vehicle_blueprints.json")