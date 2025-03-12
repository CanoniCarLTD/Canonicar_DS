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
        
        # Filter vehicle blueprints
        vehicles = [bp.id for bp in blueprint_library.filter('vehicle.*')]
        
        return vehicles
    except Exception as e:
        print(f"Error: {e}")
        return []

if __name__ == "__main__":
    vehicle_blueprints = get_vehicle_blueprints()
    
    # Save to JSON file
    with open("vehicle_blueprints.json", "w") as json_file:
        json.dump(vehicle_blueprints, json_file, indent=4)
    
    print("Available vehicle blueprints saved to vehicle_blueprints.json")