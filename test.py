import json

# Load data from a JSON file
with open('vehicle_physics.json', 'r') as file:
    data = json.load(file)

for car, vehicle_data in data.items():
    if "wheels" in vehicle_data:
        wheels = vehicle_data.pop("wheels")
        wheel_keys = ["front_left", "front_right", "rear_left", "rear_right"]
        
        for i, key in enumerate(wheel_keys):
            wheel = wheels[i]
            for attr, value in wheel.items():
                if attr == "position":
                    for coord, coord_value in value.items():
                        vehicle_data[f"{key}_wheel_position_{coord}"] = coord_value
                else:
                    vehicle_data[f"{key}_wheel_{attr}"] = value

# Save the modified data back to a JSON file
with open('cars_modified.json', 'w') as file:
    json.dump(data, file, indent=4)

print("Processed data saved to cars_modified.json")