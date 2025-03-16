# Description: This script reads vehicle physics data from a JSON file, processes it,
# and saves the modified data back to a JSON file.
# It reads the wheel data from the "wheels" key in the JSON file, extracts the relevant information, and adds it to the main vehicle data.
# This processed has been done only one time, so the data is saved to a new file called cars_modified.json.
# The script can be modified to read and process data from other JSON files as needed.

import json

with open("vehicle_physics.json", "r") as file:
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

with open("cars_modified.json", "w") as file:
    json.dump(data, file, indent=4)

print("Processed data saved to cars_modified.json")
