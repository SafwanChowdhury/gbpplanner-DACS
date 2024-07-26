import random
import time
import os
import math

def generate_random_coordinates(num_robots):
    world_size = 100
    coordinates = []
    for i in range(num_robots):
        x = random.uniform(-world_size/2, world_size/2)
        y = random.uniform(-world_size/2, world_size/2)
        coordinates.append((x, y))
    return coordinates

def rotate_90_degrees_clockwise(x, y):
    return y, -x

def write_coordinates_to_file(coordinates):
    try:
        os.chdir("../../build")  # Change to the correct directory
        with open("robot_coordinates.txt", "w") as f:
            for i, (x, y) in enumerate(coordinates):
                # Rotate coordinates 90 degrees clockwise
                rotated_x, rotated_y = rotate_90_degrees_clockwise(x, y)
                f.write(f"{i+1} {rotated_x:.2f} {rotated_y:.2f}\n")
        print(f"Saved rotated coordinates for {len(coordinates)} robots")
    except Exception as e:
        print(f"Error saving coordinates: {e}")
    finally:
        os.chdir(original_dir)  # Change back to the original directory

# Store the original directory
original_dir = os.getcwd()

# Number of robots to simulate
num_robots = 2  # Adjust this number as needed

try:
    while True:
        coordinates = generate_random_coordinates(num_robots)
        print("Moving robots to coordinates:")
        for i, (x, y) in enumerate(coordinates):
            print(f"Robot {i+1}: ({x:.2f}, {y:.2f})")
        write_coordinates_to_file(coordinates)
        time.sleep(1)  # Adjust this delay as needed

except KeyboardInterrupt:
    print("\nStopping the coordinate generation.")