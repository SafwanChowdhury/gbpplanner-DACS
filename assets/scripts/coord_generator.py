import random
import time
import os

def generate_random_coordinates():
    world_size = 100  # Adjust based on your WORLD_SZ
    x = random.uniform(-world_size/2, world_size/2)
    y = random.uniform(-world_size/2, world_size/2)
    return x, y

def write_coordinates_to_file(x, y):
    with open("robot_coordinates.txt", "w") as f:
        f.write(f"{x} {y}")

# Ensure we're in the correct directory
os.chdir("../../build")  # Replace with the actual path

try:
    while True:
        x, y = generate_random_coordinates()
        print(f"Moving robot to coordinates: ({x:.2f}, {y:.2f})")
        write_coordinates_to_file(x, y)
        time.sleep(1)  # Adjust this delay as needed

except KeyboardInterrupt:
    print("\nStopping the coordinate generation.")