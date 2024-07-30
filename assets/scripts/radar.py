import tkinter as tk
from tkinter import ttk
import requests
import math
import os
import time
from datetime import datetime

# Initialize zero point coordinates
zero_point = {'coordinateX': 0, 'coordinateY': 0, 'coordinateZ': 0}
zoom_level = 1
update_interval = 100  # Update interval in milliseconds
save_interval = 1000  # Save interval in milliseconds (1 second)
zero_point_set = False
canvas_size = 1000  # GBPPlanner world size
zoom_factor = 3.44  # new_image/old_image = 1000/688 = 1.453   old_scaling/new_scaling = 5/1.453 = 3.44
is_recording = False
recorded_waypoints = []

# List of truck URLs
truck_urls = [
    'http://192.168.1.49:39847',
    # Add more URLs here as needed
]

# Dictionary to store the last known coordinates for each robot
last_known_coordinates = {}

# Function to start recording waypoints
def start_recording():
    global is_recording
    is_recording = True
    recorded_waypoints.clear()
    print("Started recording waypoints")

# Function to stop recording and save waypoints
def stop_recording():
    global is_recording
    is_recording = False
    if recorded_waypoints:
        save_recorded_waypoints()
    print("Stopped recording waypoints")

# Function to save recorded waypoints
def save_recorded_waypoints():
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"waypoints_{current_time}.txt"
    try:
        os.chdir("../../build")  # Change to the correct directory
        with open(filename, "a") as f:
            for waypoint in recorded_waypoints:
                f.write(f"{waypoint['robot']} {waypoint['x']} {waypoint['y']}\n")
        print(f"Saved {len(recorded_waypoints)} waypoints to {filename}")
    except Exception as e:
        print(f"Error saving waypoints: {e}")
    finally:
        os.chdir(original_dir)  # Change back to the original directory


# Function to fetch data from the localhost API
def fetch_data(url):
    try:
        response = requests.get(url, timeout=0.5)
        data = response.json()
        truck_placement = data['api']['truckPlacement']
        return truck_placement
    except Exception as e:
        print(f"Error fetching data from {url}: {e}")
        return None

# Function to update the UI with fetched data
def update_data():
    global zero_point_set, is_recording
    truck_data = []
    for i, url in enumerate(truck_urls):
        data = fetch_data(url)
        if data:
            truck_data.append(data)
            last_known_coordinates[i] = {
                'coordinateX': data['coordinateX'],
                'coordinateZ': data['coordinateZ']
            }
            
            # Record waypoint if recording is active
            if is_recording and zero_point_set:
                x = (data['coordinateX'] - zero_point['coordinateX']) * 10 / zoom_factor
                y = (data['coordinateZ'] - zero_point['coordinateZ']) * 10 / zoom_factor
                rotated_x, rotated_y = rotate_90_degrees_clockwise(x, y)
                recorded_waypoints.append({
                    'robot': i+1,
                    'x': round(rotated_x, 2),
                    'y': round(rotated_y, 2)
                })
        elif i in last_known_coordinates:
            truck_data.append(last_known_coordinates[i])
        else:
            truck_data.append({
                'coordinateX': zero_point['coordinateX'],
                'coordinateZ': zero_point['coordinateZ']
            })
    
    if truck_data:
        update_ui(truck_data)
        update_radar(truck_data)
        
        # Save coordinates of all robots if zero point has been set
        if zero_point_set:
            save_robot_coordinates(truck_data)

    root.after(update_interval, update_data)


# Function to update the UI elements
def update_ui(truck_data):
    for i, data in enumerate(truck_data):
        relative_coordinateX = round(data['coordinateX'] - zero_point['coordinateX'], 2)
        relative_coordinateZ = round(data['coordinateZ'] - zero_point['coordinateZ'], 2)

        if i >= len(truck_labels):
            create_truck_labels(i)

        truck_labels[i]['coordinateX'].set(f"Robot {i+1} Coordinate X: {relative_coordinateX}")
        truck_labels[i]['coordinateZ'].set(f"Robot {i+1} Coordinate Z: {relative_coordinateZ}")
        if 'rotationX' in data:
            truck_labels[i]['rotationX'].set(f"Robot {i+1} Rotation X: {round(math.degrees(data['rotationX']), 2)}°")
            truck_labels[i]['rotationY'].set(f"Robot {i+1} Rotation Y: {round(math.degrees(data['rotationY']), 2)}°")
            truck_labels[i]['rotationZ'].set(f"Robot {i+1} Rotation Z: {round(math.degrees(data['rotationZ']), 2)}°")
        else:
            truck_labels[i]['rotationX'].set(f"Robot {i+1} Rotation X: N/A")
            truck_labels[i]['rotationY'].set(f"Robot {i+1} Rotation Y: N/A")
            truck_labels[i]['rotationZ'].set(f"Robot {i+1} Rotation Z: N/A")

# Function to update the radar view
def update_radar(truck_data):
    radar_canvas.delete("all")
    radar_canvas.create_oval(250-5, 250-5, 250+5, 250+5, fill="green")  # Center point
    
    colors = ["red", "blue", "yellow", "purple", "orange"]  # Add more colors if needed
    
    for i, data in enumerate(truck_data):
        x = (data['coordinateX'] - zero_point['coordinateX']) * zoom_level
        z = (data['coordinateZ'] - zero_point['coordinateZ']) * zoom_level
        
        # Applying rotation (90 degrees clockwise)
        x_rotated, z_rotated = rotate_90_degrees_clockwise(x, z)

        color = colors[i % len(colors)]
        radar_canvas.create_oval(250+x_rotated-5, 250+z_rotated-5, 250+x_rotated+5, 250+z_rotated+5, fill=color)

# Function to reset the zero point coordinates relative to the first robot
def reset_zero_point():
    global zero_point, zero_point_set
    data = fetch_data(truck_urls[0])
    if data:
        zero_point['coordinateX'] = data['coordinateX']
        zero_point['coordinateY'] = data['coordinateY']
        zero_point['coordinateZ'] = data['coordinateZ']

        zero_point_label.set(f"Zero Point Set to: ({round(zero_point['coordinateX'], 2)}, {round(zero_point['coordinateY'], 2)}, {round(zero_point['coordinateZ'], 2)})")
        zero_point_set = True

# Function to rotate coordinates 90 degrees clockwise
def rotate_90_degrees_clockwise(x, y):
    return y, -x


def save_robot_coordinates(truck_data):
    try:
        os.chdir("../../build")  # Change to the correct directory
        with open("robot_coordinates.txt", "w") as f:
            for i, data in enumerate(truck_data):
                # Calculate coordinates using the same method as drawMap.py
                x = (data['coordinateX'] - zero_point['coordinateX']) * 10 / zoom_factor
                y = (data['coordinateZ'] - zero_point['coordinateZ']) * 10 / zoom_factor
                
                # Rotate coordinates 90 degrees clockwise
                rotated_x, rotated_y = rotate_90_degrees_clockwise(x, y)
                
                # Round the coordinates to 2 decimal places
                rotated_x = round(rotated_x, 2)
                rotated_y = round(rotated_y, 2)
                
                f.write(f"{i+1} {rotated_x} {rotated_y}\n")
        print(f"Saved scaled and rotated coordinates for {len(truck_data)} robots")
    except Exception as e:
        print(f"Error saving coordinates: {e}")
    finally:
        os.chdir(original_dir)  # Change back to the original directory

# Functions for zooming
def zoom_in():
    global zoom_level
    zoom_level *= 1.2
    update_data()

def zoom_out():
    global zoom_level
    zoom_level /= 1.2
    update_data()

# Setting up the Tkinter window
root = tk.Tk()
root.title("Robot Placement and Rotation Data")

# Store the original directory
original_dir = os.getcwd()

# List to store label dictionaries for each robot
truck_labels = []

# Function to create labels for a new robot
def create_truck_labels(index):
    labels = {
        'coordinateX': tk.StringVar(),
        'coordinateZ': tk.StringVar(),
        'rotationX': tk.StringVar(),
        'rotationY': tk.StringVar(),
        'rotationZ': tk.StringVar()
    }
    for key in labels:
        ttk.Label(root, textvariable=labels[key]).pack()
    truck_labels.append(labels)

# Creating the radar canvas
radar_canvas = tk.Canvas(root, width=500, height=500, bg="white")
radar_canvas.pack()

# Label to show the zero point coordinates
zero_point_label = tk.StringVar(value="Zero Point not set")
ttk.Label(root, textvariable=zero_point_label).pack()

# Buttons
reset_button = ttk.Button(root, text="Reset Zero Point", command=reset_zero_point)
reset_button.pack()

zoom_in_button = ttk.Button(root, text="Zoom In", command=zoom_in)
zoom_in_button.pack()

zoom_out_button = ttk.Button(root, text="Zoom Out", command=zoom_out)
zoom_out_button.pack()

start_button = ttk.Button(root, text="Start Recording", command=start_recording)
start_button.pack()

stop_button = ttk.Button(root, text="Stop Recording", command=stop_recording)
stop_button.pack()
# Initial call to update the data
update_data()

# Running the Tkinter event loop
root.mainloop()