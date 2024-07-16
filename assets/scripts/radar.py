import tkinter as tk
from tkinter import ttk
import requests
import math
import os
import time

# Initialize zero point coordinates
zero_point = {'coordinateX': 0, 'coordinateY': 0, 'coordinateZ': 0}
zoom_level = 1
update_interval = 200  # Update interval in milliseconds
save_interval = 1000  # Save interval in milliseconds (1 second)
zero_point_set = False

# List of truck URLs
truck_urls = [
    'http://192.168.1.86:39847',
    # 'http://192.168.1.150:39847',
    # Add more URLs here as needed
]

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
    global zero_point_set
    truck_data = []
    for url in truck_urls:
        data = fetch_data(url)
        if data:
            truck_data.append(data)
    
    if truck_data:
        update_ui(truck_data)
        update_radar(truck_data)
        
        # Save coordinates of the first robot if zero point has been set
        if zero_point_set and truck_data[0]:
            save_robot_coordinates(truck_data[0])

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
        truck_labels[i]['rotationX'].set(f"Robot {i+1} Rotation X: {round(math.degrees(data['rotationX']), 2)}°")
        truck_labels[i]['rotationY'].set(f"Robot {i+1} Rotation Y: {round(math.degrees(data['rotationY']), 2)}°")
        truck_labels[i]['rotationZ'].set(f"Robot {i+1} Rotation Z: {round(math.degrees(data['rotationZ']), 2)}°")

# Function to update the radar view
def update_radar(truck_data):
    radar_canvas.delete("all")
    radar_canvas.create_oval(250-5, 250-5, 250+5, 250+5, fill="green")  # Center point
    
    colors = ["red", "blue", "yellow", "purple", "orange"]  # Add more colors if needed
    
    for i, data in enumerate(truck_data):
        x = (data['coordinateX'] - zero_point['coordinateX']) * zoom_level
        z = (data['coordinateZ'] - zero_point['coordinateZ']) * zoom_level
        
        # Applying rotation (90 degrees clockwise)
        x_rotated = z
        z_rotated = -x

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

# Function to save robot coordinates to file
def save_robot_coordinates(data):
    try:
        os.chdir("../../build")  # Change to the build directory
        x = round(data['coordinateX'] - zero_point['coordinateX'], 2)
        y = round(data['coordinateZ'] - zero_point['coordinateZ'], 2)
        with open("robot_coordinates.txt", "w") as f:
            f.write(f"{x} {y}")
        print(f"Saved robot coordinates: ({x}, {y})")
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

# Initial call to update the data
update_data()

# Running the Tkinter event loop
root.mainloop()