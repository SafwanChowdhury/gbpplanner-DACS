import tkinter as tk
from tkinter import ttk
import requests
import math
from PIL import Image, ImageDraw, ImageTk

# Initialize variables
zero_point = {'coordinateX': 0, 'coordinateZ': 0}
update_interval = 100  # Update interval in milliseconds
pen_down = False
truck_url = 'http://192.168.1.49:39847'  # URL for fetching truck data
canvas_size = 800
image = Image.new('RGB', (canvas_size, canvas_size), 'white')
draw = ImageDraw.Draw(image)
last_point = None
path = []
current_segment = []
zoom_factor = 5  # Adjust this value to change the zoom level (higher value = more zoomed out)
pen_width = 10  # Default pen width matched to approximate lane width

def fetch_data():
    """Fetch truck placement data from the API"""
    try:
        response = requests.get(truck_url, timeout=0.5)
        data = response.json()
        truck_placement = data['api']['truckPlacement']
        return truck_placement
    except Exception as e:
        print(f"Error fetching data: {e}")
        return None

def update_data():
    """Update the canvas with new truck position and path data"""
    global last_point, path, current_segment
    data = fetch_data()
    if data:
        # Calculate truck position on canvas
        x = int((data['coordinateX'] - zero_point['coordinateX']) * 10 / zoom_factor + canvas_size // 2)
        z = int((data['coordinateZ'] - zero_point['coordinateZ']) * 10 / zoom_factor + canvas_size // 2)
        
        # Create a copy of the image to draw on
        temp_image = image.copy()
        temp_draw = ImageDraw.Draw(temp_image)
        
        # Redraw the existing path
        for segment in path:
            if len(segment) > 1:
                temp_draw.line(segment, fill='black', width=pen_width)
        
        # Draw the current segment
        if len(current_segment) > 1:
            temp_draw.line(current_segment, fill='black', width=pen_width)
        
        # Draw the truck as a red dot
        temp_draw.ellipse([x-5, z-5, x+5, z+5], fill='red')
        
        if pen_down:
            current_segment.append((x, z))
        
        update_canvas(temp_image)
    
    root.after(update_interval, update_data)

def update_canvas(temp_image):
    """Update the canvas with the new image"""
    photo = ImageTk.PhotoImage(temp_image)
    canvas.create_image(0, 0, anchor=tk.NW, image=photo)
    canvas.image = photo

def toggle_pen_down():
    """Toggle the pen up/down state"""
    global pen_down, current_segment, path
    pen_down = not pen_down
    if pen_down:
        pen_button.config(text="Pen Up")
        current_segment = []  # Start a new segment
    else:
        pen_button.config(text="Pen Down")
        if current_segment:
            path.append(current_segment)  # Add the current segment to the path
            current_segment = []  # Clear the current segment

def reset_zero_point():
    """Reset the zero point to the current truck position"""
    global zero_point
    data = fetch_data()
    if data:
        zero_point['coordinateX'] = data['coordinateX']
        zero_point['coordinateZ'] = data['coordinateZ']
        zero_point_label.config(text=f"Zero Point: ({round(zero_point['coordinateX'], 2)}, {round(zero_point['coordinateZ'], 2)})")

def clear_drawing():
    """Clear the entire drawing"""
    global path, current_segment, image, draw
    path = []
    current_segment = []
    image = Image.new('RGB', (canvas_size, canvas_size), 'white')
    draw = ImageDraw.Draw(image)
    update_canvas(image)

def update_pen_width(value):
    """Update the pen width based on slider value"""
    global pen_width
    pen_width = int(float(value))
    pen_width_label.config(text=f"Pen Width: {pen_width}")

# Set up the main window
root = tk.Tk()
root.title("Waypoint Tracer")

# Create and pack the canvas
canvas_frame = ttk.Frame(root)
canvas_frame.pack(padx=10, pady=10)
canvas = tk.Canvas(canvas_frame, width=canvas_size, height=canvas_size, bg="white")
canvas.pack()

# Create and pack the control frame
control_frame = ttk.Frame(root)
control_frame.pack(padx=10, pady=10, fill=tk.X)

# Zero point label
zero_point_label = ttk.Label(control_frame, text="Zero Point not set")
zero_point_label.pack(side=tk.TOP, pady=5)

# Pen toggle button
pen_button = ttk.Button(control_frame, text="Pen Down", command=toggle_pen_down)
pen_button.pack(side=tk.LEFT, padx=5)

# Zero coordinates button
zero_button = ttk.Button(control_frame, text="Zero Coordinates", command=reset_zero_point)
zero_button.pack(side=tk.LEFT, padx=5)

# Clear drawing button
clear_button = ttk.Button(control_frame, text="Clear Drawing", command=clear_drawing)
clear_button.pack(side=tk.LEFT, padx=5)

# Pen width control
pen_width_frame = ttk.Frame(root)
pen_width_frame.pack(padx=10, pady=5, fill=tk.X)

pen_width_label = ttk.Label(pen_width_frame, text=f"Pen Width: {pen_width}")
pen_width_label.pack(side=tk.LEFT, padx=5)

pen_width_slider = ttk.Scale(pen_width_frame, from_=1, to=10, orient=tk.HORIZONTAL, command=update_pen_width, value=pen_width)
pen_width_slider.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

# Start the update loop
update_data()

# Run the application
root.mainloop()