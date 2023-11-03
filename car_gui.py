import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import carla #the sim library itself
import time # to set a delay after each photo
import cv2 #to work with images from cameras
import numpy as np #in this example to change image representation - re-shaping
import math
import sys
import pickle

from agents.navigation.global_route_planner import GlobalRoutePlanner



# # connect to the sim 
client = carla.Client('localhost', 2000)

# define speed contstants
PREFERRED_SPEED = 35 # what it says
SPEED_THRESHOLD = 2 #defines when we get close to desired speed so we drop the

# Max steering angle
MAX_STEER_DEGREES = 40

#camera mount offset on the car - you can tweak these to have the car in view or not
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

#adding params to display text to image
font = cv2.FONT_HERSHEY_SIMPLEX
# org - defining lines to display telemetry values on the screen
org = (30, 30) # this line will be used to show current speed
org2 = (30, 50) # this line will be used for future steering angle
org3 = (30, 70) # and another line for future telemetry outputs
org4 = (30, 90) # and another line for future telemetry outputs
org3 = (30, 110) # and another line for future telemetry outputs
fontScale = 0.5
# white color
color = (255, 255, 255)
# Line thickness of 2 px
thickness = 1

is_running = True  # Global flag to indicate if the app is running

def run_carla_client():
    global is_running
    if not is_running:
        return  # Stop scheduling new ticks if the app is no longer running
    try:
        world.tick()
    except Exception as e:
        print(e)
    finally:
        app.after(100, run_carla_client)


def cleanup():
    global is_running
    is_running = False

    # Revert CARLA to asynchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None  # Resets the fixed delta seconds to default
    world.apply_settings(settings)

    # Destroy any CARLA actors if needed
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for sensor in world.get_actors().filter('*sensor*'):
        sensor.destroy()

    app.destroy()

# maintain speed function
def maintain_speed(s):
    ''' 
    this is a very simple function to maintan desired speed
    s arg is actual current speed
    '''
    if s >= PREFERRED_SPEED:
        return 0
    elif s < PREFERRED_SPEED - SPEED_THRESHOLD:
        return 0.8 # think of it as % of "full gas"
    else:
        return 0.4 # tweak this if the car is way over or under preferred speed 

#function to subtract 2 vectors
def angle_between(v1, v2):
    return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))

# function to get angle between the car and target waypoint
def get_angle(car,wp):
    '''
    this function to find direction to selected waypoint
    '''
    vehicle_pos = car.get_transform()
    car_x = vehicle_pos.location.x
    car_y = vehicle_pos.location.y
    wp_x = wp.transform.location.x
    wp_y = wp.transform.location.y
    
    # vector to waypoint
    x = (wp_x - car_x)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
    y = (wp_y - car_y)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
    
    #car vector
    car_vector = vehicle_pos.get_forward_vector()
    degrees = angle_between((x,y),(car_vector.x,car_vector.y))

    return degrees

# Function to load stored waypoints from pickle file
def load_stored_waypoints(filename='waypoints.pkl'):
    with open(filename, 'rb') as f:
        return pickle.load(f)
    
def camera_callback(image,data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

def is_vehicle_stopped(vehicle, stop_threshold=0.1):
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    return speed < stop_threshold

def drive_along_route(vehicle,camera_data,route):
    quit = False
    curr_wp = 5 #we will be tracking waypoints in the route and switch to next one wen we get close to current one
    predicted_angle = 0
    while curr_wp<len(route)-1:
        # Carla Tick
        world.tick()
        if cv2.waitKey(1) == ord('q'):
            quit = True
            vehicle.apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))
            break
        image = camera_data['image']
        
        while curr_wp<len(route) and vehicle.get_transform().location.distance(route[curr_wp][0].transform.location)<5 and curr_wp!=len(route)-1:
            curr_wp +=1 #move to next wp if we are too close

        if vehicle.get_transform().location.distance(route[-1][0].transform.location) < 5:
            vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            break
            
        predicted_angle = get_angle(vehicle,route[curr_wp][0])
        image = cv2.putText(image, 'Steering angle: '+str(round(predicted_angle,3)), org, font, fontScale, color, thickness, cv2.LINE_AA)
        v = vehicle.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)
        image = cv2.putText(image, 'Speed: '+str(int(speed)), org2, font, fontScale, color, thickness, cv2.LINE_AA)
        image = cv2.putText(image, 'Next wp: '+str(curr_wp), org3, font, fontScale, color, thickness, cv2.LINE_AA)
        estimated_throttle = maintain_speed(speed)
        # extra checks on predicted angle when values close to 360 degrees are returned
        if predicted_angle<-300:
            predicted_angle = predicted_angle+360
        elif predicted_angle > 300:
            predicted_angle = predicted_angle -360
        steer_input = predicted_angle
        # limit steering to max angel, say 40 degrees
        if predicted_angle<-MAX_STEER_DEGREES:
            steer_input = -MAX_STEER_DEGREES
        elif predicted_angle>MAX_STEER_DEGREES:
            steer_input = MAX_STEER_DEGREES
        # conversion from degrees to -1 to +1 input for apply control function
        steer_input = steer_input/75

        vehicle.apply_control(carla.VehicleControl(throttle=estimated_throttle, steer=steer_input))
        cv2.imshow('RGB Camera',image)
################# Intialization #############

# Load the waypoints
stored_waypoints = load_stored_waypoints()

# Convert tuples to carla.Location objects
carla_locations = {}
for site_name, (location_tuple, dist) in stored_waypoints.items():
    x, y, z = location_tuple
    carla_location = carla.Location(x, y, z)
    carla_locations[site_name] = (carla_location, dist)

world = client.get_world()

settings = world.get_settings()
synchronous_mode = True
settings.synchronous_mode = synchronous_mode
settings.fixed_delta_seconds = 0.1 
world.apply_settings(settings)

spawn_points = world.get_map().get_spawn_points()
#look for a blueprint of Mini car
vehicle_bp = world.get_blueprint_library().filter('*mini*')

vehicle = world.try_spawn_actor(vehicle_bp[0], spawn_points[0]) # radom assign a location as spawn point

vehicle.set_autopilot(True)

world.tick()

def call_service():
    # Stop autopilot and get current vehicle location
    vehicle.set_autopilot(False)
    while not is_vehicle_stopped(vehicle):
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
        world.tick()

    vehicle_transform = vehicle.get_transform()
    current_taxi_location = vehicle_transform.location


    print(f"User Location: {user_location_var.get()}, Destination: {destination_var.get()}")
    user_destination=carla_locations[destination_var.get()][0]

    if user_location_var.get() == "User Location 1":
        user_location=spawn_points[0]
    elif user_location_var.get() == "User Location 2":
        user_location=spawn_points[21]
    else:
        user_location=spawn_points[88]

    #vehicle = world.try_spawn_actor(vehicle_bp[0], user_location)
    current_user_location = user_location.location #we start at where the car is

    print(user_location)
    
    sampling_resolution = 1
    grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)

    route = grp.trace_route(current_taxi_location, current_user_location)

    # #draw the route in sim window
    for waypoint in route:
        world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
            color=carla.Color(r=0, g=0, b=255), life_time=30.0,
            persistent_lines=True)
    print('Routing complete for taxi to user')

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
    camera_bp.set_attribute('image_size_y', '360')

    camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
    #this creates the camera in the sim
    camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

    image_w = camera_bp.get_attribute('image_size_x').as_int()
    image_h = camera_bp.get_attribute('image_size_y').as_int()

    camera_data = {'image': np.zeros((image_h,image_w,4))}
    # this actually opens a live stream from the camera
    camera.listen(lambda image: camera_callback(image,camera_data))

    cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RGB Camera',camera_data['image'])

    #drive from current taxi location to user location
    drive_along_route(vehicle,camera_data,route)
    
    while not is_vehicle_stopped(vehicle):
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
        world.tick()

    route_to_dest=grp.trace_route(current_user_location, user_destination)
    # #draw the route in sim window
    for waypoint in route_to_dest:
        world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
            color=carla.Color(r=255, g=0, b=0), life_time=30.0,
            persistent_lines=True)
    print('Routing complete for user to dest')

    # simulate wait for user to get on taxi
    for _ in range(30):
        time.sleep(0.1)  
        world.tick()  

    # Driving from user location to destination
    drive_along_route(vehicle,camera_data,route_to_dest)

    while not is_vehicle_stopped(vehicle):
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
        world.tick()

    # simulate wait for user to get on taxi
    for _ in range(30):
        time.sleep(0.1)  
        world.tick()  

    vehicle.set_autopilot(True)

    cv2.destroyAllWindows()
    camera.stop()
    for sensor in world.get_actors().filter('*sensor*'):
        sensor.destroy()
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()

########################### The GUI Front-end #############################
app = tk.Tk()
app.title("Automonous Taxi App")

# Load and display the map image
image = Image.open("map1.png")
image = image.resize((800, 500))
photo = ImageTk.PhotoImage(image)
map_frame = ttk.LabelFrame(app, text="Carla Town Map")
map_label = ttk.Label(map_frame, image=photo)
map_label.pack(padx=10, pady=10)
map_frame.pack(padx=10, pady=10, fill="both", expand=True)

# Create the user location dropdown
user_location_var = tk.StringVar()
user_location_frame = ttk.Frame(app)
user_location_label = ttk.Label(user_location_frame, text="User Location:")
user_location_label.pack(side=tk.LEFT, padx=5)
user_location_dropdown = ttk.Combobox(user_location_frame, textvariable=user_location_var, values=["User Location 1", "User Location 2", "User Location 3"])
user_location_dropdown.pack(side=tk.LEFT, padx=5)
user_location_frame.pack(padx=300, pady=5, fill="both")

# Create the destination dropdown
destination_var = tk.StringVar()
destination_frame = ttk.Frame(app)
destination_label = ttk.Label(destination_frame, text="Destination:")
destination_label.pack(side=tk.LEFT, padx=5)
point_of_interest=[
    "beach",
    "Hotel",
    "Supermarket",
    "Museums",
    "Playground",
    "Residence_district",
    "Financial_district",
    "Bank"]
destination_dropdown = ttk.Combobox(destination_frame, textvariable=destination_var, values=point_of_interest)
destination_dropdown.pack(side=tk.LEFT, padx=5)
destination_frame.pack(padx=300, pady=5, fill="both")

# Create the call service button with green color
call_service_button = tk.Button(app, text="Call Service", command=call_service, bg="green", fg="white")
call_service_button.pack(padx=10, pady=10)

app.after(100, run_carla_client)

app.protocol("WM_DELETE_WINDOW", cleanup)

app.mainloop()


