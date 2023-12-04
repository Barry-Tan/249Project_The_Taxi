# #full code - with new controller

# #all imports
import carla 
import time # to set a delay after each photo
import cv2 #to work with images from cameras
import numpy as np 
import math
import sys
import pickle
import random
from controller2d2 import Controller2D

from agents.navigation.global_route_planner import GlobalRoutePlanner

# # connect to the sim 
client = carla.Client('localhost', 2000)

# define speed contstants
PREFERRED_SPEED = 25
SPEED_THRESHOLD = 2 #defines when we get close to desired speed so we drop 

# Max steering angle
MAX_STEER_DEGREES = 40

#camera mount offset on the car - you can tweak these to have the car in view or not
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

#adding params to display text to image
font = cv2.FONT_HERSHEY_SIMPLEX
org = (30, 30) 
org2 = (30, 50) 
org3 = (30, 70)
org4 = (30, 90) 
org3 = (30, 110) 
fontScale = 0.5
# white color
color = (255, 255, 255)
# Line thickness of 2 px
thickness = 1


# maintain speed function
def maintain_speed(s):
    ''' 
    this is a very simple function to maintan desired speed
    s arg is actual current speed
    '''
    if s >= PREFERRED_SPEED:
        return 0
    elif s < PREFERRED_SPEED - SPEED_THRESHOLD:
        return 0.6 # think of it as % of "full gas"
    else:
        return 0.3 # tweak this if the car is way over or under preferred speed 

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
settings.fixed_delta_seconds = 0.1 # This value might need adjustment
world.apply_settings(settings)

spawn_points = world.get_map().get_spawn_points()
#look for a blueprint of Mini car
vehicle_bp = world.get_blueprint_library().filter('*mini*')

point_b=carla_locations['beach'][0]

start_point = spawn_points[0]
point_a = start_point.location #we start at where the car is

print(start_point)
try:
    vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)
except:
    print('vehicle declare fail')
sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)

route = grp.trace_route(point_a, point_b)
# #draw the route in sim window - Note it does not get into the camera of the car
for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=30.0,
        persistent_lines=True)

################## NEW CONTROLLER ####################
# Assuming 'route' is a list of waypoints
#waypoints = [[wp[0].transform.location.x, wp[0].transform.location.y, wp[1]] for wp in route]
desired_speed = 10  # example speed in m/s
converted_route = []

for waypoint, _ in route:
    x = waypoint.transform.location.x
    y = waypoint.transform.location.y
    converted_route.append([x, y, desired_speed])
controller = Controller2D(converted_route)



camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
camera_bp.set_attribute('image_size_y', '360')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
#this creates the camera in the sim
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# this actually opens a live stream from the camera
camera.listen(lambda image: camera_callback(image,camera_data))

cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera',camera_data['image'])

########################################################## RADAR ##########################################################
def update_control_values(throttle, brake):
    global global_throttle, global_brake
    global_throttle, global_brake = throttle, brake

def process_radar(data, vehicle, min_distance=5, preferred_speed=PREFERRED_SPEED):
    global closest_distance
    closest = min((d for d in data if math.cos(d.azimuth) > 0), key=lambda d: d.depth, default=None)

    if closest:
        closest_distance = closest.depth  # Update the global variable
        if closest_distance < min_distance:
            # If the closest object is within the minimum distance, slow down
            print(f"Slowing down, object detected at {closest_distance} meters")
            return 0.0, 0.8  # Throttle, Brake
    else:
        closest_distance = None

    # Maintain preferred speed
    current_speed = 3.6 * math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2)
    return maintain_speed(current_speed), 0.0  # Throttle, Brake

# Initialize global variables for throttle and brake
global_throttle = 0.0
global_brake = 0.0

global closest_distance
closest_distance = None

# Add a radar sensor
radar_blueprint = world.get_blueprint_library().find('sensor.other.radar')
radar_blueprint.set_attribute('horizontal_fov', str(35))
radar_blueprint.set_attribute('vertical_fov', str(20))
radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
radar_sensor = world.spawn_actor(radar_blueprint, radar_transform, attach_to=vehicle)

# Modify the radar sensor listen method
radar_sensor.listen(lambda data: update_control_values(*process_radar(data, vehicle)))

#################traffic light #########################################
def get_traffic_light_state(vehicle):
    # Get the traffic light affecting the vehicle (if any)
    traffic_light = vehicle.get_traffic_light()
    if traffic_light is not None and traffic_light.get_state() == carla.TrafficLightState.Red:
        return 'Red'
    return 'Green'


# #main loop 
quit = False
curr_wp = 5 #we will be tracking waypoints in the route and switch to next one wen we get close to current one
predicted_angle = 0
frame_counter = 0

while curr_wp<len(route)-1:
    # Carla Tick
    world.tick()
    if cv2.waitKey(1) == ord('q'):
        quit = True
        vehicle.apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))
        break
    image = camera_data['image']
    
    
    predicted_angle = get_angle(vehicle,route[curr_wp][0])
    v = vehicle.get_velocity()
    speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)
    
    image = cv2.putText(image, 'Steering angle: '+str(round(predicted_angle,3)), org, font, fontScale, color, thickness, cv2.LINE_AA)
    image = cv2.putText(image, 'Speed: '+str(int(speed)), org2, font, fontScale, color, thickness, cv2.LINE_AA)
    image = cv2.putText(image, 'Next wp: '+str(curr_wp), org3, font, fontScale, color, thickness, cv2.LINE_AA)

    if closest_distance is not None:
        image = cv2.putText(image, 'Closest Distance: ' + str(round(closest_distance, 2)) + 'm', (30, 130), font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        image = cv2.putText(image, 'Closest Distance: N/A', (30, 130), font, fontScale, color, thickness, cv2.LINE_AA)
    
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

    estimated_throttle = global_throttle  # From radar
    estimated_brake = global_brake

    traffic_light_state = get_traffic_light_state(vehicle)
    if traffic_light_state == 'Red':
        estimated_brake = 1.0  # Full brake at red light
        estimated_throttle = 0.0

    ############## NEW CONTROLLER #################
    # Get the vehicle's current state
    frame_counter +=1
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    vehicle_velocity = vehicle.get_velocity()

    # Convert vehicle rotation from degrees to radians
    yaw_rad = np.radians(vehicle_rotation.yaw)

    # Update the controller with the current state
    controller.update_values(
        x=vehicle_location.x, 
        y=vehicle_location.y, 
        yaw=yaw_rad, 
        speed=np.linalg.norm([vehicle_velocity.x, vehicle_velocity.y, vehicle_velocity.z]), 
        timestamp=time.time(), 
        frame=frame_counter

    )

    controller.update_desired_speed()
    controller.update_controls()

    # Get control commands from the controller
    throttle0, steer0, brake0 = controller.get_commands()

    vehicle.apply_control(carla.VehicleControl(throttle=throttle0, steer=steer0, brake=brake0))    
    
    # vehicle.apply_control(carla.VehicleControl(throttle=estimated_throttle, steer=steer_input,brake=estimated_brake))
    cv2.imshow('RGB Camera',image)


#clean up
# Clean up radar sensor
radar_sensor.stop()
radar_sensor.destroy()

cv2.destroyAllWindows()
camera.stop()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()