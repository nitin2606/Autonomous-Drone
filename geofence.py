from dronekit import connect, VehicleMode
from shapely.geometry import Point, Polygon
import time
from pymavlink import mavutil
from pynput import keyboard

import argparse  

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Geofence vertices (latitude, longitude)
geofence_vertices = [
    (-35.362989756075962, 149.16491746902466),
    (-35.363514717484449, 149.16502475738525),
    (-35.363418474815106, 149.16553974151611),
    (-35.362919760963592, 149.16533589363098)
]

# Create a Shapely polygon from the vertices
geofence_polygon = Polygon(geofence_vertices)


result = 'S'

gnd_speed = 3 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude*0.95:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping:""" 
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to check if the vehicle is inside the geofence
def is_inside_geofence():
    current_location = Point(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    return geofence_polygon.contains(current_location)

def land_and_disarm():
    print("Vehicle outside the geofence. Landing...")
    vehicle.mode = VehicleMode("LAND")

    # Wait for the vehicle to land
    while vehicle.location.global_relative_frame.alt > 0.1:
        time.sleep(1)

    # Disarm the vehicle
    vehicle.armed = False
    vehicle.close()

# Callback function for key release event
def on_key_release(key):
    global result

    if result != 'S':
        result = 'S'
        print(result)
        set_velocity_body(0, 0, 0)

# Callback function for key press event
def on_key_pressed(key):
    global result

    if result == 'S':
        result1 = '%s' % key

        if result1 == 'Key.up':
            result = 'F'
            print(result)
            set_velocity_body(gnd_speed, 0, 0)
        
        if result1 == "Key.down":
            result = 'B'
            print(result)
            set_velocity_body(-gnd_speed, 0, 0)
        
        if result1 == "Key.left":
            result = 'L'
            print(result)
            set_velocity_body(0, -gnd_speed, 0)

        if result1 == 'Key.right':
            result = 'R'
            print(result)
            set_velocity_body(0, gnd_speed, 0)
        
        if result1 == 'Key.x':
            result = 'S'
            print(result)
            set_velocity_body(0, 0, 0)
        
        if result1 == 'Key.g':
            result = 'G'
            print(result)

# Arm and takeoff
arm_and_takeoff(5)

# Set the keyboard listener
with keyboard.Listener(on_release=on_key_release, on_press=on_key_pressed) as listener:
    try:
        while True:
            # Check if the vehicle is inside the geofence
            if not is_inside_geofence():
                land_and_disarm()
                break

            # Print the current location
            print("Current Location: Lat={}, Lon={}, Alt={}".format(
                vehicle.location.global_frame.lat,
                vehicle.location.global_frame.lon,
                vehicle.location.global_frame.alt
            ))

            time.sleep(1)

    except KeyboardInterrupt:
        print("Execution interrupted by user.")
    finally:
        listener.stop()

# Close the connection (this should not be reached as it's handled in the land_and_disarm function)
vehicle.close()
