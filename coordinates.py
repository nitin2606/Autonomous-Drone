import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2
import numpy as np
import argparse 

count = 0

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)


gnd_speed = 1 # [m/s]

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


arm_and_takeoff(10)
time.sleep(1)
print("Hold position for 2 seconds")



current_latitude = vehicle.location.global_frame.lat
current_longitude = vehicle.location.global_frame.lon
current_altitude = vehicle.location.global_frame.alt


# Print the coordinates
print("Current Latitude: {}".format(current_latitude))
print("Current Longitude: {}".format(current_longitude))
print("Current Altitude: {} meters".format(current_altitude))


arr = []
arr.append(current_latitude)
arr.append(current_longitude)
arr.append(current_longitude)

print(arr)


vehicle.mode = VehicleMode("LAND")
vehicle.close()