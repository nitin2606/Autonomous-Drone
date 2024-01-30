import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
import argparse 


counter_1 = 0
counter_2 = 0
counter_3 = 0
counter_4 = 0

point_1 = [-35.3629198, 149.1651535]
point_2 = [-35.3627360, 149.1647351]
point_3 = [-35.3629416, 149.1641772]
point_4 = [-35.3639522, 149.1649336]


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

#-- Connect to the vehicle
#connection_string='/dev/ttyACM0'
#print('Connecting...')
#vehicle = connect(connection_string,baud=57600,wait_ready=True)

#-- Setup the commanded flying speed
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


arm_and_takeoff(10)
time.sleep(1)
print("Hold position for 1 seconds ...")

print('Going to desired Coordinates ...')

target_location = LocationGlobalRelative(coordinates[0][0], coordinates[0][1],10)

vehicle.simple_goto(target_location)

time.sleep(3)
