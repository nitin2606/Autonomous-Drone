import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
import tkinter as tk


import argparse  
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
    
    
#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'l':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("LAND")

    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(-gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(0, gnd_speed, 0)
    
    
#---- MAIN FUNCTION
#- Takeoff
#print(mavutil.mavlink.MAV_FRAME_BODY_NED)
arm_and_takeoff(5)

#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
