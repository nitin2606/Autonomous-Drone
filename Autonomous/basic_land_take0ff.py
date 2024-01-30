from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  
import dist_calulator 

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()




# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc


point_1 = [-35.3629198, 149.1651535]
point_2 = [-35.3627360, 149.1647351]
point_3 = [-35.3629416, 149.1641772]
point_4 = [-35.3639522, 149.1649336]

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

# Initialize the takeoff sequence to 15m
arm_and_takeoff(15)


target_location = LocationGlobalRelative(point_4[0], point_4[1], 20)

geo_array = [ vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]

dist = dist_calulator.distance(geo_array, point_4)

while dist > 2:

  geo_array = [ vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]
  dist = dist_calulator.distance(geo_array, point_4)

  print("Distace: ", dist)
  vehicle.simple_goto(target_location)

  if dist < 2:
    vehicle.mode = VehicleMode("LAND")
    break

  
print("Take off complete")

print("Now let's land")
vehicle.mode = VehicleMode("LAND")
vehicle.close()