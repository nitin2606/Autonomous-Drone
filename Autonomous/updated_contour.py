import cv2
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil


#connection_string = '/dev/ttyACM0'
#vehicle = connect(connection_string, baud=57600, wait_ready=True)



'''def arm_and_takeoff(aTargetAltitude):

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
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)


def send_global_velocity(vehicle,velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

arm_and_takeoff(5)
time.sleep(2)
point = LocationGlobalRelative(23.1779872, 80.0218928, 10)

vehicle.simple_goto(point)

time.sleep(14)'''

# Open the video capture
#cap = cv2.VideoCapture('http://192.168.0.113:4747/video')
cap = cv2.VideoCapture(2)
#cap = nano.Camera(flip=0, width=640, height = 480, fps=30)


# Set a minimum area threshold
min_area = 300 # Adjust this value as needed

while True:
    # Read a frame from the video
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame = cv2.line(frame, (320, 0), (320, 480), (0, 250, 0), 2)
    frame = cv2.line(frame, (0, 240), (640, 240), (0, 250, 0), 2)

    # Define the lower and upper HSV values for each target color
    lower_blue = np.array([90, 100, 20])
    upper_blue = np.array([120, 255, 255])

    lower_yellow = np.array([20, 100, 150])
    upper_yellow = np.array([40, 255, 255])

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])

    lower_red1 = np.array([0, 100, 20])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 20])
    upper_red2 = np.array([179, 255, 255])

    # Create masks for each color
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine masks to get a final mask
    final_mask = mask_blue + mask_yellow + mask_black + mask_red

    # Find contours in the combined mask
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through the detected contours and filter based on color and shape
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        contour_area = cv2.contourArea(contour)

        if 0.5 < aspect_ratio < 1.6 and contour_area > min_area:
            # This contour is approximately rectangular and has an area greater than the threshold
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

            print("X = ", (x + (x + w)) / 2, "   ", "Y = ", (y + (y + h)) / 2)

            getX = (x + (x + w)) / 2
            getY = (y + (y + h)) / 2


    # Display the frame with detected colored rectangular objects
    cv2.imshow('Colored Rectangular Object Detection', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        
            if (getX < 285):
                print("LEFT")
                #send_global_velocity(vehicle,0,-1,0,10)
                break
                
            if (getX > 365):
                print("RIGHT")
                #send_global_velocity(vehicle,0,1,0,10)
                break
            
            if (getY<190):
                print("FORWARD")
                #send_global_velocity(vehicle,1,0,0,10)
                break
            

            if (getY>320):
                print("BACKWARD")
                #send_global_velocity(vehicle,-1,0,0,10)


            if(380>getX>285 and 320>getY>190):
                print("STOP")

                #cv2.imwrite("Circle.png", frame)

                print("Detected !")

                #vehicle.mode = VehicleMode("LAND")

                #time.sleep(5)

                #cap.release()
                #vehicle.close()

            
                break
               
                

            


    # Display the frame with detected red rectangular objects
    cv2.imshow('Red Rectangular Object Detection', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()