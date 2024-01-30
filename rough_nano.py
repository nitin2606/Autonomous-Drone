import cv2
import numpy as np
#import nanocamera as nano
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math


#count = 0

# Connect to the Vehicle

'''vehicle = connect('127.0.0.1:14550')



#vehicle=connect('127.0.0.1:14550')

def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95: 
      print("Reached target altitude")
      break
    time.sleep(1)


def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]



arm_and_takeoff(5)
time.sleep(1)
print("Hold position for 2 seconds")
set_attitude(duration = 1)'''





# Open the video capture
#cap = cv2.VideoCapture('http://192.168.25.87:4747/video')
cap = cv2.VideoCapture(2)
#cap = nano.Camera(flip=0, width=640, height = 480, fps=30)



# Set a minimum area threshold
min_area = 100 # Adjust this value as needed



while True:
    # Read a frame from the video
    ret,frame = cap.read()

  


    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame = cv2.line(frame, (320,0), (320,480), (0,250,0), 2)
    frame = cv2.line(frame, (0,240), (640,240), (0,250,0), 2)


    # Define the lower and upper HSV values for red color detection
    
    #original
    lower_red1 = np.array([0, 100, 20])
    upper_red1= np.array([10, 255, 255])


    lower_red2 = np.array([160, 100, 20])
    upper_red2= np.array([179, 255, 255])


    lower_yellow1 = np.array([20, 120, 120])      # for yellow color  
    upper_yellow1 = np.array([30, 255, 255])  

    lower_yellow2 = np.array([20, 120, 120])      # for yellow color  
    upper_yellow2 = np.array([30, 255, 255]) 



    '''
    Precise
    lower_red1 = np.array([50, 100, 80])
    upper_red1= np.array([70, 255, 255])


    lower_red2 = np.array([160, 100, 40])
    upper_red2= np.array([179, 255, 255])'''


    #lower_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
   #lower_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)


    lower_mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
    
    lower_mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)

    # Create a mask to isolate the red pixels
    mask = lower_mask1 + lower_mask2


    #lower_red = np.array([0, 177, 61])
    #upper_red= np.array([8, 255, 145])

    #mask = cv2.inRange(hsv, lower_red, upper_red)


    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    

    # Loop through the detected contours and filter rectangular ones with a minimum area
    for contour in contours:

       
        
       
        #target_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        #x, y, w, h = cv2.boundingRect(target_contour)
        aspect_ratio = float(w) / h
        contour_area = cv2.contourArea(contour)
        #contour_area = cv2.contourArea(target_contour)

    
    
        if 0.5 < aspect_ratio < 1.6 and contour_area > min_area:
            # This contour is approximately rectangular and has an area greater than the threshold
            #cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)


            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,255), 2)

            #print("X = ",(x + (x+w))/2,"   ", "Y = ",(y + (y+h))/2)

            getX = (x + (x+w))/2
            getY = (y + (y+h))/2


            center_x = (x + x + w) // 2
            center_y = (y + y + h) // 2

            # Display the center point
            center_color = (0, 255, 0)  # Red color
            cv2.circle(frame, (center_x, center_y), 2, center_color, -1)

            frame = cv2.line(frame, (320,240), (center_x, center_y), (0,250,0), 2)


            if (getX < 285):
                print("LEFT")
                #set_attitude(roll_angle =-3,thrust = 0.5, duration =0.5)
                #set_attitude(roll_angle=5,thrust=0.5,duration=)
                break
           
                

            if (getX > 365):
                print("RIGHT")
                #set_attitude(roll_angle =3,thrust = 0.5, duration =0.5)
                #set_attitude(roll_angle=-20,thrust=0.5,duration=1.3)
                break
          
            

            if (getY<190):
                print("FORWARD")
                #set_attitude(pitch_angle=-3,thrust=0.5,duration=0.5)
                #set_attitude(pitch_angle=20,thrust=0.5,duration=1.3)
                break
          


            if (getY>320):
                print("BACKWARD")
                #set_attitude(roll_angle =3,thrust = 0.5, duration =0.5)
                #set_attitude(roll_angle=-20,thrust=0.5,duration=1.3)
                break

           
        
            if(380>getX>285 and 320>getY>190):
                #print("STOP")



                #cv2.imwrite(("Circle"+str(count)+".png"), frame)
              

                print("Detected !", "MODE: BRAKE")
                count = count+1

                #vehicle.mode = VehicleMode("BRAKE")

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