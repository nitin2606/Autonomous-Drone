import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2
import numpy as np



dummy_point1 = [-35.3629198, 149.1651535,30]
dummy_point2 = [-35.3627360, 149.1647351,30]
dummy_point3 = [-35.3629416, 149.1641772,30]
dummy_point4 = [-35.3639522, 149.1649336,30]




original_hotspot1 = [13.394484151126512, 77.73128900282778]
original_hotspot2 = [13.394433365359202, 77.73159701736482]
original_hotspot3 = [13.393648277167763, 77.7313608026834]
original_hotspot4 = [13.393939800670632, 77.73111960138523]

original_target = [13.393869642622231, 77.73141168658405]



#parser = argparse.ArgumentParser()
#parser.add_argument('--connect', default='127.0.0.1:14550')
#args = parser.parse_args()

connection_string='/dev/ttyACM0'
print('Connecting...')
vehicle = connect(connection_string,baud=57600,wait_ready=True)

# Connect to the Vehicle
#print ('Connecting to vehicle on: %s' % args.connect)
#vehicle = connect(args.connect, baud=921600, wait_ready=True)

#-- Connect to the vehicle
#connection_string='/dev/ttyACM0'
#print('Connecting...')
#vehicle = connect(connection_string,baud=57600,wait_ready=True)

#-- Setup the commanded flying speed

gnd_speed = 0.75 # [m/s]

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


def Dectection():

# Open the video capture
#cap = cv2.VideoCapture('127.0.0.1:14550')
    cap = cv2.VideoCapture(0)
    #cap = nano.Camera(flip=0, width=640, height = 480, fps=30)

    last_detection_time = time.time()

    # Set a minimum area threshold
    min_area = 100 # Adjust this value as needed

    frame_rate = cap.get(cv2.CAP_PROP_FPS)
    frame_interval = int(frame_rate)  # Capture one frame per second

    frame_count = 0

    while True:
        # Read a frame from the video
        ret,frame = cap.read()

        #frame_photo = frame.copy()

        frame_rate = cap.get(cv2.CAP_PROP_FPS)
        frame_interval = int(frame_rate)  # Capture one frame per second

        frame_count = 0

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



        '''
        Precise
        lower_red1 = np.array([50, 100, 80])
        upper_red1= np.array([70, 255, 255])


        lower_red2 = np.array([160, 100, 40])
        upper_red2= np.array([179, 255, 255])'''


        lower_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        
        lower_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)


        # Create a mask to isolate the red pixels
        mask = lower_mask1 + lower_mask2


        #lower_red = np.array([0, 177, 61])
        #upper_red= np.array([8, 255, 145])

        #mask = cv2.inRange(hsv, lower_red, upper_red)


        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        # Loop through the detected contours and filter rectangular ones with a minimum area
        for contour in contours:

        
            last_detection_time = time.time()
        
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


                if (getX < 285):
                    print("LEFT")
                    set_velocity_body(0, -gnd_speed, 0)
                    break
            
                    

                if (getX > 365):
                    print("RIGHT")
                    set_velocity_body(0, gnd_speed, 0)
                    break
            
                

                if (getY<190):
                    print("FORWARD")
                    set_velocity_body(gnd_speed, 0, 0)
                
                    break
            


                if (getY>320):
                    print("BACKWARD")
                    set_velocity_body(-gnd_speed, 0, 0)
                    break

            
            
                if(380>getX>285 and 320>getY>190):
                
                    set_velocity_body(0, 0, 0)

                    time.sleep(5)

                    cap.release()
                    cv2.destroyAllWindows()
                    return                   
                    
               
                
                # set_velocity_body(0, 0, 0)

            
        time_since_last_detection = time.time() - last_detection_time   

        if time_since_last_detection >= 3:
            print("Not Detected", "MODE: BRAKE") 

            cap.release()
            cv2.destroyAllWindows()
            return
            #vehicle.mode = VehicleMode("BRAKE") 


        # Display the frame with detected red rectangular objects
        #cv2.imshow('Red Rectangular Object Detection', frame)

        # Break the loop when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the window
    cap.release()
    cv2.destroyAllWindows()




arm_and_takeoff(30)
time.sleep(1)
print("Hold position for 2 seconds")



vehicle.simple_goto(LocationGlobalRelative(original_target[0], original_target[1], 20))
time.sleep(30)
Dectection()


vehicle.simple_goto(LocationGlobalRelative(original_hotspot4[0], original_hotspot4[1],10))
time.sleep(15)
Dectection()

vehicle.simple_goto(LocationGlobalRelative(original_hotspot3[0], original_hotspot3[1],10))
time.sleep(15)
Dectection()

vehicle.simple_goto(LocationGlobalRelative(original_hotspot2[0], original_hotspot2[1],10))
time.sleep(15)
Dectection()

vehicle.simple_goto(LocationGlobalRelative(original_hotspot1[0], original_hotspot1[1],10))
time.sleep(15)
Dectection()

# vehicle.simple_goto(LocationGlobalRelative(dummy_point3[0], dummy_point3[1],10))
# time.sleep(1)
# Dectection()
time.sleep(1)

vehicle.mode=VehicleMode("RTL")
time.sleep(5)
vehicle.close()


'''for pos in coordinates:
    vehicle.simple_goto(LocationGlobalRelative(pos[0],pos[1],pos[2]))
    time.sleep(12)
    print('Desending to 10 meters for clicking images...')
    v_alt = vehicle.location.global_relative_frame.alt
    
    while ( v_alt >= 0.95*10):
        set_velocity_body(0,0,1)
        v_alt = vehicle.location.global_relative_frame.alt
        print("Height: ", v_alt)
    
    set_velocity_body(0,0,0)
        
    #Dectection()'''




#vehicle.mode = VehicleMode("RTL")
#vehicle.close()

