import cv2
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time


connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, baud=57600, wait_ready=True)



def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
 
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")

  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) 


  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)


arm_and_takeoff(5)
time.sleep(2)
#vehicle.mode = VehicleMode("ALT_HOLD")


point = LocationGlobalRelative(23.1779872, 80.0218928, 10)

vehicle.simple_goto(point)

time.sleep(14)


cap = cv2.VideoCapture('http://192.168.208.173:4747/video')


min_area = 1000  

while True:
    # Read a frame from the video
    ret, frame = cap.read()

    if not ret:
        break 

  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame = cv2.line(frame, (320,0), (320,480), (255,0,0), 2)
    frame = cv2.line(frame, (640,375), (0,375), (255,0,0), 2)


    
    lower_red1 = np.array([0, 100, 20])
    upper_red1= np.array([10, 255, 255])


    lower_red2 = np.array([160, 100, 20])
    upper_red2= np.array([179, 255, 255])

    lower_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    lower_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)






    
    mask = lower_mask1 + lower_mask2

 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    
    for contour in contours:
       

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        contour_area = cv2.contourArea(contour)



        if 0.8 < aspect_ratio < 1.2 and contour_area > min_area:
           
            cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)


            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,255), 2)

            print("X = ",(x + (x+w))/2,"   ", "Y = ",(y + (y+h))/2)

            getX = (x + (x+w))/2
            getY = (y + (y+h))/2


            if (getX < 180):
                    print("LEFT")
                  
                    break
                
            if (getX > 440):
                print("RIGHT")
            
                break
            
            if (440>getX>180 and  getY<350):
                print("FORWARD")
              
                break
            
            if(440>getX>180 and getY>350):
                print("STOP")

                print("Detected !")

                cap.release()

                
                vehicle.mode = VehicleMode("LAND")
               
                time.sleep(2)

                vehicle.close()

                break
               
                

            


    # Display the frame with detected red rectangular objects
    #cv2.imshow('Red Rectangular Object Detection', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
