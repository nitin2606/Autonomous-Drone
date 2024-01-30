import cv2 as cv
import time
from threading import Thread
import numpy as np

from dronekit import connect, VehicleMode
import time

# Connect to the Pixhawk (update the connection string as necessary)
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, baud=57600, wait_ready=True)



class webCamStream :

    def __init__(self, stream_id=0):

        self.stream_id = stream_id

        self.vCap = cv.VideoCapture(self.stream_id)
        
        if self.vCap.isOpened() is False:
            print("[Exiting]: Error accessing webcam stream.")
            exit(0) 
        
        fps_input_stream = int(self.vCap.get(5))
        print("FPS of input stream: {}".format(fps_input_stream))

        self.grabbed, self.frame = self.vCap.read()
        
        if self.grabbed is False:
            print("[Exiting] No more frames to read")
            exit(0)
        
        self.stopped = True

        self.t = Thread(target = self.update, args=())
        self.t.daemon = True # daemon thread runs in background



    # Method to start thread 
    def start(self):
        self.stopped = False
        self.t.start()
    
    # Method passed to thread to read next available frame


    def update(self):

        while True:
            if self.stopped is True:
                break

            self.grabbed, self.frame = self.vCap.read()
            if self.grabbed is False:
                print("[Exiting] No more frames to read")
                self.stopped = True
                break
        
        self.vCap.release()
    
    # Method to return latest read frame
    def read(self):
        return self.frame
    
    # Method to stop reading frames
    def stop(self):
        self.stopped = True
    


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
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)


arm_and_takeoff(5)
vehicle.mode = VehicleMode("ALTHOLD")


kernel = np.ones((7,7), np.uint8)

webcam_stream = webCamStream(stream_id="http://192.168.208.173:4747/video")
#webcam_stream = webCamStream(stream_id=0)
webcam_stream.start()

num_frames_processed = 0
start = time.time()

while True:
    if webcam_stream.stopped is True:
        break

    else:
        frame = webcam_stream.read()

        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame = cv.line(frame, (320,0), (320,480), (255,0,0), 2)
        frame = cv.line(frame, (640,375), (0,375), (255,0,0), 2)

        #lower_bound = np.array([20, 120, 120])      # for yellow color  
        #upper_bound = np.array([30, 255, 255])      # for 

        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([10, 255, 255])

        mask = cv.inRange(hsv, lower_bound, upper_bound)

        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        segmented_img = cv.bitwise_and(frame, frame, mask=mask)

        contours, hierarchy = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


        for contour in contours:

            area = cv.contourArea(contour)
            
            #print(area)

            if area>50:
               
                
                print("Detected !")
                
                vehicle.mode = VehicleMode("LAND")

                time.sleep(2)

                vehicle.close()

                break
            
         

        num_frames_processed +=1

        # displaying frame
        #cv.imshow('Frame', frame)
        #cv.imshow('Mask',segmented_img)
        key = cv.waitKey(1)
        if key==ord('q'):
           
            break

end = time.time()
webcam_stream.stop()


elapsed = end-start
fps = num_frames_processed/elapsed 
print("FPS: {} , Elapsed Time: {} ".format(fps, elapsed))

# closing all windows 
cv.destroyAllWindows()





