from dronekit import connect, VehicleMode
import time
import nanocamera as nano
import cv2

# Connect to the Pixhawk (update the connection string as necessary)
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, baud=57600, wait_ready=True)


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
    print (" Altitude: "), vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)


arm_and_takeoff(5)
print(" Take Off Complete ")
time.sleep(1)

cap = nano.Camera(flip=0, width=1920, height = 1080, fps=30)
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

# Check if the VideoCapture object opened successfully
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Specify the codec (XVID is a common choice)
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (1920, 1080))
# Parameters:
# - 'output.avi': Output file name.
# - 'fourcc': FourCC code for the codec. You can use other codecs like 'MJPG', 'X264', etc.
# - 20.0: Frames per second (fps) for the output video.
# - (640, 480): Frame size (width, height).

while cap.isOpened():
    frame = cap.read()  # Read a frame from the video source

    

    # Write the frame to the output video file
    out.write(frame)

    # Display the frame in a window (optional)
    cv2.imshow('Recording', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


time.sleep(10)

# Release the VideoCapture and VideoWriter objects, and close any open windows
cap.release()
out.release()
cv2.destroyAllWindows()


print("Preparing to land")
vehicle.mode = VehicleMode("LAND")

vehicle.close()
