import cv2
import numpy as np

# cap = cv2.VideoCapture('http://192.168.25.87:4747/video')
cap = cv2.VideoCapture(2)
# cap = nano.Camera(flip=0, width=640, height=480, fps=30)

# Set the minimum and maximum radius of the circles to be detected
min_radius = 5
max_radius = 60

while True:
    # Read a frame from the video
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    frame = cv2.line(frame, (320, 0), (320, 480), (0, 250, 0), 2)
    frame = cv2.line(frame, (0, 240), (640, 240), (0, 250, 0), 2)

    # Define the lower and upper HSV values for red color detection
    lower_red1 = np.array([0, 100, 20])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 20])
    upper_red2 = np.array([179, 255, 255])

    lower_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Create a mask to isolate the red pixels
    mask = lower_mask1 + lower_mask2

    # Use GaussianBlur to reduce noise and improve circle detection
    blurred = cv2.GaussianBlur(mask, (11, 11), 0)

    # Use HoughCircles to detect circles in the image
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=50,
        param2=30,
        minRadius=min_radius,
        maxRadius=max_radius
    )

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(frame, center, 1, (0, 255, 0), 3)
            radius = i[2]
            cv2.circle(frame, center, radius, (0, 255, 0), 3)

            # Implement your logic for each detected circle here
            # For example, you can print the coordinates of the circle's center
            print(f"Detected Circle at: {center}")

    # Display the frame with detected red circular objects
    cv2.imshow('Red Circular Object Detection', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
