import cv2
import numpy as np

# Initialize video capture (0 for the default camera)
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV values for blue color
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask for blue color
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Apply morphological operations (optional)
    kernel = np.ones((5, 5), np.uint8)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # Find circles using the Hough Circle Transform
    circles = cv2.HoughCircles(
        mask_blue, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=80, param2=50, minRadius=5, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

    # Show the result
    cv2.imshow('Blue Circle Detection', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' key to exit
        break

cap.release()
cv2.destroyAllWindows()
