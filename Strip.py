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

    # Define the lower and upper HSV values for blue and red
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for blue and red
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine the masks to detect areas with both blue and red
    mask_combined = cv2.bitwise_and(mask_blue, cv2.bitwise_or(mask_red1, mask_red2))

    # Find contours in the combined mask
    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filter contours by area or other criteria as needed
        area = cv2.contourArea(contour)
        if area > 10:  # Adjust area threshold as needed
            # Fit a circle to the detected contour
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            # Draw a circle around the detected concentric discs
            cv2.circle(frame, center, radius, (0, 255, 0), 2)

    # Show the result
    cv2.imshow('Concentric Discs Detection', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' key to exit
        break

cap.release()
cv2.destroyAllWindows()
