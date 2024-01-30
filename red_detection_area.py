import cv2
import numpy as np

# Open the video capture
cap = cv2.VideoCapture('http://192.168.0.113:4747/video')

# Set a minimum area threshold
min_area = 50  # Adjust this value as needed

while True:
    # Read a frame from the video
    ret, frame = cap.read()

    if not ret:
        break  # Break the loop if there are no more frames

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame = cv2.line(frame, (320,0), (320,480), (255,0,0), 2)
    frame = cv2.line(frame, (640,375), (0,375), (255,0,0), 2)


    # Define the lower and upper HSV values for red color detection
    lower_red1 = np.array([0, 100, 20])
    upper_red1= np.array([10, 255, 255])


    lower_red2 = np.array([160, 100, 20])
    upper_red2= np.array([179, 255, 255])

    lower_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    lower_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)






    # Create a mask to isolate the red pixels
    mask = lower_mask1 + lower_mask2

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through the detected contours and filter rectangular ones with a minimum area
    for contour in contours:
       

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        contour_area = cv2.contourArea(contour)



        if 0.8 < aspect_ratio < 1.2 and contour_area > min_area:
            # This contour is approximately rectangular and has an area greater than the threshold
            #cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)


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
               
                break

            


    # Display the frame with detected red rectangular objects
    cv2.imshow('Red Rectangular Object Detection', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
