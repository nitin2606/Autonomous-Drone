import cv2
import numpy as np
from command import arm_and_takeoff,set_velocity_body

gnd_speed = 1 # [m/s]
# cap = cv2.VideoCapture('http://192.168.25.87:4747/video')
def Dectection(vehicle):
    cap = cv2.VideoCapture(2)
    # cap = nano.Camera(flip=0, width=640, height=480, fps=30)

    # Set a minimum area threshold
    min_area = 50  # Adjust this value as needed

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

        blurred = cv2.GaussianBlur(mask, (11, 11), 0)
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables for tracking the object with the shortest distance
        min_distance = float('inf')
        min_distance_object = None

        # Loop through the detected contours and filter rectangular ones with a minimum area
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            contour_area = cv2.contourArea(contour)

            if 0.5 < aspect_ratio < 1.6 and contour_area > min_area:
                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

                center_x = (x + x + w) // 2
                center_y = (y + y + h) // 2

                center_color = (0, 255, 0)  # Red color
                cv2.circle(frame, (center_x, center_y), 2, center_color, -1)

                frame = cv2.line(frame, (320, 240), (center_x, center_y), (0, 250, 0), 2)

                # Calculate the distance from the center of the frame
                distance = np.sqrt((center_x - 320) ** 2 + (center_y - 240) ** 2)

                # Update the minimum distance object if needed
                if distance < min_distance:
                    min_distance = distance
                    min_distance_object = (center_x, center_y)

        # Display the frame with detected red rectangular objects and the tracking information
        cv2.putText(frame, f"Min Distance: {min_distance:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if min_distance_object is not None:
            cv2.putText(frame, f"Tracking Object: {min_distance_object}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2)

            target_x, target_y = min_distance_object

            # Provide movement directions based on the position of the target
            if (target_x < 285):
                    print("LEFT")
                    set_velocity_body(vehicle,0, -gnd_speed, 0)
                    break
            
                    

            if (target_x > 365):
                print("RIGHT")
                set_velocity_body(vehicle,0, gnd_speed, 0)
                break
        
            

            if (target_y<190):
                print("FORWARD")
                set_velocity_body(vehicle,gnd_speed, 0, 0)
            
                break
        


            if (target_y>320):
                print("BACKWARD")
                set_velocity_body(vehicle,-gnd_speed, 0, 0)
                break

            
            
                # if(380>getX>285 and 320>getY>190):
                
                #     set_velocity_body(0, 0, 0)
                    
                #     while(5 <= vehicle.location.global_relative_frame.alt):
                #         print("ALT: "+str(vehicle.location.global_relative_frame.alt))

                #         set_velocity_body(0, 0, 0.5)

            
                #     #cv2.imwrite("img"+str(count)+".png", frame_photo)

                #     count += 1

            
                #     print("Detected !", "MODE: BRAKE")
                #     break
                
                # set_velocity_body(0, 0, 0)

        cv2.imshow('Red Rectangular Object Detection', frame)

        # Break the loop when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the window
    cap.release()
    cv2.destroyAllWindows()

 
