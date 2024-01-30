import cv2
import os

# Open the video file
video_file = 'your_video.mp4'
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

# Create a directory to store the captured frames
output_dir = 'captured_frames'
os.makedirs(output_dir, exist_ok=True)

frame_rate = cap.get(cv2.CAP_PROP_FPS)
frame_interval = int(frame_rate)  # Capture one frame per second

frame_count = 0
while True:
    ret, frame = cap.read()

    if not ret:
        break

    if frame_count % frame_interval == 0:
        # Save the frame as an image with a timestamp
        timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)
        image_filename = os.path.join(output_dir, f"{int(timestamp / 1000)}.jpg")
        cv2.imwrite(image_filename, frame)

    frame_count += 1
    #cv2.imshow('Red Rectangular Object Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the output directory
cap.release()
cv2.destroyAllWindows()
