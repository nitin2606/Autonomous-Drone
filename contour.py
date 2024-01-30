import cv2
import numpy as np

cap = cv2.VideoCapture(2)


while True:

    ret, frame = cap.read()



    
    frame = cv2.medianBlur(frame,5)

    ret,thresh = cv2.threshold(frame,127,255,0)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(frame,center,radius,(0,255,0),2)

    cv2.imshow('detected circles',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


    
cv2.waitKey(0)
cv2.destroyAllWindows()