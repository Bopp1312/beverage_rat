#! /usr/bin/env python3

import cv2

dest = "calibration_images/"

cap = cv2.VideoCapture(2)
i = 0
while(True):
    ret, frame = cap.read()
    print(ret)
    thumbnail = cv2.resize(frame,(500,300),cv2.INTER_LINEAR)
    cv2.imshow("webcam",thumbnail)
    
    val = cv2.waitKey(1) & 0xFF
    if val == ord('q'):
        break
 
    if val == ord('p'):
        cv2.imwrite(dest+"Image_"+str(i)+".jpg",frame)
        i = i +1

cap.release()
cv2.destroyAllWindows()
