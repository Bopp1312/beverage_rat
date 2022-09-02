import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

matrix_coef = np.load("Calibration/matrix_coefficents.npy")
distortion_coef = np.load("Calibration/distortion_coefficents.npy")
marker_length = 58/1000.0 #m

def main():
    cap = cv2.VideoCapture(2)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Extract all information of tags from image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters,cameraMatrix=matrix_coef,distCoeff=distortion_coef)
    
        #frame = (aruco.drawDetectedMarkers(frame.copy(), corners, ids))
        if len(corners) > 0:
            for i in range(len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, matrix_coef,distortion_coef)
                print(rvec)
                print(tvec)
                frame = (aruco.drawFrameAxes(frame,matrix_coef,dist_coef,rvec[i,:,:],tvec[i,:,:],marker_length))

        thumbnail = cv2.resize(frame, (900,600), cv2.INTER_LINEAR)
        # Display the resulting frame
        cv2.imshow("frame",thumbnail)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()