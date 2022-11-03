#!/usr/bin/env python3

import numpy as np
import rospy
import time
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
from geometry_msgs.msg import PoseStamped
import denHartLib as dh
import rospkg

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()


ros_pack = rospkg.RosPack()
pkg_path = ros_pack.get_path('beverage-rat')

# Load camera calibration
matrix_coef = np.load(pkg_path + "/Calibration/matrix_coefficents.npy")
distortion_coef = np.load(pkg_path + "/Calibration/distortion_coefficents.npy")

marker_length = 0.210 #m

marker_timeout = 5.0 # seconds
marker_distance = 2.0 # meters

# Transform from world origin to markers
# World frame is defined as 0.5 meters infront of marker:0
# with X+ of the world frame facing Marker:0
TW_0 = np.matmul(dh.transformTranx(0.5),dh.transformRotz(np.pi))
point = np.array([[0.25],[0.25],[0],[1]])
print(TW_0*point)

print(TW_0)

def main():
    last_publish = time.time()
    pub = rospy.Publisher("robot/pose", PoseStamped, queue_size=1)
    cap = cv2.VideoCapture(2)
    while(not rospy.is_shutdown()):
        if((time.time() - last_publish) > marker_timeout):

            # Capture frame-by-frame
            ret, frame = cap.read()
            frame = cv2.resize(frame, (800,450),cv2.INTER_LINEAR)
            # Extract all information of tags from image
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters,cameraMatrix=matrix_coef,distCoeff=distortion_coef)
    
            #frame = (aruco.drawDetectedMarkers(frame.copy(), corners, ids))
            if len(corners) > 0:
                for i in range(len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, matrix_coef,distortion_coef)
                    #print(rvec)
                    print(tvec[0][0]/1000)
                    print(ids[i])
                    if(ids[0] == 0):
                        # Publish aruco marker to ROS
                        tran_w_robot = toTransform(rvec, tvec)
                        msg = PoseStamped()
                        msg.header.stamp = rospy.Time.now()
                        msg.pose.position.x = tran_w_robot[0,3]
                        msg.pose.position.y = tran_w_robot[1,3]

                        pub.publish(msg)
                        last_publish = time.time()
                    frame = (cv2.aruco.drawAxis(frame,matrix_coef,distortion_coef,rvec[i,:,:],tvec[i,:,:],marker_length))

            #thumbnail = cv2.resize(frame, (900,600), cv2.INTER_LINEAR)
            thumbnail = frame
            # Display the resulting frame
            cv2.imshow("frame",thumbnail)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def toTransform(rVec,tVec):
    transform = np.eye(4,4)
    rotation, jac = cv2.Rodrigues(rVec)
    position = tVec
    transform[:3,:3] = rotation
    transform[:3,3] = position
    return np.matrix(transform) 

if __name__ == "__main__":
    rospy.init_node("aruco_watcher",)
    main()
