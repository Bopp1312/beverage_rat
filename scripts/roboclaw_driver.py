#!/usr/bin/env python3
# This code serves as the interface to the roboclaw device

import rospy
import time
import numpy as np
from roboclaw_3 import Roboclaw
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class driver:
    def __init__(self):
        # Create roboclaw objects and save to self
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/ttyACM0", 460800)
        self.roboclaw.Open()

        self.rate = rospy.Rate(1)
        self.odom_msg = Odometry()
        self.odom_msg.child_frame_id = "robot_base"

        # Subscribe to Twist message
        rospy.Subscriber("cmd/vel",Twist,self.callback_velocity,queue_size=1)
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=1)

        # Define roobot parameters used for odom calc
        self.wheel_base = 368.3/1000.0 #m
        self.wheel_radius = 33.33/1000.0 #m

        # Robot velocity transformation
        robot_tran = np.matrix([[-1/self.wheel_base, 1/self.wheel_base],
                                [               0.5,               0.5]])
        self.robot_tran_inv = np.linalg.inv(robot_tran)
        print(self.robot_tran_inv)
        # Enter loop to maintain update frequency
        self.loop()

    def loop(self):
        while(rospy.is_shutdown() == False):
            # Get encoder values
            # send to publisher values
            self.publish_odometry()
            # Command motor 
            self.rate.sleep()

    def publish_odometry(self):
        speed_1 = self.roboclaw.ReadSpeedM1(self.address)
        speed_2 = self.roboclaw.ReadSpeedM2(self.address)
        if (speed_1[0] == True) and (speed_2[0] == True):
            linear = (speed_1[1] + -speed_2[1])/2.0
            angular = -(speed_1[1] - -speed_2[1])/2.0
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.twist.twist.linear.x = linear
            self.odom_msg.twist.twist.angular.z = angular
            self.pub_odom.publish(self.odom_msg)

    def callback_velocity(self,msg):
        omega = msg.angular.z
        velocity = msg.linear.x
        cmd_vec = np.matrix([[omega],[velocity]])
        motor_velocity = (self.robot_tran_inv*cmd_vec)/(self.wheel_radius)
        
        speed_1_int = int(motor_velocity[0]*500)
        speed_2_int = int(motor_velocity[1]*500)

        # Command velocity 
        self.roboclaw.SpeedM1(self.address, speed_1_int)
        self.roboclaw.SpeedM2(self.address, speed_2_int)
        #print(speed_1_int)
        #print(speed_2_int)
        print(motor_velocity)

    def __exit__(self):
        print("Disconnecting from Roboclaw ..")
        # Disconnect
        self.roboclaw.SpeedM1(self.address, 0)
        self.roboclaw.SpeedM2(self.address, 0)

if __name__ == '__main__':
    rospy.init_node("roboclaw_driver", anonymous=True)
    obj = driver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")