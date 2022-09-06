#! /usr/bin python3
# This code serves as the interface to the roboclaw device

import rospy
from roboclaw_3 import Roboclaw
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class driver:
	def __init__(self):
		# Create roboclaw objects and save to self
        
        # Subscribe to Twist message
        rospy.subscribe 

        # Define roobot parameters used for odom calc

		# Enter loop to maintain update frequency
		loop()

	def loop(self):
		# Get encoder values
		# send to publisher values
        # Command motor 
        self.rate.sleep()
    
    def __exit__(self):
        print("Disconnecting from Roboclaw ..")
        # Disconnect

if __name__ == '__main__':
	rospy