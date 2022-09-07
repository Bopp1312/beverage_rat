#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist 


pub = rospy.Publisher('cmd/vel', Twist,queue_size=1)

def callback(data):
	print(data.axes[0])
	twist = Twist()
	twist.linear.x = data.axes[0]
	twist.angular.z = data.axes[1]*0.5
	pub.publish(twist)


def listener():
	rospy.init_node('joy_listener', anonymous=True)
	rospy.Subscriber("/joy",Joy,callback)
	rospy.spin()





if __name__ == "__main__":
	listener()