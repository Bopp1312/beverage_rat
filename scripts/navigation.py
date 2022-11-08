#! /usr/bin/env python3

import rospy 
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R
import threading


class Command:
    def __init__(self,type,angle=0,position=[0,0],pause=0):
    	# Types ["angle","position","pause"]
    	self.type = type
    	self.pause = pause #seconds
    	self.position = position #(x,y)

rotate_30 = Command("angle",angle=30)
rotate_0  = Command("angle",angle=0)

commands = []
commands.append(rotate_30)
commands.append(rotate_0)
commands.append(rotate_30)
commands.append(rotate_0)

class Navigator:
	def __init__(self):
		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()

		self.thread = threading.Thread(target=self.loop())
		self.thread.start()

    def process(self, command):
        if command.type == "pause":
        	time.sleep(command.pause)

        if command.type == "angle":
        	self.go_to_angle(command.angle)
    def go_to_angle(self, angle):
    	# Publish desired robot frame
        
    	(trans,(x,y,z,w)) = self.listener.lookupTransform('/robot', '/world', rospy.Time(0))
        # Calculate error in angle 
        r = R.from_quat([w,x,y,z])
        angles = r.as_euler('xyz')
        print(angles)

    def loop(self):
    	while(rospy.is_shutdown == False):
            self.process(commands.pop(0))


if __name__ == "__main__":
    rospy.init_node("navigator_node", anonymous=True)
    obj = Navigator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
