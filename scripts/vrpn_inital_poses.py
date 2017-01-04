#!/usr/bin/env python
#--------Include modules---------------
import rospy
import tf
from numpy import *
import numpy as np




# Node----------------------------------------------

def node():
	rospy.init_node('vrpn_inital_poses', anonymous=False)
	rate = rospy.Rate(100)	
	listener = tf.TransformListener()
	listener.waitForTransform('/robot1', '/robot2', rospy.Time(0),rospy.Duration(10.0))




	while not rospy.is_shutdown():
		(trans,rot) = listener.lookupTransform('/robot1', '/robot2', rospy.Time(0))
		rot = tf.transformations.euler_from_quaternion(rot)
		x=-trans[0]
		y=trans[2]
		yaw=rot[1]
		deg=yaw*180.0/pi
		
		#print round(x,3),'\n',round(y,3),'\n',round(yaw,3),'\n',round(deg,3)
		print round(x,3),'    ',round(y,3),'    ',round(yaw,3),'    ',round(deg,3),'\n'
		print '\n \n \n'
		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
