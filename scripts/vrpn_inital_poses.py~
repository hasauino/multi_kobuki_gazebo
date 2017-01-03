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
		print [trans[0],trans[1],rot[2]]

		print '\n \n \n'
		rate.sleep()



#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
