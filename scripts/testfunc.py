import rospy

def tfPrint(listener):
	listener.waitForTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0),rospy.Duration(10.0))
	(trans,rot) = listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))
	print trans 
