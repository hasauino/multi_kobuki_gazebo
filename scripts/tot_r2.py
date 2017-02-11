#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import actionlib_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf



from os import system
from random import random
from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from time import time

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    
    

    

# Node----------------------------------------------
def node():

	rospy.init_node('distanceCounter2', anonymous=False) 

#-------------------------------------------


    	
   		 	   	
    	rate = rospy.Rate(50)	

        	
        
	listener = tf.TransformListener()
	listener.waitForTransform('/robot_2/odom', '/robot_2/base_link', rospy.Time(0),rospy.Duration(50.0))
	
        try:
		(trans,rot) = listener.lookupTransform('/robot_2/odom', '/robot_2/base_link', rospy.Time(0))
		
		
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		trans=[0,0]
		
	xinx=trans[0]
	xiny=trans[1]	

	xprev=array([xinx,xiny])
	distance=0
	t0=time()
	
#-------------------------------RRT------------------------------------------
	while not rospy.is_shutdown():
	  (trans,rot)=listener.lookupTransform('/robot_2/odom', '/robot_2/base_link', rospy.Time(0))
	  xinx=int(trans[0]*1000)/1000.0
	  xiny=int(trans[1]*1000)/1000.0
	  xnew=array([xinx,xiny])

	  
          
          distance+=LA.norm(xnew-xprev)
          print distance,"          elapsed  ",(time()-t0)," sec"
          xprev=array([xinx,xiny])
	  rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
