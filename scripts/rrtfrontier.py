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
from functions import Nearest,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue,assigner1rrtfront
import parameters as param
#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    

    

# Node----------------------------------------------
def node():

	global mapData

    	rospy.Subscriber("/robot_1/map", OccupancyGrid, mapCallBack)

    	pub = rospy.Publisher('shapes', Marker, queue_size=10)
    	rospy.init_node('RRTexplorer', anonymous=False)

    	#Actionlib client
    	client1 = actionlib.SimpleActionClient('/robot_1/move_base', MoveBaseAction)
    	client1.wait_for_server()
    	
    	

    	goal = MoveBaseGoal()
    	goal.target_pose.header.stamp=rospy.Time.now()
    	goal.target_pose.header.frame_id="/robot_1/map"


    	   	
    	rate = rospy.Rate(100)	

	listener = tf.TransformListener()
	listener.waitForTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0),rospy.Duration(10.0))
	
        try:
		(trans,rot) = listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))
		
		
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		trans=[0,0]
		
	xinx=trans[0]
	xiny=trans[1]	
	goal.target_pose.pose.position.x=xinx-0.5
    	goal.target_pose.pose.position.y=0
    	goal.target_pose.pose.position.z=0
    	goal.target_pose.pose.orientation.w = 1.0
    	
   	
    	
    	client1.send_goal(goal)
    	client1.wait_for_result()
    	client1.get_result() 
	x_init=array([xinx,xiny])
	
	V=array([x_init])
	i=1.0
	E=concatenate((x_init,x_init))	

    	points=Marker()
       	line=Marker()
#Set the frame ID and timestamp.  See the TF tutorials for information on these.
    	points.header.frame_id=line.header.frame_id="/robot_1/map"
    	points.header.stamp=line.header.stamp=rospy.Time.now()
	
    	points.ns=line.ns = "markers"
    	points.id = 0
    	line.id =1
	
    	points.type = Marker.POINTS
    	line.type=Marker.LINE_LIST
#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    	points.action = line.action = Marker.ADD;
	
    	points.pose.orientation.w = line.pose.orientation.w = 1.0;
	
    	line.scale.x = 0.02;
    	points.scale.x=0.2; 
    	line.scale.y= 0.02;
    	points.scale.y=0.2; 
	
    	line.color.r =9.0/255.0
	line.color.g= 91.0/255.0
	line.color.b =236.0/255.0
    	points.color.r = 255.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 0.0/255.0
   	
    	points.color.a=1;
	line.color.a = 0.6;
    	points.lifetime =line.lifetime = rospy.Duration();
	

    	p=Point()

    	p.x = x_init[0] ;
    	p.y = x_init[0] ;
    	p.z = 0;

    	pp=[]
        pl=[]
    	pp.append(copy(p))
    

	frontiers=[]
	xdim=mapData.info.width
	ydim=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y 
	

#-------------------------------RRT------------------------------------------
	while not rospy.is_shutdown():

	 
# Sample free
          xr=(random()*20.0)-10.0
	  yr=(random()*20.0)-10.0
	  x_rand = array([xr,yr])
	  
	  
	  
 
# Nearest
	  x_nearest=V[Nearest(V,x_rand),:]

# Steer
	  x_new=Steer(x_nearest,x_rand,param.eta)



# ObstacleFree
	  checking=ObstacleFree2(x_nearest,x_new,mapData)
	  
	  if checking==-1:

          	
          	if len(frontiers)>0:
          		frontiers=vstack((frontiers,x_new))
          	else:
          		frontiers=[x_new]
          		
          	(trans,rot) = listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))
	  	xinx=trans[0]
		xiny=trans[1]	
		x_init=array([xinx,xiny])
	  	V=array([x_init])
	  	E=concatenate((x_init,x_init))
	  	pp=[]
        	pl=[]
	  	
	  
	  elif checking==1:
	 	V=vstack((V,x_new))	
	 	temp=concatenate((x_nearest,x_new))        
	        E=vstack((E,temp))


		

          
          
	  z=0
          while z<len(frontiers):
	  	if gridValue(mapData,frontiers[z])!=-1:
	  		frontiers=delete(frontiers, (z), axis=0)
	  		z=z-1
		z+=1
	  frontiers=assigner1rrtfront(goal,frontiers,client1,listener)  	
          print rospy.Time.now(),"   ",len(frontiers)	
	  pp=[]	
	  for q in range(0,len(frontiers)):
	  	
	  	p.x=frontiers[q][0]
          	p.y=frontiers[q][1]
          	pp.append(copy(p))
          	

          		
          
#Plotting
	  	
  	  points.points=pp
          pl=prepEdges(E)
          line.points=pl
          pub.publish(line)        
          pub.publish(points) 
		
	  
	  rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
