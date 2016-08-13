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
from functions import Nearest,Steer,Near,ObstacleFree,Find,Cost,prepEdges,gridValue,assigner1
import parameters as param
#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    

    

# Node----------------------------------------------
def node():
	r1=1
	r2=1
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
    	goal.target_pose.pose.position.x=1
    	goal.target_pose.pose.position.y=0
    	goal.target_pose.pose.position.z=0
    	goal.target_pose.pose.orientation.w = 1.0
    	
   	
    	
    	client1.send_goal(goal)
    	h=client1.get_state()
    	print h,'\n ------',rospy.Time.now(),'------ \n'
    	client1.wait_for_result()
    	client1.get_result() 

    	   	
    	rate = rospy.Rate(50)	

	listener = tf.TransformListener()
	listener.waitForTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0),rospy.Duration(10.0))
	
        try:
		(trans,rot) = listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))
		
		
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		trans=[0,0]
		
	xinx=trans[0]
	xiny=trans[1]	
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
    	points.scale.x=0.05; 
    	line.scale.y= 0.02;
    	points.scale.y=0.05; 
	
    	line.color.r =9.0/255.0
	line.color.g= 91.0/255.0
	line.color.b =236.0/255.0
    	points.color.r = 255.0/255.0
	points.color.g = 244.0/255.0
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
    
	 
	
	xdim=mapData.info.width
	ydim=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y 
	#raw_input('Press Enter to start exploration')

#-------------------------------RRT------------------------------------------
	while not rospy.is_shutdown():

	 
# Sample free
	  #indxRand= floor( len(mapData.data)*random())
	  #yr=ceil(indxRand/xdim)
	  #xr=indxRand-(floor(indxRand/xdim))*xdim
	  #xr=xr*resolution+Xstartx
	  #yr=yr*resolution+Xstarty
	  xr=(random()*20.0)-10.0
	  yr=(random()*20.0)-10.0
	  x_rand = array([xr,yr])
	  
	  
	  
 
# Nearest
	  x_nearest=V[Nearest(V,x_rand),:]

# Steer
	  x_new=Steer(x_nearest,x_rand,param.eta)
	  
	  goal.target_pose.pose.position.x=x_new[0]
	  goal.target_pose.pose.position.y=x_new[1]
          goal.target_pose.pose.orientation.w = 1.0
	  
# unKnow discovery	  
	  if gridValue(mapData,x_new)==-1:
	  	assigner1(goal,x_new,client1,listener)
	  


# ObstacleFree
	  
	  if ObstacleFree(x_nearest,x_new,mapData,param.steps):
# Near function
	  	X_near=Near(V,x_new,param.rneighb)			
	        s_Xnear=X_near.shape[0]
		if All(X_near==array([0])):
			s_Xnear=-1
		
	 	V=vstack((V,x_new))	        
	        xmin=x_nearest
		cmin=Cost(E,x_nearest)+LA.norm(x_new-x_nearest)
		ii=0

		for ii in range(0,s_Xnear):						
			xnear=copy(X_near[ii,:])
			if ObstacleFree(xnear,x_new,mapData,param.steps) and ( Cost(E,xnear)+LA.norm(xnear-x_new) )<cmin:
				xmin=copy(xnear)
				cmin=Cost(E,xnear)+LA.norm(xnear-x_new)
		
		temp=concatenate((xmin,x_new))
		E=vstack((E,temp))


		iii=0
		for iii in range(0,s_Xnear):						
			xnear=copy(X_near[iii,:])		
			if ObstacleFree(xnear,x_new,mapData,param.steps) and ( Cost(E,x_new)+LA.norm(xnear-x_new) )<Cost(E,xnear):
				row=Find(E,xnear)
				E=delete(E, (row), axis=0)
				temp=concatenate((x_new,xnear))
				E=vstack((E,temp))
	
#Plotting
	  	
	  	
    	
    		
    			
 

	  	 
	  	pl=prepEdges(E)
		p.x=x_new[0] 
          	p.y=x_new[1]
          	pp.append(copy(p))
          	points.points=pp
          	line.points=pl
          	pub.publish(points)
          	pub.publish(line)
          	
          	
		

	  rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
