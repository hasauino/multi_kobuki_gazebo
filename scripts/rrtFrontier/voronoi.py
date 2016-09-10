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
from PIL import Image
import random
import math
 


from os import system

from numpy import array,concatenate,vstack,delete,floor,ceil
from numpy import linalg as LA
from numpy import all as All
from functions import Nearest,Nearest2,Steer,Near,ObstacleFree2,Find,Cost,prepEdges,gridValue,assigner1rrtfront

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
points=[]

def callBack(data):
    global points,mapData
    Xp=[array([data.x,data.y])]
    resolution=mapData.info.resolution
    Xstartx=mapData.info.origin.position.x
    Xstarty=mapData.info.origin.position.y

    width=mapData.info.width
    

    x=int(floor((Xp[0][1]-Xstarty)/resolution))
    y=int(floor((Xp[0][0]-Xstartx)/resolution))
    
    Xp=[array([x,y])]
    print Xp
    
    if len(points)>0:
    	points=vstack((points,Xp))
    else:
       	points=Xp
      
	

    
def mapCallBack(data):
    global mapData
    mapData=data
    



def generate_voronoi_diagram(width, height, points):
	scale=2
	image = Image.new("RGB", (width*scale, height*scale))
	putpixel = image.putpixel
	imgx, imgy = image.size
	num_cells=len(points)
	nx=[]
	ny=[]
	nr = []
	ng = []
	nb = []
	for i in range(num_cells):
		nx.append((points[i][0])*scale)
		ny.append((points[i][1])*scale)
		
		nr.append(random.randrange(256))
		ng.append(random.randrange(256))
		nb.append(random.randrange(256))
	for y in range(imgy):
		for x in range(imgx):
			dmin = math.hypot(imgx-1, imgy-1)
			j = -1
			for i in range(num_cells):
				d = math.hypot(nx[i]-x, ny[i]-y)
				if d < dmin:
					dmin = d
					j = i
			putpixel((x, y), (nr[j], ng[j], nb[j]))
	
	
	for k in range(num_cells):
		for kk in range(1):
			putpixel((nx[k]+kk, ny[k]+kk), (0, 0, 0))
			putpixel((nx[k]-kk, ny[k]-kk), (0, 0, 0))
			putpixel((nx[k]-kk, ny[k]+kk), (0, 0, 0))
			putpixel((nx[k]+kk, ny[k]-kk), (0, 0, 0))
			putpixel((nx[k], ny[k]+kk), (0, 0, 0))
			putpixel((nx[k], ny[k]-kk), (0, 0, 0))
			putpixel((nx[k]-kk, ny[k]), (0, 0, 0))
			putpixel((nx[k]+kk, ny[k]), (0, 0, 0))
		
	
	
	image.save("VoronoiDiagram.png", "PNG")
        image.show()






# Node----------------------------------------------
def node():

	global points,mapData
	rospy.init_node('voronoi', anonymous=False)
	
#-------------------------------------------
    	rospy.Subscriber("/robot_1/map", OccupancyGrid, mapCallBack)
    	
    	
    	
	while mapData.header.seq<1 or len(mapData.data)<1:
		pass
	
	rospy.Subscriber("/points", Point, callBack)
	raw_input('waiting')
	
	generate_voronoi_diagram(mapData.info.width, mapData.info.height, points)
	

 	



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
