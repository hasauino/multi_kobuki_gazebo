#!/usr/bin/env python


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

#-----------------------------------------------------

def getfrontier(mapData):
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	 
	img = np.zeros((h, w, 1), np.uint8)
	
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]==100:
				img[i,j]=0
			elif data[i*w+j]==0:
				img[i,j]=255
			elif data[i*w+j]==-1:
				img[i,j]=205
	
	
       	o=cv2.inRange(img,0,1)
	edges = cv2.Canny(img,0,255)
	im2, contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(o, contours, -1, (255,255,255), 5)
	o=cv2.bitwise_not(o) 
	res = cv2.bitwise_and(o,edges)
	#------------------------------

	frontier=copy(res)
	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

	im2, contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	center=[]
	if len(contours)>0:
		upto=len(contours)-1
		i=0
		maxx=0
		maxind=0
		for i in range(0,upto):
			if cv2.arcLength(contours[i],True)>maxx:
				maxx=cv2.arcLength(contours[i],True)
				maxind=i
		
	
		cnt = contours[maxind]
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		
		xr=cx*resolution+Xstartx
		yr=cy*resolution+Xstarty
		 
      		center=[xr,yr]
	
	return center


