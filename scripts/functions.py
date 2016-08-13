#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Point
from os import system

from random import random
from numpy import array
from numpy import floor
from numpy import delete
from numpy import concatenate
from numpy import vstack
from numpy import linalg as LA
from math import copysign
from numpy import where
from numpy import logical_and as AND
from numpy import all as All
from scipy.optimize import minimize

# Nearest function-------------------------------------
def Nearest(V,x):
 n=1000000
 i=0
 for i in range(0,V.shape[0]):
    n1=LA.norm(V[i,:]-x)
    if (n1<n):
	n=n1
        result=i    
 return result

# Steer function-------------------------------------

def myfun(x,x0,x1,eta):
   X=array([x[0],x[1]])
   return LA.norm(X-x1)






def Steer(x0,x1,eta):
	

	def consFun(x):
		X=array([x[0],x[1]])
		x0=p[0]
		eta=p[2]
		return -LA.norm(X-x0)+eta



	cons = ({'type': 'ineq',
                 'fun' : consFun  })
         
       	p=(x0,x1,eta)
	res = minimize(myfun,[x0[0],x0[1]],args=p,constraints=cons, method='COBYLA',options={'disp': False})

	xnew=array([res.x[0],res.x[1]])
		
	return xnew

# gridCheck function-------------------------------------
def gridCheck(mapData,Xp):
 resolution=mapData.info.resolution
 Xstartx=mapData.info.origin.position.x
 Xstarty=mapData.info.origin.position.y

 width=mapData.info.width
 Data=mapData.data
 # check if points are in freespace or not
 # c=1 means grid cell occupied
 # c=0 means grid cell is free
 index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) )
 c=1
 if int(index) < len(Data):
 	if Data[int(index)]==0:
  	  c=0
 	else:
 	  c=1 
 
  
 #print 'point=',Xp,'    index= ',index,'           grid=',Data[int(index)]
 #print c
 return c

# ObstacleFree function-------------------------------------

def ObstacleFree(xnear,xnew,mapsub,stepz):
 out=1
 ri=LA.norm(xnew-xnear)/stepz
 xi=xnear
 
 c=1

 for c in range(0,stepz):
   xi=Steer(xi,xnew,c*ri)
   if (gridCheck(mapsub,xi) !=0):
     out=0
     
   

 if (gridCheck(mapsub,xnew) !=0):
  out=0
  
 
 return out

# Find function-------------------------------------
def Find(E,x):
 if not All(array([E.shape]).shape==array([1,1])):
	 yy=E==x[1]
	 xx=E==x[0]
	 m=AND(yy[:,3], xx[:,2])
	 m=where(m==True)

	 if len(m[0])>0:
	  return m[0][0]
 else:
	 return 0
# Near function-------------------------------------

def Near(V,xnew,r):
 xnear=array([0,0])
 i=0
 for i in range(0,V.shape[0]):
    n=LA.norm(V[i,:]-xnew)
    if (n<=r):
        p=V[i,:]
        xnear=vstack((xnear,p))


 xnear=delete(xnear, (0), axis=0)
 return xnear

# Cost function-------------------------------------

def Cost(E,xn):
	x=xn
	if All(array([E.shape]).shape==array([1,1])):
		c=0
		
	else:
		xinit=E[0,0:2]

		c=0
		while not All(x==xinit):
	        	xp=E[Find(E,x),0:2]
			c+=LA.norm(x-xp)
			x=xp
		
	return c

# prepEdges function
		
def prepEdges(E):
	p=Point()
	pl=[]
	if not All(array([E.shape]).shape==array([1,1])):	
			
		Ex=delete(E, (1), axis=1)
		Ex=delete(Ex, (2), axis=1)

		Ey=delete(E, (0), axis=1)
		Ey=delete(Ey, (1), axis=1)
		pxs=Ex.flatten()
		pys=Ey.flatten()		
		
		j=0
					
		for j in range(0,pys.shape[0]):			
			p.x=pxs[j]
			p.y=pys[j]
			pl.append(copy(p)) 
					
			
		
	return pl




























