#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 11:17:00 2019

@author: plippmann
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import rospy
from std_msgs.msg import String

'''
n = 8 # Number of possibly sharp edges
r = .7 # magnitude of the perturbation from the unit circle, 
# should be between 0 and 1
N = n*3+1 # number of points in the Path
# There is the initial point and 3 points per cubic bezier curve. Thus, the curve will only pass though n points, which will be the sharp edges, the other 2 modify the shape of the bezier curve

angles = np.linspace(0,2*np.pi,N)
codes = np.full(N,Path.CURVE4)
codes[0] = Path.MOVETO

verts = np.stack((np.cos(angles),np.sin(angles))).T*(2*r*np.random.random(N)+1-r)[:,None]
verts[-1,:] = verts[0,:] # Using this instad of Path.CLOSEPOLY avoids an innecessary straight line
path = Path(verts, codes)

fig = plt.figure()
ax = fig.add_subplot(111)
patch = patches.PathPatch(path, facecolor='none', lw=2)
ax.add_patch(patch)

ax.set_xlim(np.min(verts)*1.1, np.max(verts)*1.1)
ax.set_ylim(np.min(verts)*1.1, np.max(verts)*1.1)
ax.axis('off') # removes the axis to leave only the shape
'''


from shapely.geometry import Polygon, LineString
from shapely.ops import polygonize
import scipy.optimize

### Gaussian smoother from http://www.swharden.com/blog/2008-11-17-linear-data-smoothing-in-python/ for getting a nice polygon

def smoothListGaussian(list,degree=5):  
     window=degree*2-1  
     weight=np.array([1.0]*window)  
     weightGauss=[]  
     for i in range(window):  
         i=i-degree+1  
         frac=i/float(window)  
         gauss=1/(np.exp((4*(frac))**2))  
         weightGauss.append(gauss)  
     weight=np.array(weightGauss)*weight  
     smoothed=[0.0]*(len(list)-window)  
     for i in range(len(smoothed)):  
         smoothed[i]=sum(np.array(list[i:i+window])*weight)/sum(weight)  
     return smoothed  

# Generate the polygon
theta = np.linspace(0,2*np.pi,200, endpoint=False)
r = np.random.lognormal(0,0.4,200)
r = np.pad(r,(9,10),mode='wrap')

r = smoothListGaussian(r, degree=10)

coordsRead = (np.cos(theta)*r, np.sin(theta)*r)
coords = zip(np.cos(theta)*r, np.sin(theta)*r)

polygon = Polygon(coords)


# The function for splitting the polygon and calculating the objective function value
def splitPoly(poly, parameters):
    rot = parameters[0]
    shift = parameters[1]

    c = poly.centroid.coords[0]
    l = polygon.length
    shiftvec = (np.cos(rot), np.sin(rot))

    linestart = (c[0] - shiftvec[0]*l + shiftvec[1]*shift, c[1] - shiftvec[1]*l - shiftvec[0]*shift)
    lineend = (c[0] + shiftvec[0]*l + shiftvec[1]*shift, c[1] + shiftvec[1]*l - shiftvec[0]*shift)

    line = LineString([linestart, lineend])

    splitpoly = list(polygonize(poly.boundary.union( line ) ))
    areadiff = 1+np.abs(splitpoly[0].area - splitpoly[1].area)
    lengthdiff = 1+np.abs(splitpoly[0].length - splitpoly[1].length)
    return areadiff*lengthdiff-1, splitpoly

def wrapper(parameters):
    return splitPoly(polygon, parameters)[0]

# Use a grid search for finding a good starting value
res = scipy.optimize.brute(wrapper, ((0.0,2),(-0.1,0.1)))

# Nelder-Mead for optimizing the parameters
res = scipy.optimize.minimize(wrapper, res, method='nelder-mead')

# Split the polygon using the final parameter values
value, splitpoly = splitPoly(polygon, res.x)

print("Areas: ", splitpoly[0].area, splitpoly[1].area)
print("Perimeters: ", splitpoly[0].length, splitpoly[1].length)

# Write the coordinates
np.savetxt('polygonA.txt', np.array(list(splitpoly[0].exterior.coords)))
np.savetxt('polygonB.txt', np.array(list(splitpoly[1].exterior.coords)))


file_object  = open('polygonB.txt',"r") 
print(file_object.read())

X, Y = [], []
for line in open('polygonB.txt', 'r'):
  values = [float(s) for s in line.split()]
  X.append(values[0])
  Y.append(values[1])

plt.plot(X, Y)
plt.show()

def talker():
    pub = rospy.Publisher('chatter', String, queue_size =10)
    rate = rospy.Rate(10)
    
    linelist = list()
    freelinelist = list()
    with open("polygonB") as f:
        for line in f:
            linelist.append(line)
        
        for line in linelist:
            freelinelist.append(line.rstrip('\n'))

    for line in freelinelist:
        rospy.loginfo(line)
        pub.publish(line)

