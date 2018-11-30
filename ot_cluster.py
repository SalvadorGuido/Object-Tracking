#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 10:28:07 2018

@author: eln.Angel Mancilla


class Cluster() contains relevant attributes for cluster:

f_DistX: distance x from cluster to sensor 
f_DistY: distance y from cluster to sensor
f_RangeRate: escalar angular velocity of cluster refering to the sensor
f_RCS: reflexion level
f_Pdh0: bitmap representing probability of false measurement
f_Angle: 
f_SinAngle: 
f_CosAngle: 
u_InvalidReasonBitField: valid or invalid cluster
u_PropertiesBitField: 
s_RSPCluIdx: number assigned to cluster
s_NumAssocObjs: number of  asociated object a cluster can have 
iBestAssocObj: 

"""
import csv
import pandas as pd
import pathlib


class Cluster(object):
	"""docstring for Cluster"""
	def __init__(self):
		self.f_DistX = None
		self.f_DistY = None
		self.f_RangeRate = None 
		self.f_RCS = None 
		self.f_Pdh0 = None 
		self.f_Angle = None  
		self.f_SinAngle = None 
		self.f_CosAngle = None 
		self.u_InvalidReasonBitField = None  
		self.u_PropertiesBitField = None  
		self.s_RSPCluIdx = None  
		self.s_NumAssocObjs = None  
		self.iBestAssocObj = None  
	def set_attribute(self, newdx, newdy, newrrate):
		self.f_DistX = newdx
		self.f_DistY = newdy
		self.f_RangeRate = newrrate
	def __str__(self):
		return "Cluster:dx,xy,rrage:"+str(self.f_DistX)+":"+str(self.f_DistY)+":"+str(self.f_RangeRate)

class TrackedObject(object):
	"""docstring for TrackedObject"""

# class Missile(object):
#   MAX_SPEED = 100  # all missiles accelerate up to this speed
#   ACCELERATION = 5  # rate of acceleration per game frame

#   	def move(self):
#     	self.speed += self.ACCELERATION
#     	if self.speed > self.MAX_SPEED:
#       		self.speed = self.MAX_SPEED
    
	def __init__(self):
		#super(TrackedObject, self).__init__()

		self.x = None
		self.y = None
		self.Vx = None
		self.Vy = None
		self.Vrel = None
		self.ax = None
		self.ay = None
		self.listObjects = None
		self.probExist = None
		self.probGhost = None

class SampleClusters(object):
	"""docstring for SampleClusters"""
	def __init__(self):
		#super(SampleClusters, self).__init__()
		self.listofClusters = None
		self.validClustersPerSample = None
		self.egoInfo = None

a = Cluster()
print(a)

