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
import numpy as np
import math as mt

DISX = 2
DISTY = 3
RRATE = 4
RANGLE = .45
ListTrackedObjects = 0
ListClusters = 0

class Cluster(object):
	PI = np.pi 
	"""docstring for Cluster"""
	def __init__(self):
		self.f_DistX = None
		self.f_DistY = None
		self.f_RangeRate = None 
		self.f_RCS = None 
	#	self.f_Pdh0 = None 
		self.f_Angle = None  
		self.f_SinAngle = None 
		self.f_CosAngle = None 
	#	self.u_InvalidReasonBitField = None  
	#	self.u_PropertiesBitField = None  
	#	self.s_RSPCluIdx = None  
		self.s_NumAssocObjs = None  
		self.iBestAssocObj = None  
		self.e_Vx = None
		self.e_Vy = None
		self.e_Ax = None
		self.e_Ay = None
		self.e_Vrelx = None
		self.e_Vrely = None
	#	self.e_Arelx = None
	#	self.e_Arely = None
	def set_attribute(self, newdx, newdy, newrrate, newangle):
		self.f_DistX = newdx
		self.f_DistY = newdy
		self.f_RangeRate = newrrate
		self.f_Angle = newangle
	def eval_kinematics(self):
		rvector = mt.sqrt(self.f_DistX**2 + self.f_DistY**2)
		print("X^2:" + str(self.f_DistX**2))
		print("Y^2:" + str(self.f_DistY**2))
		#print("9^1/2:" + np.square(9))
		print("rvector:" + str(rvector))
		foor = self.f_DistX/f_CosAngle
		print("foorvector:" + str(foor))

		gama = 180/self.PI - self.f_Angle# 180 - theta = 180 - 
		self.e_Vrelx = -rvector*self.f_DistX*np.sin(gama)
		self.e_Vrely = rvector*self.f_DistY*np.cos(gama)
		self.e_Arelx = self.f_DistX
		self.e_Arely = self.f_DistY
	def get_kinematics(self):
		print("Cluster Kinematics(vx, vy, ax, ay):" + str(self.e_Vrelx) + "::" + str(self.e_Vrely) + "::" + str(self.e_Arelx) + "::" + str(self.e_Arely))

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

		self.Xo = None
		self.Yo = None
		self.Vox = None
		self.Voy = None
		self.Vrelx = None
		self.Vrely = None
		self.Aox = None
		self.Aoy = None
		self.ClustersPerObject = None
		self.probExist = None
		self.probGhost = None
		self.ObjectID = None

class SampleClusters(object):
	"""docstring for SampleClusters"""
	def __init__(self):
		#super(SampleClusters, self).__init__()
		self.listofClusters = None
		self.validClustersPerSample = None
		self.egoInfo = None


def ReadData():
	return None

def CreateClusters():
	return None

def FilterClusters():
	return None

def CreateObjects():
	return None

def MergeObjects():
	return None

def TrackingObjects():
	return None

def GraphicalInterface():
	return None





a = Cluster()
print(a)

a.set_attribute(DISX,DISTY,RRATE,RANGLE)
print(a)
a.eval_kinematics()
a.get_kinematics()