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
class Ego(object):
	"""docstring for Ego"""
	def __init__(self, r_esssx, r_esssy, r_escb, l_esssx, l_esssy, l_escb):
	# def __init__(self):
		#super(Ego, self).__init__()
		self.f_R_EgoSpeedSensorSpeedX = r_esssx
		self.f_R_EgoSpeedSensorSpeedY = r_esssy
		self.f_R_EgoSpeedClusterBased = r_escb
		self.f_L_EgoSpeedSensorSpeedX = l_esssx
		self.f_L_EgoSpeedSensorSpeedY = l_esssy
		self.f_L_EgoSpeedClusterBased = l_escb
		self.f_R_StaClsThrshld = None
		self.f_R_AmbClsThrshld = None
		self.f_R_DynClsThrshld = None
		self.f_L_StaClsThrshld = None
		self.f_L_AmbClsThrshld = None
		self.f_L_DynClsThrshld = None
		# self.f_R_EgoSpeedSensorSpeedX = None
		# self.f_R_EgoSpeedSensorSpeedY = None
		# self.f_R_EgoSpeedClusterBased = None
		# self.f_L_EgoSpeedSensorSpeedX = None
		# self.f_L_EgoSpeedSensorSpeedY = None
		# self.f_L_EgoSpeedClusterBased = None
	#def set_EgoSpeeds(r_esssx, r_esssy, r_escb, l_esssx, l_esssy, l_escb):
		# self.f_R_EgoSpeedSensorSpeedX = r_esssx
		# self.f_R_EgoSpeedSensorSpeedY = r_esssy
		# self.f_R_EgoSpeedClusterBased = r_escb
		# self.f_L_EgoSpeedSensorSpeedX = l_esssx
		# self.f_L_EgoSpeedSensorSpeedY = l_esssy
		# self.f_L_EgoSpeedClusterBased = l_escb

	def eval_thresholds(self):
		self.f_R_StaClsThrshld = np.interp(self.f_R_EgoSpeedClusterBased,[0.00, 1.50], [0.10, 1.00])
		self.f_R_AmbClsThrshld = np.interp(self.f_R_EgoSpeedClusterBased,[0.00, 1.50], [0.25, 0.50])
		self.f_R_DynClsThrshld = np.interp(self.f_R_EgoSpeedClusterBased,[0.00, 1.50], [0.38, 1.75])
		self.f_L_StaClsThrshld = np.interp(self.f_L_EgoSpeedClusterBased,[0.00, 1.50], [0.10, 1.00])
		self.f_L_AmbClsThrshld = np.interp(self.f_L_EgoSpeedClusterBased,[0.00, 1.50], [0.25, 0.50])
		self.f_L_DynClsThrshld = np.interp(self.f_L_EgoSpeedClusterBased,[0.00, 1.50], [0.38, 1.75])


class Cluster(object):
	PI = np.pi 
	"""docstring for Cluster"""
	def __init__(self):
		self.f_DistX = None
		self.f_DistY = None
		self.f_RangeRate = None 
		self.f_Angle = None  
		self.f_SinAngle = None 
		self.f_CosAngle = None 
		self.s_NumAssocObjs = None  
		self.iBestAssocObj = None  
		self.f_Vx = None
		self.f_Vy = None
		self.f_Ax = None
		self.f_Ay = None
		self.f_Vrelx = None
		self.f_Vrely = None
		self.s_ClusterID = None # {'Static','Static-Ambig', 'Ambig', 'Dynamic'}
		self.f_VradIdeal = None
		self.f_AbsRangeRateDelta = None

	def set_attribute(self, newdx, newdy, newrrate, newangle):
		self.f_DistX = newdx
		self.f_DistY = newdy
		self.f_RangeRate = newrrate
		self.f_Angle = newangle

	def set_filtercluster(self, vegoX, vegoY, StcThrhld, DynThrhld, AmbThrhld):
		self.f_VradIdeal = -((self.f_CosAngle*vegoX) + (self.f_SinAngle*vegoY)) 
		self.f_AbsRangeRateDelta = np.absolute(self.f_RangeRate - self.f_VradIdeal)
		if self.f_AbsRangeRateDelta < StcThrhld:
			if self.f_AbsRangeRateDelta > AmbThrhld:
				self.s_ClusterID = 'Static-Ambig'
			self.s_ClusterID = 'Static'
		elif self.f_AbsRangeRateDelta > DynThrhld:
			self.s_ClusterID = 'Dynamic'
		elif self.f_AbsRangeRateDelta > AmbThrhld:
			self.s_ClusterID = 'Ambig'

			
	# def eval_kinematics(self): 
	# 	rvector = mt.sqrt(self.f_DistX**2 + self.f_DistY**2)
	# 	print("X^2:" + str(self.f_DistX**2))
	# 	print("Y^2:" + str(self.f_DistY**2))
	# 	#print("9^1/2:" + np.square(9))
	# 	print("rvector:" + str(rvector))
	# 	foor = self.f_DistX/self.f_CosAngle
	# 	print("foorvector:" + str(foor))
	# 	gama = 180/self.PI - self.f_Angle# 180 - theta = 180 - 
	# 	self.f_Vrelx = -rvector*self.f_DistX*np.sin(gama)
	# 	self.f_Vrely = rvector*self.f_DistY*np.cos(gama)
	# 	self.f_Arelx = self.f_DistX
	# 	self.f_Arely = self.f_DistY

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

		self.f_Xo = None
		self.f_Yo = None
		self.f_Vox = None
		self.f_Voy = None
		self.f_Vrelx = None
		self.f_Vrely = None
		self.f_Aox = None
		self.f_Aoy = None
		self.i_ClustersPerObject = None
		self.f_probExist = None
		self.f_probGhost = None
		self.i_ObjectID = None

class SampleClusters(object):
	"""docstring for SampleClusters"""
	def __init__(self):
		#super(SampleClusters, self).__init__()
		self.validClustersPerSample = None
		self.listofValidClusters = None
		self.egoInfo = None


def FunctionReadData():
	return None

def FunctionCreateClusters():
	return None

def FunctionFilterClusters():
	return None

def FunctionCreateObjects():
	return None

def FunctionMergeObjects():
	return None

def FunctionTrackingObjects():
	return None

def FunctionGraphicalInterface():
	return None





a = Cluster()
print(a)

a.set_attribute(DISX,DISTY,RRATE,RANGLE)
print(a)
# a.eval_kinematics()
# a.get_kinematics()