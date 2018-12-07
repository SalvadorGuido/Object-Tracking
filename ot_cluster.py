#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 10:28:07 2018

@author: eln.Angel Mancilla


class Ego() contains relevant attributes for ego
	
f_EgoSpeedSensorSpeedX : x component of speed vector measured by ego sensor
f_EgoSpeedSensorSpeedY : y component of speed vector measured by ego sensor
f_EgoSpeedClusterBased : magnitud of speed vector evaluated by cluster
f_StaClsThrshld : Static Cluster Threshold criteria
f_AmbClsThrshld : Ambiguous Cluster Threshold criteria
f_DynClsThrshld : Dynamic Cluster Threshold criteria

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

# DISX = 2
# DISTY = 3
# RRATE = 4
# RANGLE = .45
# ListTrackedObjects = 0
# ListClusters = 0


class Ego(object):
	"""docstring for Ego"""
	def __init__(self):
	# def __init__(self):
		#super(Ego, self).__init__()
		self.f_EgoSpeedSensorSpeedX = None
		self.f_EgoSpeedSensorSpeedY = None
		self.f_EgoSpeedClusterBased = None
		self.f_StaClsThrshld = None
		self.f_AmbClsThrshld = None
		self.f_DynClsThrshld = None
		# self.f_EgoSpeedSensorSpeedX = None
		# self.f_EgoSpeedSensorSpeedY = None
		# self.f_EgoSpeedClusterBased = None

	def set_EgoSpeeds(self,n_esssx, n_esssy, n_escb):
		self.f_EgoSpeedSensorSpeedX = n_esssx
		self.f_EgoSpeedSensorSpeedY = n_esssy
		self.f_EgoSpeedClusterBased = n_escb

	def eval_thresholds(self):
		self.f_StaClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.10, 1.00])
		self.f_AmbClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.25, 0.50])
		self.f_DynClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.38, 1.75])


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
		self.u_InvalidReasonBitField = None
		self.u_PropertiesBitField = None
		self.f_RSP_RangeRate = None
		self.s_NumAssocObjs = None  
		self.iBestAssocObj = None  
		self.s_ClusterKinematicID = None # {'Static','Static-Ambig', 'Ambig', 'Dynamic'}
		self.f_VradIdeal = None
		self.f_AbsRangeRateDelta = None
		self.f_Vrelx = None
		self.f_Vrely = None
		self.f_Vabsx = None
		self.f_Vabsy = None

	def set_attributes(self, newdx, newdy, newrrate, newangle, newsinangle, newconangle, newIRBField, newPBField, newRSPRRte):
		self.f_DistX = newdx
		self.f_DistY = newdy
		self.f_RangeRate = newrrate
		self.f_Angle = newangle
		self.f_SinAngle = newsinangle
		self.f_CosAngle = newconangle 
		self.u_InvalidReasonBitField = newIRBField
		self.u_PropertiesBitField = newPBField
		self.f_RSP_RangeRate = newRSPRRte

	def set_filtercluster(self, vegoX, vegoY, StcThrhld, DynThrhld, AmbThrhld):
		self.f_VradIdeal = -((self.f_CosAngle*vegoX) + (self.f_SinAngle*vegoY)) 
		self.f_AbsRangeRateDelta = np.absolute(self.f_RangeRate - self.f_VradIdeal)
		if self.f_AbsRangeRateDelta < StcThrhld:
			if self.f_AbsRangeRateDelta > AmbThrhld:
				self.s_ClusterKinematicID = 'Static-Ambig'
			self.s_ClusterKinematicID = 'Static'
		elif self.f_AbsRangeRateDelta > DynThrhld:
			self.s_ClusterKinematicID = 'Dynamic'
		elif self.f_AbsRangeRateDelta > AmbThrhld:
			self.s_ClusterKinematicID = 'Ambig'

			
	def eval_kinematics(self, egospeed): 
		self.f_Vrelx = self.f_RangeRate*self.f_CosAngle 
		self.f_Vrely = self.f_RangeRate*self.f_SinAngle 
		self.f_Vabsx = self.f_Vrelx + egospeed
		self.f_Vabsy = self.f_Vrely



		rvector = mt.sqrt(self.f_DistX**2 + self.f_DistY**2)
		# print("X^2:" + str(self.f_DistX**2))
		# print("Y^2:" + str(self.f_DistY**2))
		# #print("9^1/2:" + np.square(9))
		# print("rvector:" + str(rvector))
		foor = self.f_DistX/self.f_CosAngle
		# print("foorvector:" + str(foor))
		gama = 180/self.PI - self.f_Angle# 180 - theta = 180 - 
		self.f_Vrelx = -rvector*self.f_DistX*np.sin(gama)
		self.f_Vrely = rvector*self.f_DistY*np.cos(gama)
		self.f_Arelx = self.f_DistX
		self.f_Arely = self.f_DistY

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
		self.f_DistX = None
		self.f_DistY = None
		self.f_RangeRate = None
		self.f_Angle = None
		self.f_SinAngle = None
		self.f_CosAngle = None 
		self.f_Vrelx = None
		self.f_Vrely = None
		self.f_Vabsx = None
		self.f_Vabsy = None
		self.i_ClustersPerObject = None
		self.f_probExist = None
		self.f_probGhost = None
		self.i_ObjectID = None
		self.f_Priority = None
	def set_createobject(self):
		pass
	def set_mergeobjects(self):
		pass
class TrakedObjects(object):
	"""docstring for TrakedObjects"""
	def __init__(self):
		#super(TrakedObjects, self).__init__()
		self.l_40TrakedObjects = None
		

class SampleClusters(object):
	"""docstring for SampleClusters"""
	def __init__(self):
		#super(SampleClusters, self).__init__()
		self.validClustersPerSample = None
		self.listofValidClusters = None
		


# def FunctionReadData():
# 	return None

# def FunctionCreateClusters():
# 	return None

# def FunctionFilterClusters():
# 	return None

# def FunctionCreateObjects():
# 	return None

# def FunctionMergeObjects():
# 	return None

# def FunctionTrackingObjects():
# 	return None

# def FunctionGraphicalInterface():
# 	return None





# a = Cluster()
# print(a)

# a.set_attributes(DISX,DISTY,RRATE,RANGLE)
# print(a)
# a.eval_kinematics()
# a.get_kinematics()