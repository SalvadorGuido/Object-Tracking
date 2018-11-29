#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 10:28:07 2018

@author: eln.Angel Mancilla

class cluster containing all attributes for each cluster:

f_DistX: distance x from cluster to sensor 
f_DistY: distance y from cluster to sensor
f_RangeRate: escalar angular velocity of cluster refering to the sensor
f_RCS: 
f_Pdh0: 
f_Angle: 
f_SinAngle: 
f_CosAngle: 
u_InvalidReasonBitField: 
u_PropertiesBitField: 
s_RSPCluIdx: 
s_NumAssocObjs: 
iBestAssocObj: 


"""
import csv
import pandas as pd
import pathlib


class BinarySearchTree:

    def __init__(self):
        self.root = None
        self.size = 0

    def length(self):
        return self.size

    def __len__(self):
        return self.size

    def __iter__(self):
        return self.root.__iter__()

class Nodes(object): 
    def __init__(self,key): 
        self.left = None
        self.right = None
        self.val = key 
    def __str__(self):
        return "Node:(key, left, right) "+str(self.val)+":"+str(self.left)+":"+str(self.right)

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
		self.f_DistX = newdy
		self.f_DistX = newrrate
	def __str__(self):
		return "Cluster:dx,xy,rrage:"+str(self.f_DistX)+":"+str(self.f_DistY)+":"+str(self.f_RangeRate)

a = Cluster()
print(a)

