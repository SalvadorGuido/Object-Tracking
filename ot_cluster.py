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
    	self.f_RangeRate = newrrate
    def __str__(self):
        return "Cluster:dx,xy,rrage:"+str(self.f_DistX)+":"+str(self.f_DistY)+":"+str(self.f_RangeRate)

a = Cluster()
print(a)