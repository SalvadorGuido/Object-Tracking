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
import numpy as np
from Kalman import *
# DISX = 2
# DISTY = 3
# RRATE = 4
# RANGLE = .45
# ListTrackedObjects = 0
# ListClusters = 0

# In[1]:
class Ego(object):
    """docstring for Ego"""
    def __init__(self):
    # def __init__(self):
        #super(Ego, self).__init__()
        self.f_EgoSpeedSensorSpeedX = None
        self.f_EgoSpeedSensorSpeedY = None
        self.f_EgoSpeedClusterBased = None
        self.f_EgoAccel = None
        self.f_StaClsThrshld = None
        self.f_AmbClsThrshld = None
        self.f_DynClsThrshld = None
        self.f_EgoSinYawA = None
        self.f_EgoCosYawA = None
        self.dt = None
        self.YawRate= None
        self.MountingtoCenterX = None
        self.MountingtoCenterY = None


        # self.f_EgoSpeedSensorSpeedX = None
        # self.f_EgoSpeedSensorSpeedY = None
        # self.f_EgoSpeedClusterBased = None

    def set_EgoSpeeds(self,n_esssx, n_esssy, n_escb, n_esya, n_ecya, dt, YawRate, LatPos, LongPosToCoG, SigEgoAccel):
        self.f_EgoSpeedSensorSpeedX = n_esssx
        self.f_EgoSpeedSensorSpeedY = n_esssy
        self.f_EgoSpeedClusterBased = n_escb
        self.f_EgoSinYawA = n_esya
        self.f_EgoCosYawA = n_ecya
        self.dt = dt
        self.YawRate= YawRate
        self.MountingtoCenterX = LatPos
        self.MountingtoCenterY = LongPosToCoG
        self.f_EgoAccel = SigEgoAccel


    def eval_thresholds(self):
        self.f_StaClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.10, 1.00])
        self.f_AmbClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.25, 1.50])
        self.f_DynClsThrshld = np.interp(abs(self.f_EgoSpeedClusterBased),[0.00, 1.50], [0.38, 1.75])

# In[2]:
class Cluster(object):
    PI = np.pi 
    EM_CLU_VALID = 0
    EM_CLU_HRR_SCAN_BIT = 32
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
        self.f_RSP_RangeRad = None
        self.s_NumAssocObjs = None  
        self.iBestAssocObj = None  
        self.s_ClusterKinematicID = None # {'Static','Static-Ambig', 'Ambig', 'Dynamic'}
        self.f_VradIdeal = None
        self.f_AbsRangeRateDelta = None
        self.f_ObjectPriority = None
        self.s_ValidObjectID = None
        self.f_RCS = None
        self.f_Vrelx = None
        self.f_Vrely = None
        self.f_Vabsx = None
        self.f_Vabsy = None
        self.f_AZang = None
        self.f_clusID = None

    def set_attributes(self, newdx, newdy, newrrate, newangle, newsinangle, newconangle, newIRBField, newPBField, newRSPRRte, newRCS, newAZang, clusID):
        self.f_DistX = newdx
        self.f_DistY = newdy
        self.f_RangeRate = newrrate
        self.f_Angle = newangle
        self.f_SinAngle = newsinangle
        self.f_CosAngle = newconangle 
        self.u_InvalidReasonBitField = newIRBField
        self.u_PropertiesBitField = newPBField
        self.f_RSP_RangeRad = newRSPRRte
        self.f_RCS = newRCS
        self.f_AZang = newAZang
        self.f_clusID = clusID


    def set_filtercluster(self, vegoX, vegoY, StcThrhld, DynThrhld, AmbThrhld):
        self.f_VradIdeal = -((self.f_CosAngle*vegoX) + (self.f_SinAngle*vegoY)) 
        self.f_AbsRangeRateDelta = abs(self.f_RangeRate - self.f_VradIdeal)
        if self.f_AbsRangeRateDelta < AmbThrhld:
            if self.f_AbsRangeRateDelta > StcThrhld:
                self.s_ClusterKinematicID = 'Static-Ambig'
            else: self.s_ClusterKinematicID = 'Static'
        elif self.f_AbsRangeRateDelta > DynThrhld:
            self.s_ClusterKinematicID = 'Dynamic'
        elif self.f_AbsRangeRateDelta > AmbThrhld:
            self.s_ClusterKinematicID = 'Ambig'

            
    def eval_kinematics(self, egospeed): 
        self.f_Vrelx = self.f_RangeRate*self.f_CosAngle 
        self.f_Vrely = self.f_RangeRate*self.f_SinAngle 
        self.f_Vabsx = self.f_Vrelx + egospeed
        self.f_Vabsy = self.f_Vrely

    def eval_asnewobject(self):
        self.f_ObjectPriority = 0
        self.s_NumAssocObjs = 0
        if (0 == self.s_NumAssocObjs) and (self.EM_CLU_VALID == self.u_InvalidReasonBitField) and (self.s_ClusterKinematicID == 'Dynamic') and (self.EM_CLU_HRR_SCAN_BIT != self.u_PropertiesBitField):
            self.s_ValidObjectID = 'True'
        else: 
            self.s_ValidObjectID = 'False'
        self.f_ObjectPriority += np.interp(self.f_RSP_RangeRad, [10.0, 100.0], [33.0, 0.0])
        self.f_ObjectPriority += np.interp(self.f_RCS, [-40.0, 00.0], [00.0, 33.0])
        self.f_ObjectPriority += np.interp(self.f_RangeRate, [-10.0, 5.0], [33.0, 0.0])
        if self.f_RangeRate <-0.0001:
            fTTC =     -self.f_RSP_RangeRad/self.f_RangeRate
            self.f_ObjectPriority += np.interp(fTTC, [0.0, 5.0], [5.0, 0.0])
        if self.f_ObjectPriority > 90:
            self.f_ObjectPriority = 90
        return self.s_ValidObjectID, self.f_ObjectPriority

    def get_kinematics(self):
        print("Cluster Kinematics(vx, vy, ax, ay):" + str(self.e_Vrelx) + "::" + str(self.e_Vrely) + "::" + str(self.e_Arelx) + "::" + str(self.e_Arely))

    def __str__(self):
        return "Cluster:dx,xy,rrage:"+str(self.f_DistX)+":"+str(self.f_DistY)+":"+str(self.f_RangeRate)

# In[3]:
class TrackedObjects(object):
    """docstring for TrackedObjects"""
    TRACKEDCOUNTER = 0
    MAXTRACKEDOBJECTS = 40
    MAXCICLESALIVE = 10
    RADIOTOCOMBINE = 10

    def __init__(self):
        #super(TrackedObjects, self).__init__()
        self.list_40TrackedObjects = []
        self.d_objects = 0
        self.temp_obj = TrackedObject()

    def set_insertNewObjects(self, newposobj, egoInfo):
        n_newobjects = len(newposobj)
        # print("in class TrackedObjects no received " + str(n_newobjects))
        if abs((self.MAXTRACKEDOBJECTS - self.TRACKEDCOUNTER )) >= n_newobjects:
            self.TRACKEDCOUNTER += n_newobjects
            for i in range(n_newobjects):
                newObject=TrackedObject()
                # newObject.set_createobject(newposobj[i].f_DistX, newposobj[i].f_DistY, newposobj[i].f_RangeRate, newposobj[i].f_Vrelx, newposobj[i].f_Vrely, newposobj[i].f_Vabsx, newposobj[i].f_Vabsy,  newposobj[i].f_ObjectPriority)
                newObject.set_createobject(newposobj[i].f_DistX, newposobj[i].f_DistY, newposobj[i].f_Vabsx, newposobj[i].f_Vabsy, newposobj[i].f_ObjectPriority,  newposobj[i].f_RSP_RangeRad,  newposobj[i].f_AZang, 
                	egoInfo.f_EgoSpeedClusterBased, egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)
                # newObject.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)            
                self.list_40TrackedObjects.append(newObject)

            self.list_40TrackedObjects.sort(reverse = True,  key= lambda TrackedObject: TrackedObject.f_Priority)
            # print("Counter in class if true " + str(self.TRACKEDCOUNTER))
        else:
            n_toinsert = self.MAXTRACKEDOBJECTS - self.TRACKEDCOUNTER
            self.TRACKEDCOUNTER += n_toinsert
            if self.MAXTRACKEDOBJECTS == self.TRACKEDCOUNTER:
                del self.list_40TrackedObjects[-1:]
                newObject=TrackedObject()

                newObject.set_createobject(newposobj[0].f_DistX, newposobj[0].f_DistY, newposobj[0].f_Vabsx, newposobj[0].f_Vabsy, newposobj[0].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                    egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)
                self.list_40TrackedObjects.append(newObject)
            else:

                for i in range(n_toinsert):
                    newObject=TrackedObject()
                    # newObject.set_createobject(newposobj[i].f_DistX, newposobj[i].f_DistY, newposobj[i].f_RangeRate, newposobj[i].f_Vrelx, newposobj[i].f_Vrely, newposobj[i].f_Vabsx, newposobj[i].f_Vabsy,  newposobj[i].f_ObjectPriority)
                    newObject.set_createobject(newposobj[i].f_DistX, newposobj[i].f_DistY, newposobj[i].f_Vabsx, newposobj[i].f_Vabsy, newposobj[i].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                        egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)
                    # newObject.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)            
                    self.list_40TrackedObjects.append(newObject)
            self.list_40TrackedObjects.sort(reverse = True,  key= lambda TrackedObject: TrackedObject.f_Priority)
            # self.list_40TrackedObjects.append(newObject)
            
            # n_topop = abs(len(self.list_40TrackedObjects)-self.MAXTRACKEDOBJECTS)
            # del self.list_40TrackedObjects[-n_topop:]
            # print("Counter in class if false " + str(self.TRACKEDCOUNTER))

    def set_combinenewobjects(self):
        for i in range(len(self.list_40TrackedObjects)):
            self.list_40TrackedObjects[i].f_DistAbs = np.sqrt(np.power(list_40TrackedObjects[i].f_DistY) + np.power(list_40TrackedObjects[i].f_DistX))
        for i in range(len(self.list_40TrackedObjects)):
            for j in range(len(self.list_40TrackedObjects)):
                if i != j:
                    eval_distance(self.list_40TrackedObjects[i], self.list_40TrackedObjects[j])
                    if self.d_objects <= self.RADIOTOCOMBINE:
                        remove_object(self.list_40TrackedObjects[i], self.list_40TrackedObjects[j], i, j, self.list_40TrackedObjects)


    
    def remove_object(self, obj1, obj2, indexremove, idexinsert, listobjects):
        self.temp_obj = obj1
        self.temp_obj.f_DistX = (obj1.f_DistX + obj2.f_DistX)/2
        self.temp_obj.f_DistY = (obj1.f_DistY + obj2.f_DistY)/2
        self.temp_obj.f_DistAbs = (obj1 + obj2)/2
        self.temp_obj.f_Vabsx = (obj1.f_Vabsx + obj2.f_Vabsx)/2
        self.temp_obj.f_Vabsy = (obj1.f_Vabsy + obj2.f_Vabsy)/2
        self.temp_obj.f_AccelX = (obj1.f_AccelX + obj2.f_AccelX)/2
        self.temp_obj.f_AccelY = (obj1.f_AccelY + obj2.f_AccelY)/2
        self.temp_obj.f_EgoSpeedClusterBased = obj1.f_EgoSpeedClusterBased 
        self.temp_obj.f_EgoAccel = None
        self.temp_obj.f_EgoSinYawA = None
        self.temp_obj.f_EgoCosYawA = None
        self.temp_obj.dt = None
        self.temp_obj.YawRate= None
        self.temp_obj.MountingCenterX = None
        self.temp_obj.MountingCenterY = None
        self.i_ClustersPerObject = []
        self.f_probExist = None
        self.f_probGhost = None
        self.i_ObjectID = None
        self.f_Priority = (obj1.f_Priority + obj2.f_Priority)/2
        self.i_lifeciclescoutner = min(obj1.i_lifeciclescoutner, obj2.i_lifeciclescoutner)
        self.s_isobjectalive = 'Alive' 
        self.f_Kalman = Kalman()
        listobjects.pop(indexremove)
        listobjects.insert(idexinsert, self.temp_obj)


    def set_lifecounterup(self):
        for i in range(len(self.list_40TrackedObjects)):
            self.list_40TrackedObjects[i].i_lifeciclescoutner += 1
            if (10 == self.list_40TrackedObjects[i].i_lifeciclescoutner):
                self.list_40TrackedObjects[i].s_isobjectalive = 'Dead'
            elif(10 > self.list_40TrackedObjects[i].i_lifeciclescoutner):
                self.list_40TrackedObjects[i].s_isobjectalive = 'Alive'

    def eval_distance(self, obj1, obj2):
        self.d_objects = np.sqrt(np.power(obj1.f_DistY - obj2.f_DistY, 2) + np.power(obj1.f_DistX - obj2.f_DistX, 2))
        
            
        
    def __str__(self):
        return "TrackedObjects:"+str(self.list_40TrackedObjects)


# In[4]:    
class TrackedObject(object):
    """docstring for TrackedObject"""
    MAXCICLESALIVE = 10

# class Missile(object):
#   MAX_SPEED = 100  # all missiles accelerate up to this speed
#   ACCELERATION = 5  # rate of acceleration per game frame

#       def move(self):
#         self.speed += self.ACCELERATION
#         if self.speed > self.MAX_SPEED:
#               self.speed = self.MAX_SPEED
    
    def __init__(self):
        #super(TrackedObject, self).__init__()
        self.ciclesLive = None 
        self.f_DistX = None
        self.f_DistY = None
        self.f_DistAbs = None
        self.f_Vabsx = None
        self.f_Vabsy = None
        self.f_AccelX = None
        self.f_AccelY = None
        self.f_EgoSpeedClusterBased = None
        self.f_EgoAccel = None
        self.f_EgoSinYawA = None
        self.f_EgoCosYawA = None
        self.dt = None
        self.YawRate= None
        self.MountingCenterX = None
        self.MountingCenterY = None
        self.rangeRad= None
        self.AZang= None

        self.assocClus = []
        self.f_probExist = None
        self.f_probGhost = None
        self.i_ObjectID = None
        self.f_Priority = None
        self.i_lifeciclescoutner = 0
        self.s_isobjectalive = None # {'Alive','Dead'}
        self.f_Kalman= Kalman()

    def set_createobject(self, newdisx, newdisy, newvabsx, newvabsy, newprior, rangeRad, AZang, EgoSpeedClusterBased,EgoAccel, EgoSinYawA, EgoCosYawA, dt,YawRate,MountingCenterX,MountingCenterY):
        self.f_DistX = newdisx
        self.f_DistY = newdisy
        self.f_Vabsx = newvabsx
        self.f_Vabsy = newvabsy
        self.f_AccelX = None
        self.f_AccelY = None
        self.f_EgoSpeedClusterBased= EgoSpeedClusterBased
        self.f_EgoAccel = EgoAccel
        self.f_EgoSinYawA = EgoSinYawA
        self.f_EgoCosYawA = EgoCosYawA
        self.dt = dt
        self.YawRate= YawRate
        self.MountingCenterX = MountingCenterX
        self.MountingCenterY = MountingCenterY
        self.rangeRad= rangeRad
        self.AZang= AZang
        self.f_Kalman.set_initialKalVal(newdisx, newdisy, newvabsx, newvabsy, dt,YawRate, rangeRad, AZang, EgoSinYawA, EgoCosYawA, EgoSpeedClusterBased, MountingCenterX,MountingCenterY, EgoAccel)
        # self.i_ClustersPerObject = []
        # self.f_probExist = None
        # self.f_probGhost = None
        # self.i_ObjectID = None
        self.f_Priority = newprior
    def set_KalmanEstimation(self):
    	self.f_Kalman.Matrix_A_P_Q_H_R_I()
		self.f_Kalman.OldStateVector(ciclesLive)
		self.f_Kalman.RelativeVelocities()
		self.f_Kalman.AceleratioFramework()
		self.f_Kalman.KalmanFilter_Predict()

	def set_KalmanCorrection(self):
		for clust in range(self.assocClus):
			


    def set_mergeobjects(self):
        pass


# In[5]:
class SampleClusters(object):
    """docstring for SampleClusters"""
    def __init__(self):
        #super(SampleClusters, self).__init__()
        self.validClustersPerSample = None
        self.listofValidClusters = None
        



# def FunctionReadData():
#     return None

# def FunctionCreateClusters():
#     return None

# def FunctionFilterClusters():
#     return None

# def FunctionCreateObjects():
#     return None

# def FunctionMergeObjects():
#     return None

# def FunctionTrackingObjects():
#     return None

# def FunctionGraphicalInterface():
#     return None





# a = Cluster()
# print(a)

# a.set_attributes(DISX,DISTY,RRATE,RANGLE)
# print(a)
# a.eval_kinematics()
# a.get_kinematics()