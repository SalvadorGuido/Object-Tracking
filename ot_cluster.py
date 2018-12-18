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
# In[1]:
import numpy as np
from Kalman import *
# DISX = 2
# DISTY = 3
# RRATE = 4
# RANGLE = .45
# ListTrackedObjects = 0
# ListClusters = 0

# In[2]:
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

# In[3]:
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

    def set_attributes(self, newdx, newdy, newrrate, newangle, newsinangle, newconangle, newIRBField, newPBField, newRSPRRte, newRCS):
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

# In[4]:
class TrackedObjects(object):
    """docstring for TrackedObjects"""
    TRACKED_OBJS_CNTR = 0
    MAX_TRACKED_OBJS = 40
    MAX_COMBINE_CICLES = 10
    RAD_COMBINE = 5.0

    def __init__(self):
        #super(TrackedObjects, self).__init__()
        self.l_40TrackedObjs = []
        self.l_buffCombinedObjs = []
        self.d_objects = 0
        self.temp_obj = TrackedObject()


    def set_lifecounterup(self):
        # print("##########INSIDE set_lifecounterup") 
        for i in range(len(self.l_40TrackedObjs)):
            self.l_40TrackedObjs[i].i_lifeciclescoutner += 1
            if (self.MAX_COMBINE_CICLES == self.l_40TrackedObjs[i].i_lifeciclescoutner):
                self.l_40TrackedObjs[i].s_isobjcombinable = 'No-Combinable'
            elif(self.MAX_COMBINE_CICLES > self.l_40TrackedObjs[i].i_lifeciclescoutner):
                self.l_40TrackedObjs[i].s_isobjcombinable = 'Combinable'

    def eval_distance(self, obj_1, obj_2):
        # print("##########INSIDE eval_distance") 
        self.d_objects = np.sqrt(np.power(obj_1.f_DistY - obj_2.f_DistY, 2) + np.power(obj_1.f_DistX - obj_2.f_DistX, 2))

    def set_evalDistanceToEgo(self):
        # print("##########INSIDE set_evalDistanceToEgo ") 
        n_objects = len(self.l_40TrackedObjs)
        if self.l_40TrackedObjs: #     #### eval distance between ego and each object in l_40TrackedObjs
            for i in range(n_objects):
                self.l_40TrackedObjs[i].f_DistAbs = np.sqrt(np.power(self.l_40TrackedObjs[i].f_DistY, 2) + np.power(self.l_40TrackedObjs[i].f_DistX, 2))   

    def set_createNewObjects_v2(self, l_clusterstocreate, egoInfo):
        self.TRACKED_OBJS_CNTR
        if self.l_40TrackedObjs:
        	self.l_40TrackedObjs.sort(reverse = True,  key = lambda TrackedObject: TrackedObject.f_Priority)

        if l_clusterstocreate:
            if self.TRACKED_OBJS_CNTR == self.MAX_TRACKED_OBJS:
                
                del self.l_40TrackedObjs[-1:]
                newObject=TrackedObject()
                newObject.set_creationobjflag('NewObject')
                newObject.set_isobjcombinableflag('Combinable')
                # newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_RangeRate, l_newobjs[i].f_Vrelx, l_newobjs[i].f_Vrely, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy,  l_newobjs[i].f_ObjectPriority)
                newObject.set_createobject(l_clusterstocreate[0].f_DistX, l_clusterstocreate[0].f_DistY, l_clusterstocreate[0].f_Vabsx, l_clusterstocreate[0].f_Vabsy, l_clusterstocreate[0].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                    egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)                  
                self.l_40TrackedObjs.append(newObject)
            elif self.TRACKED_OBJS_CNTR < self.MAX_TRACKED_OBJS:
                n_objs_to_insert = len(l_clusterstocreate)
                for i in range(n_objs_to_insert):
                    self.TRACKED_OBJS_CNTR += 1
                    newObject=TrackedObject()
                    newObject.set_creationobjflag('NewObject')
                    newObject.set_isobjcombinableflag('Combinable')
                    # newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_RangeRate, l_newobjs[i].f_Vrelx, l_newobjs[i].f_Vrely, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy,  l_newobjs[i].f_ObjectPriority)
                    newObject.set_createobject(l_clusterstocreate[i].f_DistX, l_clusterstocreate[i].f_DistY, l_clusterstocreate[i].f_Vabsx, l_clusterstocreate[i].f_Vabsy, l_clusterstocreate[i].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                    egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)                  
                    if len(self.l_40TrackedObjs) < self.MAX_TRACKED_OBJS:
                    	self.l_40TrackedObjs.append(newObject)
                    
        self.l_40TrackedObjs.sort(reverse = True,  key = lambda TrackedObject: TrackedObject.f_Priority)            
    
    def set_createNewObjects(self, l_newobjs, egoInfo):
        ### receives sorted list of clusters to be objects, creates new TrackedObject() and append it to l_40TrackedObjs
        ### sorts l_40TrackedObjs based on probability
        # print("##########INSIDE set_createNewObjects ")
        # print("Number of l_40TrackedObjs::type::elements" +str(len(self.l_40TrackedObjs)) + "::" + str(type(self.l_40TrackedObjs)) + "::" +str(self.l_40TrackedObjs))
        # print("Number of l_newobjs::type::elements" +str(len(l_newobjs)) + "::" + str(type(l_newobjs)) + "::" +str(l_newobjs))
        n_newobjects = len(l_newobjs)
        self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
        # if self.l_40TrackedObjs:
            # print("NOT EMPTY l_40TrackedObjs")
            # print("Number of l_40TrackedObjs::type::elements" +str(len(self.l_40TrackedObjs)) + "::" + str(type(self.l_40TrackedObjs)) + "::" +str(self.l_40TrackedObjs))
            # self.l_40TrackedObjs.sort(reverse = True,  key= lambda TrackedObject: TrackedObject.f_Priority)
        if l_newobjs:
            # print("NOT EMPTY l_newobjs")
            if (self.MAX_TRACKED_OBJS - self.TRACKED_OBJS_CNTR) >= n_newobjects:
                # self.TRACKED_OBJS_CNTR += n_newobjects
                # print("abs((self.MAX_TRACKED_OBJS - self.TRACKED_OBJS_CNTR))" + str(abs((self.MAX_TRACKED_OBJS - self.TRACKED_OBJS_CNTR))))
                # print("n_newobjects" + str(n_newobjects))
                
                for i in range(n_newobjects):
                    newObject=TrackedObject()
                    newObject.set_creationobjflag('NewObject') 
                    newObject.set_isobjcombinableflag('Combinable')
                    # newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_RangeRate, l_newobjs[i].f_Vrelx, l_newobjs[i].f_Vrely, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy,  l_newobjs[i].f_ObjectPriority)
                    newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy, l_newobjs[i].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                        egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)                  
                    self.l_40TrackedObjs.append(newObject)
                    self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
                    # print("Number of l_40TrackedObjs" +str(len(self.l_40TrackedObjs)))

                # print("line 207 Number of Objects in l_40TrackedObjs if " + str(len(self.l_40TrackedObjs)))
                # print("line 208 Number of new objects if " + str(len(l_newobjs)) + "::type::" + str(type(l_newobjs)))
                # print(self.l_40TrackedObjs)
                self.l_40TrackedObjs.sort(reverse = True,  key= lambda TrackedObject: TrackedObject.f_Priority)

                # print("Counter in class if true " + str(self.TRACKED_OBJS_CNTR))
            else:
                
                # self.TRACKED_OBJS_CNTR += n_toinsert
                self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
                if self.MAX_TRACKED_OBJS == self.TRACKED_OBJS_CNTR:
                    del self.l_40TrackedObjs[-1:]
                    newObject=TrackedObject()
                    newObject.set_creationobjflag('NewObject')
                    newObject.set_isobjcombinableflag('Combinable')
                    ##########set_createobject(self, newdisx, newdisy, newvabsx, newvabsy, newprior, EgoSpeedClusterBased, EgoAccel, EgoSinYawA, EgoCosYawA, dt, YawRate, MountingCenterX, MountingCenterY):
                    newObject.set_createobject(l_newobjs[0].f_DistX, l_newobjs[0].f_DistY, l_newobjs[0].f_Vabsx, l_newobjs[0].f_Vabsy, l_newobjs[0].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                        egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)
                    self.l_40TrackedObjs.append(newObject)
                    self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
                    # print("Number of l_40TrackedObjs" +str(len(self.l_40TrackedObjs)))
                elif self.MAX_TRACKED_OBJS > self.TRACKED_OBJS_CNTR:
                    n_toinsert = self.MAX_TRACKED_OBJS - self.TRACKED_OBJS_CNTR
                    for i in range(n_toinsert):
                        newObject=TrackedObject()
                        newObject.set_creationobjflag('NewObject')
                        newObject.set_isobjcombinableflag('Combinable')
                        # newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_RangeRate, l_newobjs[i].f_Vrelx, l_newobjs[i].f_Vrely, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy,  l_newobjs[i].f_ObjectPriority)
                        newObject.set_createobject(l_newobjs[i].f_DistX, l_newobjs[i].f_DistY, l_newobjs[i].f_Vabsx, l_newobjs[i].f_Vabsy, l_newobjs[i].f_ObjectPriority, egoInfo.f_EgoSpeedClusterBased,
                            egoInfo.f_EgoAccel, egoInfo.f_EgoSinYawA, egoInfo.f_EgoCosYawA, egoInfo.dt, egoInfo.YawRate, egoInfo.MountingtoCenterX, egoInfo.MountingtoCenterY)
                        # newObject.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)            
                        self.l_40TrackedObjs.append(newObject)
                        self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
                        # print("Number of l_40TrackedObjs" +str(len(self.l_40TrackedObjs)))
                # print("line 233 Number of Objects in l_40TrackedObjs else" + str(len(self.l_40TrackedObjs)))
                # print("line 234 Number of new objects else " + str(len(l_newobjs)) + "::type::" + str(type(l_newobjs)))
                self.l_40TrackedObjs.sort(reverse = True,  key = lambda TrackedObject: TrackedObject.f_Priority)
                # print("line 236Tracked Objects Counter " + str(self.TRACKED_OBJS_CNTR))
        # else:
        #     pass# print("INSIDE set_createNewObjects EMPTY ARGUMENT")


    def set_evalCombineObjs(self):
        print("####################INSIDE set_evalCombineObjs####################") 
        n_objects = len(self.l_40TrackedObjs)
        # if n_objects > 0:            
        # if self.l_40TrackedObjs:     
        for p in range(n_objects):
            for j in range(n_objects):
                if (p < j) and ('AlreadyCombinedObj' != self.l_40TrackedObjs[p].s_creationobjflag) and ('AlreadyCombinedObj' != self.l_40TrackedObjs[j].s_creationobjflag):
                    self.eval_distance(self.l_40TrackedObjs[p], self.l_40TrackedObjs[j])
                    # print("OBJECTS EVALUATED::" + str(p) + "::" + str(j))
                    # print("Number of tracked objs in l_40TrackedObjs :::::" + str(len(self.l_40TrackedObjs)))
                    # print("Number of tracked objs in l_buffCombinedObjs ::" + str(len(self.l_buffCombinedObjs)))
                    # print("Distance between objects " + str(round(self.d_objects,4)))
                    # print("Obj(p) s_isobjcombinable " + str(self.l_40TrackedObjs[p].s_isobjcombinable))
                    # print("Obj(j) s_isobjcombinable " + str(self.l_40TrackedObjs[j].s_isobjcombinable))  
                    if (self.d_objects <= self.RAD_COMBINE) and ('Combinable' == self.l_40TrackedObjs[p].s_isobjcombinable) and ('Combinable' == self.l_40TrackedObjs[j].s_isobjcombinable):
                        # print("NewCombinedObject, Number of l_buffCombinedObjs before::" + str(len(self.l_buffCombinedObjs)))
                        self.set_createCombinedObjs(self.l_40TrackedObjs[p], self.l_40TrackedObjs[j])
                        # print("NewCombinedObject, Number of l_buffCombinedObjs after::" + str(len(self.l_buffCombinedObjs)))  
        
    def set_createCombinedObjs(self, obj_1, obj_2):
        # print("##########INSIDE set_createCombinedObjs") 
        # print("Number of tracked objs in l_40TrackedObjs ::" + str(len(self.l_40TrackedObjs)))
        # print("Number of tracked objs in l_buffCombinedObjs ::" + str(len(self.l_buffCombinedObjs)))
            
        # print("INDEX TO REMOVE " + str(indexremove))
        # print("INDEX TO INSERT " + str(indexinsert))
        obj_1.set_isobjcombinableflag('No-Combinable')
        obj_2.set_isobjcombinableflag('No-Combinable')
        obj_1.set_creationobjflag('AlreadyCombinedObj')
        obj_2.set_creationobjflag('AlreadyCombinedObj')  

        # self.temp_obj = obj_1
        # self.temp_obj = TrackedObject()
        self.temp_obj.f_DistX = (obj_1.f_DistX + obj_2.f_DistX)/2
        self.temp_obj.f_DistY = (obj_1.f_DistY + obj_2.f_DistY)/2
        self.temp_obj.f_DistAbs = (obj_1.f_DistAbs + obj_2.f_DistAbs)/2
        self.temp_obj.f_Vabsx = (obj_1.f_Vabsx + obj_2.f_Vabsx)/2
        self.temp_obj.f_Vabsy = (obj_1.f_Vabsy + obj_2.f_Vabsy)/2
        self.temp_obj.f_AccelX = (obj_1.f_AccelX + obj_2.f_AccelX)/2
        self.temp_obj.f_AccelY = (obj_1.f_AccelY + obj_2.f_AccelY)/2
        self.temp_obj.f_EgoSpeedClusterBased = (obj_1.f_EgoSpeedClusterBased + obj_2.f_EgoSpeedClusterBased)/2
        self.temp_obj.f_EgoAccel = None
        self.temp_obj.f_EgoSinYawA = None
        self.temp_obj.f_EgoCosYawA = None
        self.temp_obj.f_ObjLength = abs(obj_1.f_DistX - obj_2.f_DistX)
        self.temp_obj.f_ObjWidth = abs(obj_1.f_DistY - obj_2.f_DistY)        
        self.temp_obj.dt = None
        self.temp_obj.YawRate= None
        self.temp_obj.MountingCenterX = None
        self.temp_obj.MountingCenterY = None
        self.temp_obj.i_ClustersPerObject = []
        self.temp_obj.f_probExist = None
        self.temp_obj.f_probGhost = None
        self.temp_obj.i_ObjectID = None
        self.temp_obj.f_Priority = max(obj_1.f_Priority, obj_2.f_Priority)*0.7 + min(obj_1.f_Priority, obj_2.f_Priority)*0.3
        self.temp_obj.i_lifeciclescoutner = min(obj_1.i_lifeciclescoutner, obj_2.i_lifeciclescoutner)
        self.temp_obj.s_isobjcombinable = 'Combinable'
        self.temp_obj.s_creationobjflag = 'NewCombinedObject' #{'NewObject', 'AlreadyCombinedObj', 'NewCombinedObject'}
        self.temp_obj.f_Kalman = Kalman()
        self.l_buffCombinedObjs.append(self.temp_obj)
        # if indexinsert > indexremove:
        #     self.l_40TrackedObjs.pop(indexremove)
        #     self.l_40TrackedObjs.insert(indexinsert - 1, self.temp_obj)
        #     self.l_40TrackedObjs.pop(indexinsert)
        # elif indexinsert < indexremove:
        #     self.l_40TrackedObjs.pop(indexremove)
        #     self.l_40TrackedObjs.insert(indexinsert, self.temp_obj)
        #     self.l_40TrackedObjs.pop(indexinsert + 1)        

    def set_update40TrackedObjs(self):
        print("##########INSIDE set_update40TrackedObjs") 
        print("Number of Objects in l_40TrackedObjs " + str(len(self.l_40TrackedObjs)))
        print("Number of Objects in l_buffCombinedObjs " + str(len(self.l_buffCombinedObjs)))# + "::type::" + str(type(self.l_buffCombinedObjs)))
        # print(self.l_buffCombinedObjs)
        # if self.l_40TrackedObjs:
        n_alrcombobjs = len([p for p in self.l_40TrackedObjs if p.s_creationobjflag == 'AlreadyCombinedObj']) ###Number of objects already combined
        n_new_combobjs = len(self.l_buffCombinedObjs) ###Number of elements in buffer l_buffCombinedObjs

        if n_alrcombobjs > 0:
            print("Number of AlreadyCombinedObj::" + str(n_alrcombobjs))
            print("Number of Objs in l_40TrackedObjs before del::" + str(len(self.l_40TrackedObjs)))
            print("Number of Objs in l_buffCombinedObjs::" + str(len(self.l_buffCombinedObjs)))
            self.l_40TrackedObjs.sort(reverse = True,  key = lambda TrackedObject: TrackedObject.s_creationobjflag)
            del self.l_40TrackedObjs[-n_alrcombobjs:]
            print("Number of Objs in l_40TrackedObjs after del::" + str(len(self.l_40TrackedObjs)))
            print("Number of Objs in l_buffCombinedObjs:: after del::" + str(len(self.l_buffCombinedObjs)))
        self.TRACKED_OBJS_CNTR = len(self.l_40TrackedObjs)
        self.l_buffCombinedObjs.sort(reverse = True,  key = lambda TrackedObject: TrackedObject.f_Priority)
        if (self.TRACKED_OBJS_CNTR == self.MAX_TRACKED_OBJS) and (n_new_combobjs > 0):
        	del self.l_40TrackedObjs[-1:]
        	self.l_40TrackedObjs.append(self.l_buffCombinedObjs[0])
        elif self.l_buffCombinedObjs and (self.TRACKED_OBJS_CNTR < self.MAX_TRACKED_OBJS):
        	for j in range(self.MAX_TRACKED_OBJS - self.TRACKED_OBJS_CNTR):
        		print("l_buffCombinedObjs::type::elements" + str(type(self.l_buffCombinedObjs)) +str(self.l_buffCombinedObjs))
        		self.l_40TrackedObjs.append(self.l_buffCombinedObjs[j])            
            	
        # print("l_40TrackedObjs::type::elements" + str(type(self.l_40TrackedObjs)) +str(self.l_40TrackedObjs))
        # self.l_buffCombinedObjs.clear()
        # print("Number of Objs in l_40TrackedObjs after append::" + str(len(self.l_40TrackedObjs)))
        #     print("Number of Objs in l_buffCombinedObjs::" + str(len(self.l_buffCombinedObjs)))
        #     print("44444444444 l_buffCombinedObjs::type::elements" + str(type(self.l_buffCombinedObjs)) +str(self.l_buffCombinedObjs))
            
        # for i in range(n_alrcombobjs):
        #     if 'AlreadyCombinedObj' == self.l_40TrackedObjs[i].s_creationobjflag:
        #         self.l_40TrackedObjs.pop(i)
           
        
    def __str__(self):
        return "Class::TrackedObjects::N Tracked Objects"+str(len(self.l_40TrackedObjs))


# In[5]:    
class TrackedObject(object):
    """docstring for TrackedObject"""
# class Missile(object):
#   MAX_SPEED = 100  # all missiles accelerate up to this speed
#   ACCELERATION = 5  # rate of acceleration per game frame

#       def move(self):
#         self.speed += self.ACCELERATION
#         if self.speed > self.MAX_SPEED:
#               self.speed = self.MAX_SPEED
    
    def __init__(self):
        #super(TrackedObject, self).__init__()
        self.f_DistX = None
        self.f_DistY = None
        self.f_DistAbs = 0
        self.f_Vabsx = None
        self.f_Vabsy = None
        self.f_AccelX = 0
        self.f_AccelY = 0
        self.f_EgoSpeedClusterBased = 0
        self.f_EgoAccel = 0
        self.f_EgoSinYawA = 0
        self.f_EgoCosYawA = 0
        self.f_ObjLength = 0
        self.f_ObjWidth = 0
        self.dt = None
        self.YawRate= None
        self.MountingCenterX = 0
        self.MountingCenterY = 0

        self.i_ClustersPerObject = []
        self.f_probExist = 0
        self.f_probGhost = 0
        self.i_ObjectID = None
        self.f_Priority = 0
        self.i_lifeciclescoutner = 0
        self.s_isobjcombinable = None # {'Combinable','No-Combinable'}
        self.s_creationobjflag = None # {'NewObject', 'AlreadyCombinedObj', 'NewCombinedObject'}
        self.f_Kalman= Kalman()

    def set_createobject(self, newdisx, newdisy, newvabsx, newvabsy, newprior, EgoSpeedClusterBased, EgoAccel, EgoSinYawA, EgoCosYawA, dt, YawRate, MountingCenterX, MountingCenterY):
        self.f_DistX = newdisx
        self.f_DistY = newdisy
        self.f_Vabsx = newvabsx
        self.f_Vabsy = newvabsy
        self.f_AccelX = 0
        self.f_AccelY = 0
        self.f_EgoSpeedClusterBased = EgoSpeedClusterBased
        self.f_EgoAccel = EgoAccel
        self.f_EgoSinYawA = EgoSinYawA
        self.f_EgoCosYawA = EgoCosYawA
        self.dt = dt
        self.YawRate= YawRate
        self.MountingCenterX = MountingCenterX
        self.MountingCenterY = MountingCenterY
        self.f_Kalman.set_initialKalVal(newdisx, newdisy, newvabsx, newvabsy, dt,YawRate, EgoSinYawA, EgoCosYawA, EgoSpeedClusterBased, MountingCenterX,MountingCenterY, EgoAccel)
        # self.i_ClustersPerObject = []
        # self.f_probExist = None
        # self.f_probGhost = None
        # self.i_ObjectID = None
        self.f_Priority = newprior
    def set_isobjcombinableflag(self, newisobjcomb):
        self.s_isobjcombinable = newisobjcomb

    def set_creationobjflag(self, newcreatflag):
        self.s_creationobjflag = newcreatflag

    def set_mergeobjects(self):
        pass


# In[6]:
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