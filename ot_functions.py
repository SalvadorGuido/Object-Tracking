
# coding: utf-8

# In[2]:
#%%

import ReadCsv

# In[3]:

import ot_cluster
from time import sleep 
import os



# In[4]:
def cls():
    if os.name=='posix':
        !clear
    else:
        !clc
     
    
def FunctionReadData(sample):
    vClusters=ReadCsv.get_nofValidClusters(sample)
    return vClusters

def FunctionCreateClusters(sample, egoRinfo, egoLinfo):
    vClus=FunctionReadData(sample)
    validRClusters=[]
    validLClusters=[]
    dynRClus=[]
    dynLClus=[]
    for clusR in range(vClus[0]):
        cR=ot_cluster.Cluster()
        cRinfo=ReadCsv.get_RightInfoCluster(clusR,sample)
        cR.set_attributes(cRinfo[0],cRinfo[1],cRinfo[2],cRinfo[3], cRinfo[4], cRinfo[5], cRinfo[6], cRinfo[7], cRinfo[8],cRinfo[9], cRinfo[10], clusR)
        cR.set_filtercluster(egoRinfo.f_EgoSpeedSensorSpeedX,egoRinfo.f_EgoSpeedSensorSpeedY, egoRinfo.f_StaClsThrshld,egoRinfo.f_DynClsThrshld, egoRinfo.f_AmbClsThrshld)
        cR.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)
        cR.eval_asnewobject()
        if cR.s_ClusterKinematicID == 'Dynamic':
            dynRClus.append(cR)
        validRClusters.append(cR)
        
    for clusL in range(vClus[1]):
        cL=ot_cluster.Cluster()
        cLinfo=ReadCsv.get_LeftInfoCluster(clusL,sample)
        cL.set_attributes(cLinfo[0],cLinfo[1],cLinfo[2],cLinfo[3],cLinfo[4], cLinfo[5], cLinfo[6], cLinfo[7], cLinfo[8], cLinfo[9], cLinfo[10], clusL)
        cL.set_filtercluster(egoLinfo.f_EgoSpeedSensorSpeedX,egoLinfo.f_EgoSpeedSensorSpeedY, egoLinfo.f_StaClsThrshld,egoLinfo.f_DynClsThrshld, egoLinfo.f_AmbClsThrshld)
        cL.eval_kinematics(egoLinfo.f_EgoSpeedClusterBased)
        cL.eval_asnewobject()
        if cL.s_ClusterKinematicID == 'Dynamic':
            dynLClus.append(cL)
        validLClusters.append(cL)   
    return validRClusters,validLClusters, dynRClus, dynLClus


def FunctionFilterClusters():
    return None

def FunctionCreateObjects(valLeftClusters, valRightClusters, l_trackedobj, r_trackedobj, egoRInfo, egoLinfo):
    
#    print("IN FUNCTION CREATE OBJECTS")
    L_ValidClustersObjects=[]
    R_ValidClustersObjects=[]
    L_nvalidclusters=len(valLeftClusters)
    R_nvalidclusters=len(valRightClusters)
    
    for i in range (L_nvalidclusters):
        if valLeftClusters[i].s_ValidObjectID == 'True':
            L_ValidClustersObjects.append(valLeftClusters[i])
            

    for i in range (R_nvalidclusters):
        if valRightClusters[i].s_ValidObjectID == 'True':
            R_ValidClustersObjects.append(valRightClusters[i])
            
    L_ValidClustersObjects.sort(reverse = True,  key= lambda Cluster: Cluster.f_ObjectPriority)
    R_ValidClustersObjects.sort(reverse = True, key= lambda Cluster: Cluster.f_ObjectPriority)
    #print(R_ValidClustersObjects[1].f_ObjectPriority)
    #print(L_ValidClustersObjects[1].f_ObjectPriority)
    l_trackedobj.set_insertNewObjects(L_ValidClustersObjects, egoLInfo)
    l_trackedobj.set_lifecounterup()
    # print("in function FunctionCreateObjects ************************left")
    r_trackedobj.set_insertNewObjects(R_ValidClustersObjects, egoRInfo)
    r_trackedobj.set_lifecounterup()
    # print("in function FunctionCreateObjects &&&&&&&&&&&&&&  Right")


    # return L_ValidClustersObjects, R_ValidClustersObjects
    return l_trackedobj, r_trackedobj

def FunctionMergeObjects():
    return None

def FunctionTrackingObjects():
    return None

def FunctionGraphicalInterface():
    return None


# In[5]:

LeftTrackedObjects = ot_cluster.TrackedObjects()
RightTrackedObjects = ot_cluster.TrackedObjects()

CICLES_TO_RUN = 60
DELAY_IN_S = 0.001

for sample in range(CICLES_TO_RUN):
    sleep(DELAY_IN_S)
    cls()
#    !clear
 #   sample = 200
    egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(sample)
    egoInfoRight=ReadCsv.get_egoRightInfoCluster(sample)

    
    egoRInfo=ot_cluster.Ego()
    egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2],egoInfoRight[3],egoInfoRight[4], egoInfoRight[5], egoInfoRight[6], egoInfoRight[7], egoInfoRight[8], egoInfoRight[9])
    
    egoLInfo=ot_cluster.Ego()
    egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2],egoInfoLeft[3],egoInfoLeft[4], egoInfoLeft[5], egoInfoLeft[6], egoInfoLeft[7] ,egoInfoLeft[8] ,egoInfoLeft[9])
    
    egoRInfo.eval_thresholds()
    egoLInfo.eval_thresholds()

    [valRightClusters, valLeftClusters, dynRClus, dynLClus]  = FunctionCreateClusters(sample, egoRInfo, egoLInfo)
    

    FunctionCreateObjects(valLeftClusters, valRightClusters, LeftTrackedObjects, RightTrackedObjects, egoRInfo, egoLInfo)

    print("sample:" + str(sample))
    if len(LeftTrackedObjects.list_40TrackedObjects) > 0:
        
        print(LeftTrackedObjects.TRACKEDCOUNTER)
        print("LeftTrackedObjects kalman :" + str(sample))
        LeftTrackedObjects.list_40TrackedObjects[0].set_KalmanEstimation()
        #print(LeftTrackedObjects.list_40TrackedObjects[0].f_Kalman.X)
        LeftTrackedObjects.list_40TrackedObjects[0].set_AssocClusters(dynLClus)
        
        #LeftTrackedObjects.list_40TrackedObjects[0].set_KalmanCorrection()

    if len(RightTrackedObjects.list_40TrackedObjects) > 0:
        print(RightTrackedObjects.TRACKEDCOUNTER)
        print("RightTrackedObjects kalman:" + str(sample))
        RightTrackedObjects.list_40TrackedObjects[0].set_KalmanEstimation()
        #print(RightTrackedObjects.list_40TrackedObjects[0].f_Kalman.X)
        RightTrackedObjects.list_40TrackedObjects[0].set_AssocClusters(dynRClus)
        #RightTrackedObjects.list_40TrackedObjects[0].set_KalmanCorrection()
        print(RightTrackedObjects.list_40TrackedObjects[0].f_Kalman.X)


    # [sorteda, sortedb] = FunctionCreateObjects(valLeftClusters, valRightClusters)
    # for i in range (len(sorteda)):
    #     #print("###############################################")
    #     print("Left objects:" + str(sorteda[i].f_ObjectPriority) + "No of objects " + str(len(sorteda)))

        
    # for i in range (len(sortedb)):
    #     #print("+++++++++++++++++++++++++++++++++++++++++++++++")
    #     print("Right objects:" + str(sortedb[i].f_ObjectPriority) + "No of objects " + str(len(sortedb)))

    # In[9]:
    
    
# valLeftClusters  = FunctionCreateClusters(egoRInfoi egoLInfo)[1]
# valRightClusters = FunctionCreateClusters(egoRInfo, egoLInfo)[0]


# a=len (valRightClusters)
# for i in range (a):
#     if valRightClusters[i].s_ValidObjectID == 'True':
#         print(valRightClusters[i].s_ValidObjectID)

# a=len (valLeftClusters)
# for i in range (a):
#     if valLeftClusters[i].s_ValidObjectID == 'True':
#         print(valLeftClusters[i].s_ValidObjectID)
