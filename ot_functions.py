
# coding: utf-8

# In[2]:
#%%

import ReadCsv

# In[3]:


import ot_cluster


# In[4]:

def FunctionReadData():
    vClusters=ReadCsv.get_nofValidClusters(sample)
    return vClusters

def FunctionCreateClusters(egoRinfo, egoLinfo):
    vClus=FunctionReadData()
    validRClusters=[]
    validLClusters=[]
    for clusR in range(vClus[0]):
        cR=ot_cluster.Cluster()
        cRinfo=ReadCsv.get_RightInfoCluster(clusR,sample)
        cR.set_attributes(cRinfo[0],cRinfo[1],cRinfo[2],cRinfo[3], cRinfo[4], cRinfo[5], cRinfo[6], cRinfo[7], cRinfo[8],cRinfo[9])
        cR.set_filtercluster(egoRinfo.f_EgoSpeedSensorSpeedX,egoRinfo.f_EgoSpeedSensorSpeedY, egoRinfo.f_StaClsThrshld,egoRinfo.f_DynClsThrshld, egoRinfo.f_AmbClsThrshld)
        cR.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)
        cR.eval_asnewobject()
        validRClusters.append(cR)
        
    for clusL in range(vClus[1]):
        cL=ot_cluster.Cluster()
        cLinfo=ReadCsv.get_LeftInfoCluster(clusL,sample)
        cL.set_attributes(cLinfo[0],cLinfo[1],cLinfo[2],cLinfo[3],cLinfo[4], cLinfo[5], cLinfo[6], cLinfo[7], cLinfo[8], cLinfo[9])
        cL.set_filtercluster(egoLinfo.f_EgoSpeedSensorSpeedX,egoLinfo.f_EgoSpeedSensorSpeedY, egoLinfo.f_StaClsThrshld,egoLinfo.f_DynClsThrshld, egoLinfo.f_AmbClsThrshld)
        cL.eval_kinematics(egoLinfo.f_EgoSpeedClusterBased)
        cL.eval_asnewobject()
        validLClusters.append(cL)   
    return validRClusters,validLClusters

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


# In[5]:


for sample in range(10):

    egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(sample)
    egoInfoRight=ReadCsv.get_egoRightInfoCluster(sample)
    
    egoRInfo=ot_cluster.Ego()
    egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2])
    
    egoLInfo=ot_cluster.Ego()
    egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2])
    
    egoRInfo.eval_thresholds()
    egoLInfo.eval_thresholds()
    
    
    # In[9]:
    
    
    valLeftClusters=FunctionCreateClusters(egoRInfo, egoLInfo)[1]
    valRightClusters=FunctionCreateClusters(egoRInfo, egoLInfo)[0]
    a=len (valRightClusters)
    for i in range (a):
        if valRightClusters[i].s_ValidObjectID== 'True' :
            print(valRightClusters[i].s_ValidObjectID)

