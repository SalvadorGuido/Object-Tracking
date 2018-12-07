
# coding: utf-8

# In[1]:


import ReadCsv 


# In[16]:


import ot_cluster


# In[17]:


sample = 1
def FunctionReadData():
    vClusters=ReadCsv.get_nofValidClusters(sample)
    return vClusters

def FunctionCreateClusters():
    vClus=FunctionReadData()
    validRClusters=[]
    validLClusters=[]
    for clusR in range(vClus[0]):
        cR=ot_cluster.Cluster()
        cRinfo=ReadCsv.get_RightInfoCluster(clusR,sample)
        cR.set_attributes(cRinfo[0],cRinfo[1],cRinfo[2],cRinfo[3], cRinfo[4], cRinfo[5], cRinfo[6], cRinfo[7], cRinfo[8])
        validRClusters.append(cR) 
    for clusL in range(vClus[1]):
        cL=ot_cluster.Cluster()
        cLinfo=ReadCsv.get_LeftInfoCluster(clusL,sample)
        cL.set_attributes(cLinfo[0],cLinfo[1],cLinfo[2],cLinfo[3],cLinfo[4], cLinfo[5], cLinfo[6], cLinfo[7], cLinfo[8])
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
#codigo en matlab de los filtros


# In[18]:


valLeftClusters=FunctionCreateClusters()[1]
valRightClusters=FunctionCreateClusters()[0]
sample=1
egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(sample)
egoInfoRight=ReadCsv.get_egoRightInfoCluster(sample)
egoRInfo=ot_cluster.Ego()
egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2])
egoLInfo=ot_cluster.Ego()
egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2])
egoRInfo.eval_thresholds()
egoLInfo.eval_thresholds()

print(egoLInfo.f_EgoSpeedSensorSpeedX)
print(egoLInfo.f_EgoSpeedSensorSpeedY)
print(egoLInfo.f_EgoSpeedClusterBased)
print(egoLInfo.f_StaClsThrshld)
print(egoLInfo.f_AmbClsThrshld)
egoLInfo.f_DynClsThrshld

