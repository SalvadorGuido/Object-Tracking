
# coding: utf-8

# In[1]:


import numpy as np
import pandas as pd
path="D:\Continental\AEP2018\T11.1c_LH_25_DEG_2018.06.27_at_20.53.27_radar-mi_1160.csv" 
x=pd.read_csv(path)


# In[4]:


#Listing headers of main table.
headers=x.columns
#Getting information of interest.
RLSensor=x[headers[10:((13*385)-1)]]
RRSensor=x[headers[((13*385)+9):len(headers)-7]]
#Listing headers of table of interest.
RLSheaders=RLSensor.columns
RRSheaders=RRSensor.columns
#Dropping duplicates samples using column timestamp.
RLSenValSam=RLSensor.drop_duplicates(subset='LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
RRSenValSam=RRSensor.drop_duplicates(subset='RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')

#function to get number of valid clusters for an specific sample
def get_nofValidClusters(sample):
    left=RLSenValSam[RLSheaders[1]][sample]
    right=RRSenValSam[RRSheaders[1]][sample]
    return (right, left)

#get information of a cluster in an specific sample of left radar
def get_LeftInfoCluster(cluster, sample):
    dx = RLSenValSam[RLSheaders[(cluster*13)+2]][sample]
    dy = RLSenValSam[RLSheaders[(cluster*13)+3]][sample]
    Rrate = RLSenValSam[RLSheaders[(cluster*13)+4]][sample]
    angle = RLSenValSam[RLSheaders[(cluster*13)+7]][sample]
    sinA = RLSenValSam[RLSheaders[(cluster*13)+8]][sample]
    cosA = RLSenValSam[RLSheaders[(cluster*13)+9]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA)
#get information of a cluster in an specific sample of right radar
def get_RightInfoCluster(cluster, sample):
    dx = RRSenValSam[RRSheaders[(cluster*13)+2]][sample]
    dy = RRSenValSam[RRSheaders[(cluster*13)+3]][sample]
    Rrate = RRSenValSam[RRSheaders[(cluster*13)+4]][sample]
    angle = RRSenValSam[RRSheaders[(cluster*13)+7]][sample]
    sinA = RRSenValSam[RRSheaders[(cluster*13)+8]][sample]
    cosA = RRSenValSam[RRSheaders[(cluster*13)+9]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA)
    


# Test of functions
print (get_nofValidClusters(2))
print (get_LeftInfoCluster(0,1))
print (get_RightInfoCluster(0,3))

