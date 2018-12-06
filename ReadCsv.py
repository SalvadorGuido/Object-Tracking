
# coding: utf-8

# In[67]:


import numpy as np
import pandas as pd
path="D:\Continental\AEP2018\T11.1c_LH_25_DEG_2018.06.27_at_20.53.27_radar-mi_1160_Long.csv" 
radarInfo=pd.read_csv(path)


# In[216]:


#Listing headers of main table.
headers=radarInfo.columns

indexLeftRawDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
indexRightRawDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
#Getting raw information of radars.
rawLSensorData=radarInfo[headers[indexLeftRawDataSensor:indexLeftRawDataSensor+(13*384)+2]]
rawRSensorData=radarInfo[headers[indexRightRawDataSensor:indexRightRawDataSensor+(13*384)+2]]
#Listing headers of table of raw information.
rawLSheaders=rawLSensorData.columns
rawRSheaders=rawRSensorData.columns
#Dropping duplicates samples using column timestamp.
rawLSensorValidData=rawLSensorData.drop_duplicates(subset='LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
rawRSensorValidData=rawRSensorData.drop_duplicates(subset='RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')

rawLSensorValidData=rawLSensorValidData.reset_index()
rawRSensorValidData=rawRSensorValidData.reset_index()
#RSP data sensor
#RSP index time data 
indexLeftRSPTimeDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.ClustListHead.u_TimeStamp')
indexRightRSPTimeDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.ClustListHead.u_TimeStamp')

indexLeftRSPInitDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_VrelRad')
indexRightRSPInitDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_VrelRad')

rspLSensorData=radarInfo[headers[indexLeftRSPTimeDataSensor:indexLeftRSPInitDataSensor+(24*384)]]
rspLSensorData=rspLSensorData.join(radarInfo['LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp'])

rspRSensorData=radarInfo[headers[indexRightRSPTimeDataSensor:indexRightRSPInitDataSensor+(24*384)]]
rspRSensorData=rspRSensorData.join(radarInfo['RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp'])
#Listing headers of table of rsp information.
rspLSheaders=rspLSensorData.columns
rspRSheaders=rspRSensorData.columns
#Dropping duplicates samples using column timestamp.
rspLSensorValidData=rspLSensorData.drop_duplicates(subset='LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
rspRSensorValidData=rspRSensorData.drop_duplicates(subset='RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')

rspLSensorValidData=rspLSensorValidData.reset_index()
rspRSensorValidData=rspRSensorValidData.reset_index()

indexLeftRSPDataSensor=rspLSheaders.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_RangeRad')
indexRightRSPDataSensor=rspRSheaders.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_RangeRad')


# In[217]:


#get information for an specific cluster
def get_nofValidClusters(sample):
    left=rawLSensorValidData[rawLSheaders[1]][sample]
    right=rawRSensorValidData[rawRSheaders[1]][sample]
    return (right, left)

def get_LeftInfoCluster(cluster, sample):
    dx = rawLSensorValidData[rawLSheaders[(cluster*13)+2]][sample]
    dy = rawLSensorValidData[rawLSheaders[(cluster*13)+3]][sample]
    Rrate = rawLSensorValidData[rawLSheaders[(cluster*13)+4]][sample]
    angle = rawLSensorValidData[rawLSheaders[(cluster*13)+7]][sample]
    sinA = rawLSensorValidData[rawLSheaders[(cluster*13)+8]][sample]
    cosA = rawLSensorValidData[rawLSheaders[(cluster*13)+9]][sample]
    rspRangeRad=rspLSensorValidData[rspLSheaders[indexLeftRSPDataSensor+(cluster*24)]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA,rspRangeRad)

def get_RightInfoCluster(cluster, sample):
    dx = rawRSensorValidData[rawRSheaders[(cluster*13)+2]][sample]
    dy = rawRSensorValidData[rawRSheaders[(cluster*13)+3]][sample]
    Rrate = rawRSensorValidData[rawRSheaders[(cluster*13)+4]][sample]
    angle = rawRSensorValidData[rawRSheaders[(cluster*13)+7]][sample]
    sinA = rawRSensorValidData[rawRSheaders[(cluster*13)+8]][sample]
    cosA = rawRSensorValidData[rawRSheaders[(cluster*13)+9]][sample]
    rspRangeRad=rspRSensorValidData[rspRSheaders[indexRightRSPDataSensor+(cluster*24)]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA,rspRangeRad)
    


# In[219]:


print (get_nofValidClusters(2))
print (get_RightInfoCluster(0,1))
print (get_LeftInfoCluster(0,1))

