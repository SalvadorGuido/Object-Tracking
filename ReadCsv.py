
# coding: utf-8

# In[2]:


import pandas as pd

path="D:\Continental\AEP2018\T11.1c_LH_25_DEG_2018.06.27_at_20.53.27_radar-mi_1160_Long.csv" 
#path='/home/eln/Documents/Continental/AEP2018/AlgorithmsProject/radarKalman/T11radar.csv' 
radarInfo=pd.read_csv(path)
# In[18]:


#Listing headers of main table
noClusters=384
EMnoColumns=13
RSPnoColumns=24

headers=radarInfo.columns

indexLeftRawDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
indexRightRawDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
#Getting raw information of radars.
rawLSensorData=radarInfo[headers[indexLeftRawDataSensor:indexLeftRawDataSensor+(EMnoColumns*noClusters)+2]]
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fEgoSpeedClusterBased'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedX'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedY'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fCosYawAngle'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fSinYawAngle'])
rawLSensorData=rawLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fCycletime'])
rawLSensorData=rawLSensorData.join(radarInfo['LEFT SENSOR.DataProcCycle.VehDyn_DataProcCycle.Lateral.YawRate.YawRate'])
rawLSensorData=rawLSensorData.join(radarInfo['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LatPos'])
rawLSensorData=rawLSensorData.join(radarInfo['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPos'])
rawLSensorData=rawLSensorData.join(radarInfo['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPosToCoG'])


rawRSensorData=radarInfo[headers[indexRightRawDataSensor:indexRightRawDataSensor+(EMnoColumns*noClusters)+2]]
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fEgoSpeedClusterBased'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedX'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedY'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fCosYawAngle'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fSinYawAngle'])
rawRSensorData=rawRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fCycletime'])
rawRSensorData=rawRSensorData.join(radarInfo['RIGHT SENSOR.DataProcCycle.VehDyn_DataProcCycle.Lateral.YawRate.YawRate'])
rawRSensorData=rawRSensorData.join(radarInfo['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LatPos'])
rawRSensorData=rawRSensorData.join(radarInfo['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPos'])
rawRSensorData=rawRSensorData.join(radarInfo['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPosToCoG'])

#Listing headers of table of raw information.
rawLSheaders=rawLSensorData.columns
rawRSheaders=rawRSensorData.columns
#Dropping duplicates samples using column timestamp.
rawLSensorValidData=rawLSensorData.drop_duplicates(subset='SIM EM LEFT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp')
rawRSensorValidData=rawRSensorData.drop_duplicates(subset='SIM EM RIGHT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp')

rawLSensorValidData=rawLSensorValidData.reset_index()
rawRSensorValidData=rawRSensorValidData.reset_index()
#RSP data sensor
#RSP index time data 
indexLeftRSPTimeDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.ClustListHead.u_TimeStamp')
indexRightRSPTimeDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.ClustListHead.u_TimeStamp')

indexLeftRSPInitDataSensor=headers.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_VrelRad')
indexRightRSPInitDataSensor=headers.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_VrelRad')

rspLSensorData=radarInfo[headers[indexLeftRSPTimeDataSensor:indexLeftRSPInitDataSensor+(RSPnoColumns*noClusters)]]
rspLSensorData=rspLSensorData.join(radarInfo['SIM EM LEFT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp'])
rspRSensorData=radarInfo[headers[indexRightRSPTimeDataSensor:indexRightRSPInitDataSensor+(RSPnoColumns*noClusters)]]
rspRSensorData=rspRSensorData.join(radarInfo['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp'])

#Listing headers of table of rsp information.
rspLSheaders=rspLSensorData.columns
rspRSheaders=rspRSensorData.columns
#Dropping duplicates samples using column timestamp.
rspLSensorValidData=rspLSensorData.drop_duplicates(subset='SIM EM LEFT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp')
rspRSensorValidData=rspRSensorData.drop_duplicates(subset='SIM EM RIGHT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp')

rspLSensorValidData=rspLSensorValidData.reset_index()
rspRSensorValidData=rspRSensorValidData.reset_index()

indexLeftRSPDataSensor=rspLSheaders.get_loc('LEFT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_RangeRad')
indexRightRSPDataSensor=rspRSheaders.get_loc('RIGHT SENSOR.DataProcCycle.RSP2_ClusterListNS.a_Clusters[0].f_RangeRad')



# In[19]:


#get information for an specific cluster
def get_nofValidClusters(sample):
    left=rawLSensorValidData[rawLSheaders[1]][sample]
    right=rawRSensorValidData[rawRSheaders[1]][sample]
    return (right, left)

def get_LeftInfoCluster(cluster, sample):
    dx = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+2]][sample]
    dy = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+3]][sample]
    Rrate = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+4]][sample]
    angle = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+7]][sample]
    sinA = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+8]][sample]
    cosA = rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+9]][sample]
    IRBfield= rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+10]][sample]
    PBField= rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+11]][sample]
    RCS= rawLSensorValidData[rawLSheaders[(cluster*EMnoColumns)+5]][sample]
    rspRangeRad=rspLSensorValidData[rspLSheaders[indexLeftRSPDataSensor+(cluster*RSPnoColumns)]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA, IRBfield, PBField,rspRangeRad, RCS)

def get_RightInfoCluster(cluster, sample):
    dx = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+2]][sample]
    dy = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+3]][sample]
    Rrate = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+4]][sample]
    angle = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+7]][sample]
    sinA = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+8]][sample]
    cosA = rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+9]][sample]
    IRBfield= rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+10]][sample]
    PBField= rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+11]][sample]
    RCS= rawRSensorValidData[rawRSheaders[(cluster*EMnoColumns)+5]][sample]
    rspRangeRad=rspRSensorValidData[rspRSheaders[indexRightRSPDataSensor+(cluster*RSPnoColumns)]][sample]
    return (dx, dy, Rrate, angle, sinA, cosA, IRBfield, PBField,rspRangeRad, RCS)

def get_egoLeftInfoCluster(sample):
    EgoSpeed= rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fEgoSpeedClusterBased'][sample]
    EgoVx= rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedX'][sample]
    EgoVy= rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedY'][sample]
    EgoSinYawA= rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fSinYawAngle'][sample]
    EgoCosYawA= rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fCosYawAngle'][sample]
    dt = rawLSensorValidData['SIM EM LEFT.DataProcCycle.EMGlobalOutput.fCycletime'][sample]
    YawRate = rawLSensorValidData['LEFT SENSOR.DataProcCycle.VehDyn_DataProcCycle.Lateral.YawRate.YawRate'][sample]
    LatPos = rawLSensorValidData['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LatPos'][sample]
    LongPos = rawLSensorValidData['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPos'][sample]
    LongPosToCoG = rawLSensorValidData['LEFT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPosToCoG'][sample]

    return (EgoVx,EgoVy,EgoSpeed,EgoSinYawA,EgoCosYawA, dt, YawRate, LatPos, LongPos, LongPosToCoG)

def get_egoRightInfoCluster(sample):
    EgoSpeed= rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fEgoSpeedClusterBased'][sample]
    EgoVx= rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedX'][sample]
    EgoVy= rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobals.DaPGlobals.fEgoSpeedSensorSpeedY'][sample]
    EgoSinYawA= rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fSinYawAngle'][sample]
    EgoCosYawA= rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fCosYawAngle'][sample]
    dt = rawRSensorValidData['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.fCycletime'][sample]
    YawRate = rawRSensorValidData['RIGHT SENSOR.DataProcCycle.VehDyn_DataProcCycle.Lateral.YawRate.YawRate'][sample]
    LatPos = rawRSensorValidData['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LatPos'][sample]
    LongPos = rawRSensorValidData['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPos'][sample]
    LongPosToCoG = rawRSensorValidData['RIGHT SENSOR.DataProcCycle.VehPar_DataProcCycle.SensorMounting.LongPosToCoG'][sample]

    return (EgoVx,EgoVy,EgoSpeed,,EgoSinYawA,EgoCosYawA, dt, YawRate, LatPos, LongPos, LongPosToCoG)


# In[27]:

"""
print (get_nofValidClusters(2))
print (get_RightInfoCluster(0,1))
print (get_LeftInfoCluster(0,1))
print (get_egoLeftInfoCluster(4))
print (get_egoRightInfoCluster(2))

"""