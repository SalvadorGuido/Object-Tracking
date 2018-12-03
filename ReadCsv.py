
# coding: utf-8

# In[3]:


import numpy as np
import pandas as pd
path="D:\Continental\AEP2018\T11.1c_LH_25_DEG_2018.06.27_at_20.53.27_radar-mi_1160.csv" 
x=pd.read_csv(path)


# In[48]:


len(x.columns)


# In[13]:


headers=list(x)

RRTimeStamp=x['SIM EM LEFT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp']
RLTimeStamp=x['SIM EM RIGHT.DataProcCycle.EMGlobalOutput.sSigHeader.uiTimeStamp']
RLSensor=x[headers[10:((13*385)-1)]]
RRSensor=x[headers[((13*385)+9):len(x.columns)-7]]
RLSheaders=RLSensor.columns
#get information for an specific cluster
cluster=3
RLSensor[RLSheaders[(cluster*13)+2:(cluster*13)+15]][1000:1001]


# In[20]:


RLSenValSam=RLSensor.drop_duplicates(subset='LEFT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
RRSenValSam=RRSensor.drop_duplicates(subset='RIGHT SENSOR.DataProcCycle.EM_ClusterList.u_TimeStamp')
print(RRSenValSam)
#get information for an specific cluster
cluster=3
RLSenValSam[RLSheaders[(cluster*13)+2:(cluster*13)+15]][6:7]

