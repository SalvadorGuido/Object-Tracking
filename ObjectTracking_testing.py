import matplotlib.pyplot as plt
import numpy as np
import matplotlib.path as mpath
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from matplotlib.colors import ListedColormap
import time
import random



# coding: utf-8

# In[2]:


import ReadCsv
import ot_cluster


# In[3]:




# In[4]:



def FunctionReadData(s):
    vClusters=ReadCsv.get_nofValidClusters(s)
    return vClusters

def FunctionCreateClusters(egoRinfo, egoLinfo,s):
    vClus=FunctionReadData(s)
    validRClusters=[]
    validLClusters=[]
    for clusR in range(vClus[0]):
        cR=ot_cluster.Cluster()
        cRinfo=ReadCsv.get_RightInfoCluster(clusR,s)
        cR.set_attributes(cRinfo[0],cRinfo[1],cRinfo[2],cRinfo[3], cRinfo[4], cRinfo[5], cRinfo[6], cRinfo[7], cRinfo[8],cRinfo[9])
        cR.set_filtercluster(egoRinfo.f_EgoSpeedSensorSpeedX,egoRinfo.f_EgoSpeedSensorSpeedY, egoRinfo.f_StaClsThrshld,egoRinfo.f_DynClsThrshld, egoRinfo.f_AmbClsThrshld)
        cR.eval_kinematics(egoRinfo.f_EgoSpeedClusterBased)
        cR.eval_asnewobject()
        validRClusters.append(cR) 
    for clusL in range(vClus[1]):
        cL=ot_cluster.Cluster()
        cLinfo=ReadCsv.get_LeftInfoCluster(clusL,s)
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


# GI Functions

def label(xy, text):
    y = xy[1] - 0.15  # shift y-value for label so that it's below the artist
    plt.text(xy[0], y, text, ha="center", family='sans-serif', size=14)

def obj_centroid(size, pos):
	x_sum = 0
	y_sum = 0
	for i in range(size):
		x_sum += pos[i][0]
		y_sum += pos[i][1]

	return((x_sum/size,y_sum/size))

def obj_height(size, pos):
	smallest = 0
	largest = 0
	for i in range(size):
		if pos[i][1] > largest:
			largest = pos[i][1]
		if pos[i][1] < smallest:
			smallest = pos[i][1]
	return largest-smallest

def obj_width(size, pos):
	smallest = 0
	largest = 0
	for i in range(size):
		if pos[i][0] > largest:
			largest = pos[i][0]
		if pos[i][0] < smallest:
			smallest = pos[i][0]
	return largest-smallest

def create_figure(size,pos):
	# ob_cen = obj_centroid(size,pos)
	# ob_hei = obj_height(size,pos)
	# ob_wid = obj_width(size, pos)
	ob_cen = pos
	ob_hei = 1
	ob_wid = 1

	return mpatches.Rectangle(ob_cen, ob_wid, ob_hei)



# In[5]:
# Number of samples
sample = 20
#sample = 6320

egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(sample)
egoInfoRight=ReadCsv.get_egoRightInfoCluster(sample)


egoRInfo=ot_cluster.Ego()
egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2],egoInfoRight[3],egoInfoRight[4], egoInfoRight[5], egoInfoRight[6], egoInfoRight[7], egoInfoRight[8], egoInfoRight[9])

egoLInfo=ot_cluster.Ego()
egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2],egoInfoLeft[3],egoInfoLeft[4], egoInfoLeft[5], egoInfoLeft[6], egoInfoLeft[7] ,egoInfoLeft[8] ,egoInfoLeft[9])

egoRInfo.eval_thresholds()
egoLInfo.eval_thresholds()


radarOffset = abs(egoRInfo.MountingtoCenterY) + abs(egoLInfo.MountingtoCenterY)


# In[6]:

valLeftClusters  = FunctionCreateClusters(egoRInfo, egoLInfo,0)[1]
valRightClusters = FunctionCreateClusters(egoRInfo, egoLInfo,0)[0]
# a=len (valLeftClusters)
# for i in range (a):
#     print(valRightClusters[i].f_ObjectPriority)

fig, ax = plt.subplots()
patchesL = []
patchesR = []
patch_colors = []
#egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(1)

#egoLInfo=ot_cluster.Ego()
#egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2])

#egoLInfo.eval_thresholds()
#valLeftClusters=FunctionCreateClusters(egoRInfo, egoLInfo,40)[1]

max_clusters = 200
n_clusters = len(valLeftClusters)
    
for j in range(max_clusters):
    if(j > n_clusters-1):
        temp_tuple = (-100,-100)
        r = create_figure(5, temp_tuple)
        patchesL.append(r)
        print(r)
    else:
        temp_tuple = (-valLeftClusters[j].f_DistY,valLeftClusters[j].f_DistX)
        if valLeftClusters[j].s_ClusterKinematicID == "Dynamic":
            r = create_figure(5, temp_tuple)
            patchesL.append(r)
            patch_colors.append("red")
            print(r)
        elif valLeftClusters[j].s_ClusterKinematicID == "Static":
            r = create_figure(5, temp_tuple)
            patchesL.append(r)
            patch_colors.append("red")
        elif valLeftClusters[j].s_ClusterKinematicID == "Ambig":
            r = create_figure(5, temp_tuple)
            patchesL.append(r)
            patch_colors.append("red")
        elif valLeftClusters[j].s_ClusterKinematicID == "Static-Ambig":
            r = create_figure(5, temp_tuple)
            patchesL.append(r)
            patch_colors.append("red")

n_clusters = len(valRightClusters)
    
for j in range(max_clusters):
    if(j > n_clusters-1):
        temp_tuple = (-100,-100)
        r = create_figure(5, temp_tuple)
        patchesR.append(r)
        patch_colors.append("red")
        print(r)
    else:
        temp_tuple = (-valRightClusters[j].f_DistY,valRightClusters[j].f_DistX)
        if valRightClusters[j].s_ClusterKinematicID == "Dynamic":
            r = create_figure(5, temp_tuple)
            patchesR.append(r)
            patch_colors.append("red")
            print(r)
        elif valRightClusters[j].s_ClusterKinematicID == "Static":
            r = create_figure(5, temp_tuple)
            patchesR.append(r)
            patch_colors.append("red")
            print(r)
        elif valRightClusters[j].s_ClusterKinematicID == "Ambig":
            r = create_figure(5, temp_tuple)
            patchesR.append(r)
            patch_colors.append("red")
            print(r)
        elif valRightClusters[j].s_ClusterKinematicID == "Static-Ambig":
            r = create_figure(5, temp_tuple)
            patchesR.append(r)
            patch_colors.append("red")
            print(r)
        else:
            r = create_figure(5, temp_tuple)
            patchesR.append(r)
            patch_colors.append("red")
            print(r)
            


addedPatch = patchesL + patchesR
#colors = np.linspace(0, 1, len(addedPatch))
#collection = PatchCollection(addedPatch)

tcmap = ListedColormap(patch_colors)

collection = PatchCollection(addedPatch, cmap=tcmap, alpha=0.3)

#collection.set_array(np.array(colors))
ax.add_collection(collection)

print(ax)
print(collection.get_array())

plt.ion()
plt.axis('off')
plt.tight_layout()
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
plt.tight_layout()
plt.show()


for i in range(1,sample):
    print(i)
    print("Samples: " + str(sample))
    print("N of CLusters: " +  str(n_clusters))
    print("valLeftClusters: " + str(len((valLeftClusters))))

    
    egoRInfo=ot_cluster.Ego()
    egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2],egoInfoRight[3],egoInfoRight[4], egoInfoRight[5], egoInfoRight[6], egoInfoRight[7], egoInfoRight[8], egoInfoRight[9])
    
    egoInfoLeft=ReadCsv.get_egoLeftInfoCluster(i)
    egoInfoRight=ReadCsv.get_egoRightInfoCluster(i)
    
    
    egoRInfo=ot_cluster.Ego()
    egoRInfo.set_EgoSpeeds(egoInfoRight[0],egoInfoRight[1],egoInfoRight[2],egoInfoRight[3],egoInfoRight[4], egoInfoRight[5], egoInfoRight[6], egoInfoRight[7], egoInfoRight[8], egoInfoRight[9])
    
    egoLInfo=ot_cluster.Ego()
    egoLInfo.set_EgoSpeeds(egoInfoLeft[0],egoInfoLeft[1],egoInfoLeft[2],egoInfoLeft[3],egoInfoLeft[4], egoInfoLeft[5], egoInfoLeft[6], egoInfoLeft[7] ,egoInfoLeft[8] ,egoInfoLeft[9])
    
    egoRInfo.eval_thresholds()
    egoLInfo.eval_thresholds()

    
    
    valLeftClusters=FunctionCreateClusters(egoRInfo, egoLInfo,i)[1]
    valRightClusters=FunctionCreateClusters(egoRInfo, egoLInfo,i)[0]
    
    n_clusters= len(valLeftClusters)
    for j in range(1,max_clusters-1):
        if j < n_clusters:
            patchesL[j].set_x(-valLeftClusters[j].f_DistY)
            patchesL[j].set_y(valLeftClusters[j].f_DistX)
            if valLeftClusters[j].s_ClusterKinematicID == "Dynamic":
                patch_colors.append("red")
            elif valLeftClusters[j].s_ClusterKinematicID == "Static":
                patch_colors.append("red")
            elif valLeftClusters[j].s_ClusterKinematicID == "Ambig":
                patch_colors.append("red")
            elif valLeftClusters[j].s_ClusterKinematicID == "Static-Ambig":
                patch_colors.append("red")
        else:
            patchesL[j].set_x(-100)
            patchesL[j].set_y(-100)
            patch_colors.append("red")
            print(i,j, patchesL[j].get_x(),patchesL[j].get_y())
            
    n_clusters = len(valRightClusters)
    #print('VALID RIGHT CLUSTERS: ' + str(n_clusters))
        
    for j in range(1,max_clusters-1):
        if j < n_clusters:
            patchesR[j].set_x(-valRightClusters[j].f_DistY + radarOffset)
            patchesR[j].set_y(valRightClusters[j].f_DistX)
            if valRightClusters[j].s_ClusterKinematicID == "Dynamic":
                patch_colors.append("red")
            elif valRightClusters[j].s_ClusterKinematicID == "Static":
                patch_colors.append("red")
            elif valRightClusters[j].s_ClusterKinematicID == "Ambig":
                patch_colors.append("red")
            elif valRightClusters[j].s_ClusterKinematicID == "Static-Ambig":
                patch_colors.append("red")
                
            print(i,j, patchesR[j].get_x(),patchesR[j].get_y())
        else:
            patchesR[j].set_x(-100)
            patchesR[j].set_y(-100)
            print(i,j, patchesR[j].get_x(),patchesR[j].get_y())


    addedPatch = patchesL + patchesR
    #colors = np.linspace(0, 1, len(addedPatch))
    #collection = PatchCollection(addedPatch)
    #collection.set_array(np.array(colors)) 
    tcmap = ListedColormap(patch_colors)

    collection = PatchCollection(addedPatch, cmap=tcmap, alpha=0.3)
    ax.add_collection(collection)
    plt.pause(4e-4)
    plt.cla()
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    plt.tight_layout()
    plt.show()


