import glob
import shutil as shutil
import numpy as np
import os as os
import math as math
from MinMax import MinMax
import open3d as o3d
import time
import re
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN

def Cluster(Point, SampleDistance = 2, min_samples=2):
    data = np.asarray(Point)
    # Start Clustering (but not sorted yet in this part)
    model = DBSCAN(eps=SampleDistance, min_samples=min_samples)
    model.fit_predict(data)

    # Prepare the list, prepare the list inside clusterlist.
    ClusterList = [[] for _ in range(len(set(model.labels_)))]

    # In clustering, we maybe will find points which are noise, so if found noise, noise status will become True.
    # The noise points will be grouped in one cluster (-1) and will be removed.
    noise = False
    # print('Total Found ClusterList :', len(ClusterList))

    # Start sorting the data based on index number of clustering [after clustering step]
    for data_no in range(0, len(data)):
        # Check the point belongs to which cluster (cluster 1, cluster 2, cluster 3?)
        clusterIdx = model.labels_[data_no]

        # index = -1 means it is noise point
        if clusterIdx != -1:
            ClusterList[clusterIdx].append(Point[data_no])
    return ClusterList



def ReadXyzFile(filename):
    # print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def Sq2(value):
    return value * value

# # 分割所有凸點-----------------------------------------------------------------------------------------------

shutil.rmtree('/Users/yxc/研究所/Layering/cluster')
os.makedirs('/Users/yxc/研究所/Layering/cluster')

allfile = glob.glob("Extruded Parts Removal/*")
file = glob.glob("Extruded Parts Removal/Cluster*")
for afile in allfile:
    print(afile)

removefile = list(set(allfile) - set(file))
for remove in removefile:
    os.remove(remove)
print('remove finish')

for lastfile in glob.glob("Extruded Parts Removal/*"):
    print(lastfile)
name = input("Please enter your name: ")

# # revise centor error(calibration)
movecnetor_error = np.array([[1, 0, 0, -0.1], [0, 1, 0, -0.2], [0, 0, 1, 0], [0, 0, 0, 1]])

oricentor_gold = o3d.io.read_point_cloud('/Users/yxc/研究所/Extruded Parts Removal/Result/SF_Grid_List.xyz')
oricentor_raw = o3d.io.read_point_cloud('/Users/yxc/研究所/Extruded Parts Removal/Result/RemovedPoints.xyz')
newcentor_gold = oricentor_gold.transform(movecnetor_error)
newcentor_raw = oricentor_raw.transform(movecnetor_error)
o3d.io.write_point_cloud("Source File/SF_Grid_List.xyz", newcentor_gold)
o3d.io.write_point_cloud("Source File/RemovedPoints.xyz", newcentor_raw)

print('\ncentor revised')
file = 'Source File/SF_Grid_List.xyz'
Rfile = 'Source File/RemovedPoints.xyz'
file_PointList = ReadXyzFile(file)
Rfile_PointList = ReadXyzFile(Rfile)

Cfile = Cluster(file_PointList, SampleDistance=2, min_samples=2)  # EdgePoint
print('Total Found SF_no :', len(Cfile))

CRfile = Cluster(Rfile_PointList, SampleDistance=2, min_samples=5)  # EdgePoint
print('Total Found Removed_no :', len(Cfile))

for cfile_cluster_no in range(0, len(Cfile)):
    print('SF_no =', cfile_cluster_no)
    SaveFile('cluster/orefCluster_{}.xyz'.format(cfile_cluster_no), Cfile[cfile_cluster_no])

for Rfile_cluster_no in range(0, len(CRfile)):
    print('Removed_no =', Rfile_cluster_no)
    SaveFile('cluster/orawCluster_{}.xyz'.format(Rfile_cluster_no), CRfile[Rfile_cluster_no])
    SaveFile('Extruded Parts Removal/orawCluster_{}.xyz'.format(Rfile_cluster_no), CRfile[Rfile_cluster_no])


# # 修正凸點座標轉移矩陣順序-----------------------------------------------------------------------------------------------
beforecluster = sorted(glob.glob(os.path.join("Extruded Parts Removal/", "*.xyz")))
print(beforecluster)
beforecentor = []
for before_no in range(0, len(beforecluster)):
    bf = open(beforecluster[before_no], "r")
    blines = bf.readlines()
    # print('No of Points [XYZ]:', len(blines))
    Bfile = []
    for x in range(0, len(blines)):
        RawData = blines[x].strip().split(" ")  # [x y z] from File
        Bfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])
    X = []
    Y = []
    Z = []

    XSum = 0
    YSum = 0
    ZSum = 0
    for PointNo in range(0, len(Bfile)):
        X.append(Bfile[PointNo][0])
        Y.append(Bfile[PointNo][1])
        Z.append(Bfile[PointNo][2])

        XSum = XSum + Bfile[PointNo][0]
        YSum = YSum + Bfile[PointNo][1]
        ZSum = ZSum + Bfile[PointNo][2]

    XMean = XSum / len(Bfile)
    YMean = YSum / len(Bfile)
    ZMean = ZSum / len(Bfile)

    CP = []
    CPnor = []
    CP.append([XMean, YMean, ZMean])
    # print(CP)
    SaveFile('Extruded Parts Removal/beforeCP_{}.xyz'.format(before_no), CP)
    beforecentor.append([XMean, YMean, ZMean])
SaveFile('Extruded Parts Removal/beforecentor.xyz', beforecentor)
# print(beforecentor)

before_part_no = int(len(beforecentor) / 2)
for point_noi in range(0, before_part_no):
    dis = []
    for point_noj in range(0, len(beforecentor)):
        dis_2point = math.sqrt(Sq2(beforecentor[point_noi][0] - beforecentor[point_noj][0]) +
                               Sq2(beforecentor[point_noi][1] - beforecentor[point_noj][1]) +
                               Sq2(beforecentor[point_noi][2] - beforecentor[point_noj][2]))
        dis.append(dis_2point)
    dis_data = np.asarray(dis)
    # print(dis_data)
    mnp = np.min(dis_data, axis=0)
    # print(mnp)
    min = np.sort(dis_data)[1]
    # 第2大索引
    min_index2 = np.argsort(dis_data)[1]
    # print(min_index2 + 1)
    print('\n原cluster =', beforecluster[point_noi], '\n修正編號', point_noi, '->', min_index2 - 4)#!要注意有幾個凸點！！！

    before_tra = np.genfromtxt("/Users/yxc/研究所/Extruded Parts Removal/Result/tra_{}.xyz".format(point_noi), dtype=None, comments='#', delimiter=' ')
    before_tra_inv = np.genfromtxt("/Users/yxc/研究所/Extruded Parts Removal/Result/tra_inv_{}.xyz".format(point_noi), dtype=None, comments='#', delimiter=' ')

    SaveFile("transform/tra_{}.xyz".format(min_index2 - 4), before_tra)  # tra_inv:算完反推原坐標系 #!要注意有幾個凸點！！！
    SaveFile("transform/tra_inv_{}.xyz".format(min_index2 - 4), before_tra_inv)  # tra:原點轉換到凸點原點 #!要注意有幾個凸點！！！

    # time.sleep(0.1)

name = input("Please enter your name: ")

# # 修正凸點編號順序-----------------------------------------------------------------------------------------------
totalcluster = sorted(glob.glob(os.path.join("cluster/", "o*")))
print(totalcluster)
areacentor = []
for total_no in range(0, len(totalcluster)):
    bf = open(totalcluster[total_no], "r")
    blines = bf.readlines()
    # print('No of Points [XYZ]:', len(blines))
    Bfile = []
    for x in range(0, len(blines)):
        RawData = blines[x].strip().split(" ")  # [x y z] from File
        Bfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])
    X = []
    Y = []
    Z = []

    XSum = 0
    YSum = 0
    ZSum = 0
    for PointNo in range(0, len(Bfile)):
        X.append(Bfile[PointNo][0])
        Y.append(Bfile[PointNo][1])
        Z.append(Bfile[PointNo][2])

        XSum = XSum + Bfile[PointNo][0]
        YSum = YSum + Bfile[PointNo][1]
        ZSum = ZSum + Bfile[PointNo][2]

    XMean = XSum / len(Bfile)
    YMean = YSum / len(Bfile)
    ZMean = ZSum / len(Bfile)

    CP = []
    CPnor = []
    CP.append([XMean, YMean, ZMean])
    # print(CP)
    SaveFile('cluster/CP_{}.xyz'.format(total_no), CP)
    areacentor.append([XMean, YMean, ZMean])
SaveFile('cluster/areacentor.xyz', areacentor)
# print(areacentor)

part_no = int(len(areacentor) / 2)
for point_noi in range(0, part_no):
    dis = []
    for point_noj in range(0, len(areacentor)):
        dis_2point = math.sqrt(Sq2(areacentor[point_noi][0] - areacentor[point_noj][0]) +
                               Sq2(areacentor[point_noi][1] - areacentor[point_noj][1]) +
                               Sq2(areacentor[point_noi][2] - areacentor[point_noj][2]))
        dis.append(dis_2point)
    dis_data = np.asarray(dis)
    # print(dis_data)
    mnp = np.min(dis_data, axis=0)
    # print(mnp)
    min = np.sort(dis_data)[1]
    # 第2小索引
    min_index2 = np.argsort(dis_data)[1]
    # print(min_index2 + 1)
    print('rawcluster = ', totalcluster[point_noi], '   refcluster = ', totalcluster[min_index2])
    before_raw = o3d.io.read_point_cloud(totalcluster[point_noi])
    before_ref = o3d.io.read_point_cloud(totalcluster[min_index2])

    after_raw = o3d.io.write_point_cloud("cluster/rawCluster{}.xyz".format(point_noi), before_raw)
    after_ref = o3d.io.write_point_cloud("cluster/refCluster{}.xyz".format(point_noi), before_ref)
    time.sleep(0.1)
    # name = input("Please enter your name: ")
    # print("Name:", name)

