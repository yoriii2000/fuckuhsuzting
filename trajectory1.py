import shutil as shutil
import numpy as np
import os as os
import math as math
from MinMax import MinMax
import open3d as o3d

from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN

import glob
import re

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
    print('Total Found ClusterList :', len(ClusterList))

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


a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluster_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluster_|_|.xyz', x)[2])))
print(a)

protrusion_no = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1

shutil.rmtree('/Users/yxc/研究所/Layering/Process')
shutil.rmtree('/Users/yxc/研究所/Layering/Trajectory')
shutil.rmtree('/Users/yxc/研究所/Layering/twotraj')
shutil.rmtree('/Users/yxc/研究所/Layering/layercluster')
os.makedirs('/Users/yxc/研究所/Layering/layercluster')
os.makedirs('/Users/yxc/研究所/Layering/Process')
os.makedirs('/Users/yxc/研究所/Layering/Trajectory')
os.makedirs('/Users/yxc/研究所/Layering/twotraj')

each_layer = []
for pro_number in range(0, protrusion_no):
    print(pro_number)
    tra = np.genfromtxt("transform/tra_{}.xyz".format(pro_number), dtype=None, comments='#', delimiter=' ')

    each_layer = sorted(glob.glob(os.path.join("each layer/", "Cluster_{}*".format(pro_number))), key=lambda x: (int(re.split('Cluster_|_|.xyz', x)[1]),
                                                                                                                int(re.split('Cluster_|_|.xyz', x)[2])))
    for each_layer_no in range(0, len(each_layer)):
        layer_before = o3d.io.read_point_cloud(each_layer[each_layer_no])
        layer_after = layer_before.transform(tra)
        o3d.io.write_point_cloud("Process/layer_after{}_{}.xyz".format(pro_number, each_layer_no), layer_after)
        layer = "Process/layer_after{}_{}.xyz".format(pro_number, each_layer_no)
        data = ReadXyzFile('{}'.format(layer))

        for PointNo in range(0, len(data)):
            if PointNo == 0:
                XminPCA = data[PointNo][0]
                XmaxPCA = data[PointNo][0]
                YminPCA = data[PointNo][1]
                YmaxPCA = data[PointNo][1]
                ZminPCA = data[PointNo][2]
                ZmaxPCA = data[PointNo][2]

            else:
                if data[PointNo][0] < XminPCA:
                    XminPCA = data[PointNo][0]
                if data[PointNo][0] > XmaxPCA:
                    XmaxPCA = data[PointNo][0]
                if data[PointNo][1] < YminPCA:
                    YminPCA = data[PointNo][1]
                if data[PointNo][1] > YmaxPCA:
                    YmaxPCA = data[PointNo][1]
                if data[PointNo][2] < ZminPCA:
                    ZminPCA = data[PointNo][2]
                if data[PointNo][2] > ZmaxPCA:
                    ZmaxPCA = data[PointNo][2]

        a0 = [XminPCA, YmaxPCA, ZminPCA]
        a1 = [XminPCA, YminPCA, ZminPCA]
        a2 = [XmaxPCA, YminPCA, ZminPCA]
        a3 = [XmaxPCA, YmaxPCA, ZminPCA]

        # corner = [a0, a1, a2, a3]
        # SaveFile('coor/corner{}_{}.xyz'.format(pro_number, each_layer_no), corner)

        v1 = np.subtract(a1, a0)
        d1 = math.sqrt(Sq2(v1[0]) + Sq2(v1[1]) + Sq2(v1[2]))
        v2 = np.subtract(a2, a0)
        d2 = math.sqrt(Sq2(v2[0]) + Sq2(v2[1]) + Sq2(v2[2]))
        v3 = np.subtract(a3, a0)
        d3 = math.sqrt(Sq2(v3[0]) + Sq2(v3[1]) + Sq2(v3[2]))

        f1 = np.subtract(a1, a2)
        f3 = np.subtract(a3, a2)

        d = []
        d.append([d1, d2, d3])
        # print('d =', d)
        dmin = np.min(d, axis=1)

        v = np.vstack((v1, v2, v3))
        # print('v =\n', v)

        # a0、a1
        ever = [0, 1 / 5, 1 / 2, 4 / 5, 0]

        for trajline_no in range(1, 4):
            print('trajline_no = ', trajline_no)
            twopoints = []

            if dmin == d1:
                am = a0 + ever[trajline_no] * v1
                af = a3 + ever[trajline_no] * (-f3)
                # print(am)
            # a0、a3
            elif dmin == d3:
                am = a0 + ever[trajline_no] * v3
                af = a1 + ever[trajline_no] * (-f1)
                # print(am)

            Ftraj = []
            twopoints = np.vstack((am, af))
            # SaveFile('amaf.xyz', twopoints)

            vf = np.subtract(twopoints[1], twopoints[0])
            twodtraj = []
            for twodtraj_no in range(0, 10):  # 點的個數
                ams = twopoints[0] + twodtraj_no * (1 / 9 * vf)  # 間隔數
                ams = ams.tolist()
                twodtraj.append(ams)

            if trajline_no == 1 or trajline_no == 3:
                SaveFile('twotraj/twodtraj{}_{}_{}.xyz'.format(pro_number, each_layer_no, trajline_no), twodtraj)
            else:
                SaveFile('twotraj/twodtraj{}_{}_{}.xyz'.format(pro_number, each_layer_no, trajline_no), twodtraj)

            Ftraj = []
            for j in range(0, len(data)):
                value = np.subtract(twodtraj[0], twodtraj[len(twodtraj) - 1])
                fv = abs(value)
                # print('fv = ', fv)
                if fv[0] > fv[1]:
                    for k in range(0, len(twodtraj)):
                        if twodtraj[k][1] - 1.7 < data[j][1] < twodtraj[k][1] + 1.7:  # 2
                            Ftraj.append(data[j])

                elif fv[1] > fv[0]:
                    for k in range(0, len(twodtraj)):
                        if twodtraj[k][0] - 1.7 < data[j][0] < twodtraj[k][0] + 1.7:  # 2
                            Ftraj.append(data[j])

            Flist = np.array(list(set([tuple(t) for t in Ftraj])))

            if trajline_no == 1 or trajline_no == 3:
                SaveFile('Trajectory/Trajp{}_{}_{}.xyz'.format(pro_number, each_layer_no, trajline_no), Flist)
            else:
                SaveFile('Trajectory/Trajp{}_{}_{}.xyz'.format(pro_number, each_layer_no, trajline_no), Flist)









