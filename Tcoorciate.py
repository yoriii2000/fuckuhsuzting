import os as os
import open3d as o3d
import numpy as np
import math

import glob
from numpy.linalg import inv

import time
import shutil
import re

def search_radius_vector_3d(pcd, pcd_tree, k, i, dis):

    pcd.colors[i] = [0, 0, 1] # 查詢點

    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], dis)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    X = []
    Y = []
    Z = []
    C = []

    XSum = 0
    YSum = 0
    ZSum = 0
    # print('pcd.points[idx[j]] =', pcd.points[idx[0]])

    for j in range(0, len(idx)):
        X = pcd.points[idx[j]][0]
        Y = pcd.points[idx[j]][1]
        Z = pcd.points[idx[j]][2]
        XSum = XSum + pcd.points[idx[j]][0]
        YSum = YSum + pcd.points[idx[j]][1]
        ZSum = ZSum + pcd.points[idx[j]][2]

    XMean = XSum / len(idx)
    YMean = YSum / len(idx)
    ZMean = ZSum / len(idx)

    C.append([XMean, YMean, ZMean])

    return idx, C


def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def ReadXyzNorFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3]), float(RawData[4]),float(RawData[5])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

shutil.rmtree('/Users/yxc/研究所/Layering/coor')
os.makedirs('/Users/yxc/研究所/Layering/coor')

a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)

alltrajcoor = []
protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1

for protruison_no in range(0, protrusion_numaber):
    print('protruison_no = ', protruison_no)

    ba0 = sorted(glob.glob(os.path.join("Trajectory/", "*Trajp{}*".format(protruison_no))), key=lambda x: (int(re.split('Trajp|_|.xyz', x)[2]),
                                                                                                                             int(re.split('Trajp|_|.xyz', x)[3])))
    sa0 = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protruison_no))), key=lambda x: (int(re.split('traj|_|.xyz', x)[2]),
                                                                                                                             int(re.split('traj|_|.xyz', x)[3])))

    shutil.rmtree('/Users/yxc/研究所/Layering/grindcoor{}'.format(protruison_no))
    os.makedirs('/Users/yxc/研究所/Layering/grindcoor{}'.format(protruison_no))
    layer = int(len(ba0) / 3)
    print('該凸點總層數 = ', layer)

    #
    linenor = []
    linenor_1 = []
    linenor_2 = []
    linenor_3 = []
    for linenor_no in range(1, 4):
        print('line = ', linenor_no)
        linefile = 'Output File/traj{}_{}_{}.xyz'.format(protruison_no, layer - 1, linenor_no)
        print(linefile)
        lf = open(linefile, "r")

        trajectory = lf.readlines()

        pointfile = []
        for x in range(0, len(trajectory)):
            RawData = trajectory[x].strip().split(" ")  # [x y z] from File
            pointfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

        norfile = 'Trajectory/Trajp{}_{}_{}.xyz'.format(protruison_no, layer - 1, linenor_no)

        pcd = o3d.io.read_point_cloud(norfile)
        tra_inv = np.genfromtxt("transform/tra_inv_{}.xyz".format(protruison_no), dtype=None, comments='#',
                                delimiter=' ')
        pcd = pcd.transform(tra_inv)

        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=2.5))

        # if protruison_no == 1:
        #     o3d.visualization.draw_geometries([pcd], "Normal points", width=800, height=600, left=50, top=50,
        #                                        point_show_normal=True, mesh_show_wireframe=False, mesh_show_back_face=False)

        pcd1 = pcd

        k = 5  # 20
        dis = 30  # 50 new28

        samplepointsNormal = []
        pcd1.paint_uniform_color([1, 0, 0])

        for i in range(0, len(pcd1.points)):  # pcd1 480 points
            pcd_tree = o3d.geometry.KDTreeFlann(pcd1)
            # knn, C = search_knn_vector_3d(pcd1, pcd_tree, k, i)
            knn, C = search_radius_vector_3d(pcd1, pcd_tree, k, i, dis)

            p = pcd1.points[i]
            v = pcd1.points[i] - C[0]
            n = pcd1.normals[i]

            d_v = math.sqrt(Sq2(v[0]) + Sq2(v[1]) + Sq2(v[2]))
            d_n = math.sqrt(Sq2(n[0]) + Sq2(n[1]) + Sq2(n[2]))

            angle = (math.acos(np.dot(v, n) / d_v * d_n))
            angle = (angle * 180) / math.pi
            # print('angle = ', angle)
            if angle > 89.7:
                pcd1.normals[i][0] = -pcd1.normals[i][0]
                pcd1.normals[i][1] = -pcd1.normals[i][1]
                pcd1.normals[i][2] = -pcd1.normals[i][2]

            # pcd2 = pcd1.voxel_down_sample(voxel_size=2.5)
            # o3d.io.write_point_cloud('nor.xyz', pcd1)
            p = p.tolist()

            samplepointsNormal.append(p)
            samplepointsNormal[i].append(n[0])
            samplepointsNormal[i].append(n[1])
            samplepointsNormal[i].append(n[2])

        # if protruison_no == 1:
        #     o3d.visualization.draw_geometries([pcd1], "change the normal way", width=800, height=600, left=50, top=50,
        #                                       point_show_normal=True, mesh_show_wireframe=False,
        #                                       mesh_show_back_face=False)

        SamplePoint = samplepointsNormal
        Point = samplepointsNormal

        SMpoint = pointfile
        gridnor = []
        for SMpoint_no in range(0, len(SMpoint)):
            # print(SMpoint_no)
            dx = []
            for PointNo in range(0, len(SamplePoint)):  # len(SamplePoint)

                dx.append(math.sqrt(Sq2((SMpoint[SMpoint_no][0] - SamplePoint[PointNo][0]))
                                    + Sq2((SMpoint[SMpoint_no][1] - SamplePoint[PointNo][1]))
                                    + Sq2((SMpoint[SMpoint_no][2] - SamplePoint[PointNo][2]))))

            mn = np.min(dx)
            # print(mn)
            for i in range(0, len(dx)):
                if dx[i] == mn:
                    linenor.append([SamplePoint[i][3], SamplePoint[i][4], SamplePoint[i][5]])

        # print('linenor = ', linenor)
        # print('linenor = ', len(linenor))

    linenor_1 = linenor[0:10]
    linenor_2 = linenor[10:20]
    linenor_3 = linenor[20:30]
    # print('linenor_1 = ', linenor_1)
    # print('linenor_2 = ', linenor_2)
    # print('linenor_3 = ', linenor_3)

    print("next protrusion\n")
    #


    for Bfile0_no in range(0, layer):

        # ---------------------------------------------------------------------------------------------------------
        print('layer = ', Bfile0_no)
        normalline = sorted(glob.glob(os.path.join("Trajectory/", "*Trajp{}_{}_*".format(protruison_no, Bfile0_no))), key=os.path.getmtime)
        layerline = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}_*".format(protruison_no, Bfile0_no))), key=os.path.getmtime)

        for layerline_no in range(0, len(layerline)):
            # print(ba0[Bfile0_no])
            # print(sa0[Bfile0_no])
            sf = open(layerline[layerline_no], "r")
            print('------------------------------------------', layerline)
            print('layerline_no = ', layerline_no + 1)
            slines = sf.readlines()
            # print('No of Points [XYZ]:', len(slines))
            Sfile = []
            for x in range(0, len(slines)):
                RawData = slines[x].strip().split(" ")  # [x y z] from File
                Sfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

            gridpointnor = Sfile
            grindoint = np.array(Sfile)

            for PointNo in range(0, len(gridpointnor)):
                # print(PointNo)
                xp = []
                yp = []
                zp = []

                if layerline_no + 1 == 1:
                    xnor = linenor_1[PointNo][0]
                    ynor = linenor_1[PointNo][1]
                    znor = linenor_1[PointNo][2]
                elif layerline_no + 1 == 2:
                    xnor = linenor_2[PointNo][0]
                    ynor = linenor_2[PointNo][1]
                    znor = linenor_2[PointNo][2]
                elif layerline_no + 1 == 3:
                    xnor = linenor_3[PointNo][0]
                    ynor = linenor_3[PointNo][1]
                    znor = linenor_3[PointNo][2]

                if PointNo == len(gridpointnor) - 1:  # 最後一個點

                    xv = np.subtract(grindoint[PointNo - 1], grindoint[PointNo])
                    xp = -xv
                    # dx = math.sqrt(Sq2(xv[0]) + Sq2(xv[1]) + Sq2(xv[2]))
                    # xp = xv / dx

                elif PointNo == 0:
                    xp = np.subtract(grindoint[PointNo + 2], grindoint[PointNo])

                else:
                    xp = np.subtract(grindoint[PointNo + 1], grindoint[PointNo])
                    # print('xv =', xv)

                zp.append([xnor, ynor, znor])
                zp = np.asarray(zp)

                # 軌跡點坐標系

                yp = np.cross(zp, xp)
                xp = np.cross(yp, zp)
                # zp = np.cross(xp, yp)


                xp = xp.flatten()
                yp = yp.flatten()
                zp = zp.flatten()

                dx = math.sqrt(Sq2(xp[0]) + Sq2(xp[1]) + Sq2(xp[2]))
                xp = xp / dx
                dy = math.sqrt(Sq2(yp[0]) + Sq2(yp[1]) + Sq2(yp[2]))
                yp = yp / dy
                dz = math.sqrt(Sq2(zp[0]) + Sq2(zp[1]) + Sq2(zp[2]))
                zp = zp / dz

                xyz_vector0 = np.vstack([xp, yp, zp])
                # print('xp = ', xp)
                # print('xyz_vector0 = \n', xyz_vector0)

                xyz_vector = xyz_vector0.T
                # print('xyz_vector = \n', xyz_vector)
                cTt = np.c_[xyz_vector, grindoint[PointNo]]

                e = [[0, 0, 0, 1]]
                cTt = np.r_[cTt, e]

                tTc = inv(cTt)

                if layerline_no == 1:
                    SaveFile('grindcoor{}/cTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, layerline_no + 1, (len(gridpointnor) - 1) - PointNo), cTt)
                else:
                    SaveFile('grindcoor{}/cTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, layerline_no + 1, PointNo), cTt)


                xb = grindoint[PointNo] + xp
                yb = grindoint[PointNo] + yp
                zb = grindoint[PointNo] + zp

                alltrajcoor.append(xb.tolist())
                alltrajcoor.append(yb.tolist())
                alltrajcoor.append(zb.tolist())
                alltrajcoor.append(grindoint[PointNo].tolist())

    SaveFile('coor/alltrajcoor{}.xyz'.format(protruison_no), alltrajcoor)








