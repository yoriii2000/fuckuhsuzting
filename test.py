import os as os
import open3d as o3d
import numpy as np
import math
import transformat as trans
import copy

import glob
from numpy.linalg import inv

import time


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList


def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

# pcd = o3d.io.read_point_cloud("test/metalmodel.xyz")
#
# # 原點轉到圓心
# eTm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -183.85], [0, 0, 0, 1]])  # code 裡單位為毫米mm    # golfmodel metal
# model9 = pcd.transform(eTm)
# o3d.io.write_point_cloud("test/model9.xyz", model9)
#
# target = o3d.io.read_point_cloud("test/icpresult0.xyz")
# source = o3d.io.read_point_cloud("test/result3.xyz")
# threshold = 0.5
# trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix，这是一个转换矩阵，
#                          [0,1,0,0],   # 象征着没有任何位移，没有任何旋转，我们输入
#                          [0,0,1,0],   # 这个矩阵为初始变换
#                          [0,0,0,1]])
# draw_registration_result(source, target, trans_init)
#
# # # fitness越高越好， inlier_rmse越小越好
# reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
#                                                       o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                                       o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 4000))
#
# print(reg_p2p)
# print("Transformation is:")
# # print(reg_p2p.transformation)
#
# tra = reg_p2p.transformation
# draw_registration_result(source, target, reg_p2p.transformation)


# #


s0filename = 's0.xyz'
objpoints = ReadXyzFile('{}'.format(s0filename))

s180filename = 's180.xyz'
ropoints = ReadXyzFile('{}'.format(s180filename))

s0 = []
s180 = []

for objpoints_no in range(0, len(objpoints)):
        s0.append(objpoints[objpoints_no])

for ropoints_no in range(0, len(ropoints)):
        s180.append(ropoints[ropoints_no])

for s0_no in range(0, len(s0)):
    s0[s0_no].append(255)
    s0[s0_no].append(255)
    s0[s0_no].append(255)

for s180_no in range(0, len(s180)):
    s180[s180_no].append(255)
    s180[s180_no].append(255)
    s180[s180_no].append(255)

SaveFile('test/s0.xyz', s0)
SaveFile('test/s180.xyz', s180)

source_0 = o3d.io.read_point_cloud("test/s0.xyz")
source_180 = o3d.io.read_point_cloud('test/s180.xyz')

# RTmatrix = 'cameratransformation.xyz'
source0_no = trans.demo_manual_registration(source_0)
source180_no = trans.demo_manual_registration(source_180)

# print('source_no[0] = ', source_no[0])

#  旋轉法找出旋轉中心
choose0 = [s0[source0_no[0]][0], s0[source0_no[0]][1], s0[source0_no[0]][2]]
choose180 = [s180[source180_no[0]][0], s180[source180_no[0]][1], s180[source180_no[0]][2]]
centor = np.add(choose0, choose180)
centorxyz = [[centor[0]/2, centor[1]/2, centor[2]/2]]

SaveFile('test/centorxyz.xyz', centorxyz)



