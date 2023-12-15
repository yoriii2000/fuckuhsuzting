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

# 手臂校正取點
oripoint = [0, 0, 0]
Uni = 1
Uox = [0 + Uni, 0, 0]
Uoy = [0, 0 + Uni, 0]
Uoz = [0, 0, 0 + Uni]
Oxyz = np.vstack([oripoint, Uox, Uoy, Uoz])
Oxyz = Oxyz.tolist()

# print(Oxyz)

# 新校正版
# chosepointo = [55, 65, 63.54]
# chosepointx = [55, -65, 63.54]
# chosepointy = [-55, 65, 63.54]

# 校正塊
chosepointo = [40, 40, 244.1 + 36.1]
chosepointx = [40, -40, 244.1 + 36.1]
chosepointy = [-40, 40, 244.1 + 36.1]

vx = np.subtract(chosepointx, chosepointo)
vy = np.subtract(chosepointy, chosepointo)
vz = np.cross(vx, vy)

dvx = math.sqrt(Sq2(vx[0]) + Sq2(vx[1]) + Sq2(vx[2]))
vx = vx / dvx
dvy = math.sqrt(Sq2(vy[0]) + Sq2(vy[1]) + Sq2(vy[2]))
vy = vy / dvy
dvz = math.sqrt(Sq2(vz[0]) + Sq2(vz[1]) + Sq2(vz[2]))
vz = vz / dvz

choose_vector0 = np.vstack([vx, vy, vz])
print('4654645498449849498494984', choose_vector0)
choose_vector = choose_vector0.T
eTcos = np.c_[choose_vector, chosepointo]
b = [[0, 0, 0, 1]]
eTcos = np.r_[eTcos, b]
print(eTcos)

SaveFile('test/eTcos.xyz', eTcos)

# 掃描校正取點
filename = 'cube_top.xyz'
berfoe_source = o3d.io.read_point_cloud("cube_top.xyz")
objpoints = ReadXyzFile('{}'.format(filename))

basepoint = []
basepoint_co = []

before_source_no = trans.demo_manual_registration(berfoe_source)
# readfile = 'ScanResult_01.xyz'

# f = open(readfile, "r")
# lines = f.readlines()
# print('no = ', pick_point_no[0])
#
# PointList = []
# RawData = lines[pick_point_no[0]].strip().split()  # [x y z] from File
# PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])
# print('xyz = ', PointList)
# input()

for objpoints_no in range(0, len(objpoints)):
    if (objpoints[objpoints_no][0] > objpoints[before_source_no[0]][0] and objpoints[objpoints_no][1] > objpoints[before_source_no[0]][1]):
        basepoint.append(objpoints[objpoints_no])
        basepoint_co.append(objpoints[objpoints_no])

SaveFile('test/basepoint.xyz', basepoint)
# SaveFile('test/basepoint_co.xyz', basepoint_co)

for basepoint_co_no in range(0, len(basepoint_co)):
    basepoint_co[basepoint_co_no].append(255)
    basepoint_co[basepoint_co_no].append(255)
    basepoint_co[basepoint_co_no].append(255)

SaveFile('test/basepoint_co.xyz', basepoint_co)

source = o3d.io.read_point_cloud("test/basepoint_co.xyz")
source_no = trans.demo_manual_registration(source)

# # 校正姿態
# topCchoseo = [basepoint[source_no[0]][0] - 32, basepoint[source_no[0]][1] - 7, basepoint[source_no[0]][2] - 200]    # glofmodel
# topCchoseo = [basepoint[source_no[0]][0] - 32, basepoint[source_no[0]][1] - 7, basepoint[source_no[0]][2] - 190]    # model 8
topCchoseo = [basepoint[source_no[0]][0], basepoint[source_no[0]][1], basepoint[source_no[0]][2]]    # 校正塊 glofmodel

# 校正塊選點
xp = [basepoint[source_no[1]][0] - basepoint[source_no[0]][0], basepoint[source_no[1]][1] - basepoint[source_no[0]][1], basepoint[source_no[1]][2] - basepoint[source_no[0]][2]]
yp = [basepoint[source_no[2]][0] - basepoint[source_no[0]][0], basepoint[source_no[2]][1] - basepoint[source_no[0]][1], basepoint[source_no[2]][2] - basepoint[source_no[0]][2]]

xp = np.array(xp)
yp = np.array(yp)
zp = np.cross(xp, yp)

dxp = math.sqrt(Sq2(xp[0]) + Sq2(xp[1]) + Sq2(xp[2]))
xp = xp / dxp
dyp = math.sqrt(Sq2(yp[0]) + Sq2(yp[1]) + Sq2(yp[2]))
yp = yp / dyp
dzp = math.sqrt(Sq2(zp[0]) + Sq2(zp[1]) + Sq2(zp[2]))
zp = zp / dzp

Cpvector0 = np.vstack([xp, yp, zp])
Cpvector = Cpvector0.T
CTp = np.c_[Cpvector, topCchoseo]
b = [[0, 0, 0, 1]]
CTcosp = np.r_[CTp, b]
print('CTp = ', CTcosp)
SaveFile('test/CTcosp.xyz', CTcosp)

cospTC = np.linalg.inv(CTcosp)
SaveFile('test/cosTC.xyz', cospTC)

# 原點轉移到選點
pointbefore = o3d.io.read_point_cloud('test/basepoint.xyz')
pointafter = pointbefore.transform(cospTC)
o3d.io.write_point_cloud("test/basepointafter.xyz", pointafter)

eTc = np.dot(eTcos, cospTC)
SaveFile('test/top_eTc.xyz', eTc)

# 原點轉移到法蘭
# pointbefore = o3d.io.read_point_cloud('test/basepoint.xyz')
# pointafter = pointbefore.transform(eTc)
# o3d.io.write_point_cloud("test/basepointafter.xyz", pointafter)

# 原點轉到法蘭
print('scanresult_start')
scanresult = o3d.io.read_point_cloud("ScanFile/topscan.xyz")
print('scanresult_middle')
transresult = scanresult.transform(eTc)
o3d.io.write_point_cloud("test/topframeresult.xyz", transresult)
print('scanresult_end')

# # 找出旋轉中心
# centorpcd_0 = o3d.io.read_point_cloud("s0.xyz")
# transresult_0 = centorpcd_0.transform(eTc)
# o3d.io.write_point_cloud("test/s0.xyz", transresult_0)
#
# centorpcd_180 = o3d.io.read_point_cloud("s180.xyz")
# transresult_180 = centorpcd_180.transform(eTc)
# o3d.io.write_point_cloud("test/s180.xyz", transresult_180)
#
# s0filename = "test/s0.xyz"
# objpoints = ReadXyzFile('{}'.format(s0filename))
#
# s180filename = "test/s180.xyz"
# ropoints = ReadXyzFile('{}'.format(s180filename))
#
# s0 = []
# s180 = []
#
# for objpoints_no in range(0, len(objpoints)):
#         s0.append(objpoints[objpoints_no])
#
# for ropoints_no in range(0, len(ropoints)):
#         s180.append(ropoints[ropoints_no])
#
# for s0_no in range(0, len(s0)):
#     s0[s0_no].append(255)
#     s0[s0_no].append(255)
#     s0[s0_no].append(255)
#
# for s180_no in range(0, len(s180)):
#     s180[s180_no].append(255)
#     s180[s180_no].append(255)
#     s180[s180_no].append(255)
#
# SaveFile('test/s0.xyz', s0)
# SaveFile('test/s180.xyz', s180)
#
# source_0 = o3d.io.read_point_cloud("test/s0.xyz")
# source_180 = o3d.io.read_point_cloud('test/s180.xyz')
#
# # RTmatrix = 'cameratransformation.xyz'
# source0_no = trans.demo_manual_registration(source_0)
# print('type = ', source0_no)
# print('type = ', type(source0_no))
#
# source180_no = trans.demo_manual_registration(source_180)
#
# # print('source_no[0] = ', source_no[0])
#
# #  旋轉法找出旋轉中心
# choose0 = [s0[source0_no[0]][0], s0[source0_no[0]][1], s0[source0_no[0]][2]]
# choose180 = [s180[source180_no[0]][0], s180[source180_no[0]][1], s180[source180_no[0]][2]]
# centor = np.add(choose0, choose180)
# centorxyz = [[centor[0]/2, centor[1]/2, 0]]
#
# SaveFile('test/centorxyz.xyz', centorxyz)
#
# # # 中心點校正
# Tmatric = np.array([[1, 0, 0, centor[0]/2], [0, 1, 0, centor[1]/2], [0, 0, 1, 0], [0, 0, 0, 1]])
# print(Tmatric)
#
# realcentor_top = transresult.transform(Tmatric)
# o3d.io.write_point_cloud("test/realcentor_top.xyz", realcentor_top)
# 結合----------------------------------------------------------------------

# target = transresult
# source = o3d.io.read_point_cloud("test/icpresult1_8.xyz")
# threshold = 0.3
# trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix，这是一个转换矩阵，
#                          [0,1,0,0],   # 象征着没有任何位移，没有任何旋转，我们输入
#                          [0,0,1,0],   # 这个矩阵为初始变换
#                          [0,0,0,1]])
# draw_registration_result(source, target, trans_init)
#
# # # fitness越高越好， inlier_rmse越小越好
# reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
#                                                       o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                                       o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 3000))
#
# trans = reg_p2p.transformation
# draw_registration_result(source, target, reg_p2p.transformation)
# print(reg_p2p)
#
# metal9 = source.transform(trans)
# modelcombine = metal9 + transresult
# o3d.io.write_point_cloud("test/modelcombine.xyz", modelcombine)
#
# # 濾波
# num_points = 5
# radius = 0.8
# Model, ind = modelcombine.remove_radius_outlier(num_points, radius)
# o3d.io.write_point_cloud("test/rmodel.xyz", Model)
#
# # 點雲降點
# Fmodel = Model.voxel_down_sample(voxel_size=0.7)
# o3d.io.write_point_cloud("test/Fmodel.xyz", Fmodel)
#
# # 原點轉到圓心
# eTm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -183.85], [0, 0, 0, 1]])  # code 裡單位為毫米mm    # golfmodel metal
# # eTm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -178.95], [0, 0, 0, 1]])  # code 裡單位為毫米mm     # model5、8
# # eTm = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -171.57], [0, 0, 0, 1]])  # code 裡單位為毫米mm     # 校正塊
#
# model9 = Fmodel.transform(eTm)
# o3d.io.write_point_cloud("test/model9.xyz", model9)
#
# # # icp with cad file
# target = o3d.io.read_point_cloud("cad_metal9.xyz")
# source = model9
# threshold = 0.25
# trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix，这是一个转换矩阵，
#                          [0,1,0,0],   # 象征着没有任何位移，没有任何旋转，我们输入
#                          [0,0,1,0],   # 这个矩阵为初始变换
#                          [0,0,0,1]])
# draw_registration_result(source, target, trans_init)
#
# # # fitness越高越好， inlier_rmse越小越好
# reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
#                                                       o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                                       o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 3000))
#
# cad_trans = reg_p2p.transformation
# draw_registration_result(source, target, reg_p2p.transformation)
# print(reg_p2p)
#
# toolmodel9 = source.transform(cad_trans)
# o3d.io.write_point_cloud("test/toolmodel9.xyz", toolmodel9)