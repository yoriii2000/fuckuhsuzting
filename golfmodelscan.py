import os as os
import open3d as o3d
import numpy as np
import math
import transformat as trans
import copy
import glob
from numpy.linalg import inv
import time

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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

# # STEP0: 手臂姿態校正---------------------------------------------------------------------------------------------------

# # 手臂校正取點-------------------------------------------------------------------------------------------

oripoint = [0, 0, 0]
Uni = 1
Uox = [0 + Uni, 0, 0]
Uoy = [0, 0 + Uni, 0]
Uoz = [0, 0, 0 + Uni]
Oxyz = np.vstack([oripoint, Uox, Uoy, Uoz])
Oxyz = Oxyz.tolist()

# 校正塊
chosepointo = [40, 40, 163.83 + 36.1]
chosepointx = [40, -40, 163.83 + 36.1]
chosepointy = [40, 40, 244.1 + 36.1]

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
choose_vector = choose_vector0.T
eTcos = np.c_[choose_vector, chosepointo]
b = [[0, 0, 0, 1]]
eTcos = np.r_[eTcos, b]
print(eTcos)

SaveFile('test/eTcos.xyz', eTcos)

# 掃描校正取點--------------------------------------------------------------------------------

filename = 'cube_side.xyz'
objpoints = ReadXyzFile('{}'.format(filename))
  
basepoint = []
basepoint_co = []

for objpoints_no in range(0, len(objpoints)):
    # if (max[2] -2 < objpoints[objpoints_no][2] < max[2] + 2):
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
# RTmatrix = 'cameratransformation.xyz'
source_no = trans.demo_manual_registration(source)

# print('source_no[0] = ', source_no[0])


# 補償校正姿態
# Cchoseo = [basepoint[source_no[0]][0], basepoint[source_no[0]][1], basepoint[source_no[0]][2]]

# Cchoseo = [basepoint[source_no[0]][0] + 28, basepoint[source_no[0]][1] - 207, basepoint[source_no[0]][2] - 20]    # 用心校正版補償  glofmodel  207
# Cchoseo = [basepoint[source_no[0]][0] + 28, basepoint[source_no[0]][1] - 200, basepoint[source_no[0]][2] - 20]   # 用心校正版補償  model 8
Cchoseo = [basepoint[source_no[0]][0], basepoint[source_no[0]][1] - 50, basepoint[source_no[0]][2]]   # 用校正塊補償  glofmodel

# 校正塊座標
xp = [basepoint[source_no[1]][0] - basepoint[source_no[0]][0],
      basepoint[source_no[1]][1] - basepoint[source_no[0]][1],
      basepoint[source_no[1]][2] - basepoint[source_no[0]][2]]

yp = [basepoint[source_no[2]][0] - basepoint[source_no[0]][0],
      basepoint[source_no[2]][1] - basepoint[source_no[0]][1],
      basepoint[source_no[2]][2] - basepoint[source_no[0]][2]]

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
CTp = np.c_[Cpvector, Cchoseo]
b = [[0, 0, 0, 1]]
CTcosp = np.r_[CTp, b]
# print('CTp = ', CTcosp)
SaveFile('test/CTcosp.xyz', CTcosp)

cospTC = np.linalg.inv(CTcosp)
SaveFile('test/cosTC.xyz', cospTC)

# 原點轉移到選點
# pointbefore = o3d.io.read_point_cloud('test/basepoint.xyz')
# pointafter = pointbefore.transform(cospTC)
# o3d.io.write_point_cloud("test/basepointafter.xyz.xyz", pointafter)

# 相機相對於法蘭
eTc = np.dot(eTcos, cospTC)

# 原點轉移到法蘭
pointbefore = o3d.io.read_point_cloud('test/basepoint.xyz')
pointafter = pointbefore.transform(eTc)
o3d.io.write_point_cloud("test/basepointafter.xyz", pointafter)


# # STEP1: 掃描初步拼接---------------------------------------------------------------------------------------------------

scan = sorted(glob.glob(os.path.join("ScanFile/", "ScanResult*")), key=os.path.getmtime)
print('scanfile number = ', len(scan))
rotateangle = 360 / len(scan)
print('rotateangle = ', rotateangle)

# 補償偏移矩陣
# 無校正姿態
# tr = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# #  初步校正塊手動補償位移
tr = np.array([[1, 0, 0, 0.22], [0, 1, 0, 0.37], [0, 0, 1, 0], [0, 0, 0, 1]])

ALLroMatrix = []
for angle in range(0, 360, int(rotateangle)):
    # print(angle)
    degree = -angle * (np.pi / 180)
    rotateM = np.array([[np.cos(degree), -np.sin(degree), 0.0, 0.0],
                        [np.sin(degree), np.cos(degree), 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]
                        ])
    # print('rotateM = ', rotateM)
    rotateM = rotateM.tolist()

    ALLroMatrix.append(rotateM)
# print('ALLroMatrix = ', len(ALLroMatrix))

# 轉移
assresult = o3d.geometry.PointCloud()
for scanfile_no in range(0, len(scan)):
    scanresult = o3d.io.read_point_cloud(scan[scanfile_no])
    # 原點轉到法蘭
    transresult = scanresult.transform(eTc)
    o3d.io.write_point_cloud("test/transresult{}.xyz".format(scanfile_no + 1), transresult)

    # 補償偏移
    aftermove = transresult.transform(tr)

    # 旋轉
    finallpos = aftermove.transform(ALLroMatrix[scanfile_no])
    if (scanfile_no + 1) == 1:
        o3d.io.write_point_cloud("test/result1.xyz", finallpos)
        o3d.io.write_point_cloud("test/icpresult1.xyz", finallpos)
    elif (scanfile_no + 1) == 2:
        o3d.io.write_point_cloud("test/result2.xyz", finallpos)
        # o3d.io.write_point_cloud("test/icpresult2.xyz", finallpos)
    else:
        o3d.io.write_point_cloud("test/result{}.xyz".format(scanfile_no + 1), finallpos)

    assresult += finallpos

o3d.io.write_point_cloud("test/assresult.xyz", assresult)

# STEP2: ICP 拼接------------------------------------------------------------------------------------------------------

resultfile = sorted(glob.glob(os.path.join("test/", "result*")))
new_file1 = []
new_file2 = []

for resultfile_no in range(0, len(resultfile)):
    if resultfile_no % 2 == 1:
        new_file1.append(resultfile[resultfile_no])
    else:
        new_file2.append(resultfile[resultfile_no])
# print(new_file1)
# print(new_file2)
newarray = resultfile
# new_file2.reverse()
# newarray = new_file1 + new_file2
print(newarray)
input("Check combine file : ")

# # 點雲累加
# for newarray_no in range(0, len(newarray)):
#
#     # # 以topscan為基準拼接
#     # # 特徵較多的面先開始
#
#     if newarray_no == 0:
#
#         target_top = o3d.io.read_point_cloud("test/topframeresult.xyz")
#
#         source = o3d.io.read_point_cloud("test/icpresult1.xyz")
#         # source = o3d.io.read_point_cloud("test/icpresult2.xyz")
#
#         threshold = 0.4
#         trans_init = np.asarray([[1, 0, 0, 0],  # 4x4 identity matrix，这是一个转换矩阵，
#                                  [0, 1, 0, 0],  # 象征着没有任何位移，没有任何旋转，我们输入
#                                  [0, 0, 1, 0],  # 这个矩阵为初始变换
#                                  [0, 0, 0, 1]])
#         draw_registration_result(source, target_top, trans_init)
#
#         # # fitness越高越好， inlier_rmse越小越好
#         reg_p2p = o3d.pipelines.registration.registration_icp(source, target_top, threshold, trans_init,
#                                                               o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                                               o3d.pipelines.registration.ICPConvergenceCriteria(
#                                                                   max_iteration=4000))
#
#         print(reg_p2p)
#         print("Transformation is:")
#         # print(reg_p2p.transformation)
#
#         tra = reg_p2p.transformation
#         draw_registration_result(source, target_top, reg_p2p.transformation)
#
#         com_top = source.transform(tra)
#         o3d.io.write_point_cloud("test/icpresult1.xyz", com_top)
#
#         com_top = com_top + target_top
#         o3d.io.write_point_cloud("test/icpresult0.xyz", com_top)
#
#     else:
#         target = o3d.io.read_point_cloud("test/icpresult0.xyz")
#         source = o3d.io.read_point_cloud(newarray[newarray_no])
#         print('newarray ', newarray[newarray_no])
#
#         threshold = 0.15
#         trans_init = np.asarray([[1, 0, 0, 0],
#                                  [0, 1, 0, 0],
#                                  [0, 0, 1, 0],
#                                  [0, 0, 0, 1]])
#         draw_registration_result(source, target, trans_init)
#
#         reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
#                                                               o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                                               o3d.pipelines.registration.ICPConvergenceCriteria(
#                                                                   max_iteration=3000))
#
#         trans = reg_p2p.transformation
#         draw_registration_result(source, target, reg_p2p.transformation)
#         print(reg_p2p)
#
#         icpresult = source.transform(trans)
#         o3d.io.write_point_cloud("test/icpresult{}.xyz".format(newarray_no + 1), icpresult)
#
#         combine_result = target + icpresult
#         if newarray_no + 1 == len(newarray):
#             o3d.io.write_point_cloud("test/icpresult1_{}.xyz".format(len(scan)), combine_result)
#
#             # 濾波
#             num_points = 5
#             radius = 0.8
#             Model, ind = combine_result.remove_radius_outlier(num_points, radius)
#             o3d.io.write_point_cloud("test/rpcd.xyz", Model)
#         else:
#             o3d.io.write_point_cloud("test/icpresult0.xyz", combine_result)

# # 點雲分開拼接
combine_result = o3d.geometry.PointCloud()
for newarray_no in range(0, len(newarray)):

    # # 以topscan為基準拼接
    # # 特徵較多的面先開始

    target_top = o3d.io.read_point_cloud("test/topframeresult.xyz")
    source = o3d.io.read_point_cloud(newarray[newarray_no])

    print('newarray ', newarray[newarray_no])

    threshold = 0.25
    trans_init = np.asarray([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    draw_registration_result(source, target_top, trans_init)

    reg_p2p = o3d.pipelines.registration.registration_icp(source, target_top, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                          o3d.pipelines.registration.ICPConvergenceCriteria(
                                                              max_iteration=3000))

    trans = reg_p2p.transformation
    draw_registration_result(source, target_top, reg_p2p.transformation)
    print(reg_p2p)

    icpresult = source.transform(trans)
    o3d.io.write_point_cloud("test/icpresult{}.xyz".format(newarray_no + 1), icpresult)
    combine_result += icpresult

top = o3d.io.read_point_cloud("test/topframeresult.xyz")
combine_result = combine_result + top
o3d.io.write_point_cloud("test/icpresult1_6.xyz", combine_result)

# 濾波
num_points = 5
radius = 0.8
Model, ind = combine_result.remove_radius_outlier(num_points, radius)
o3d.io.write_point_cloud("test/rpcd.xyz", Model)


# # 點雲降點
rpcd = o3d.io.read_point_cloud("test/rpcd.xyz")
metalmodel = rpcd.voxel_down_sample(voxel_size=0.5)
o3d.io.write_point_cloud("test/metalmodel.xyz", metalmodel)

movematric = np.array([[1, 0, 0, -0.4], [0, 1, 0, -0.25], [0, 0, 1, 0], [0, 0, 0, 1]])
# movematric = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

bmetalmodel = o3d.io.read_point_cloud("test/metalmodel.xyz")
metalmodelcentor = bmetalmodel.transform(movematric)
o3d.io.write_point_cloud("test/metalmodelcentor.xyz", metalmodelcentor)

