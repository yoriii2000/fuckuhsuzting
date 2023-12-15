import os
import open3d as o3d
import numpy as np
import math
import File

# def search_knn_vector_3d(pcd, pcd_tree, k, i):
def search_radius_vector_3d(pcd, pcd_tree, k, i, dis):

    pcd.colors[i] = [0, 0, 1] # 查詢點
    # [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], k)
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], dis)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
    # print(pcd.points[idx[0]])

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
    # print('pcd.points[idx[j][0]] =', pcd.points[idx[0]])

    return idx, C

def Sq2(value):
    return value * value

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

# file = 'Source File/SF_Grid_List_9.xyz'
no=1
print('cluster_no = ', no)
Pcd = File.ReadXyzFile('cluster/rawCluster{}.xyz'.format(no))
for PcdNo in range(0, len(Pcd)):
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)

# File.SaveFile('Source File/RemovedPoints_9', Pcd)
File.SaveFile('cluster/rawCluster{}'.format(no), Pcd)
print('this cluster have 0 normal vector\n')

file = 'cluster/refCluster{}.xyz'.format(no)

pcd = o3d.io.read_point_cloud(file)
    # pcd = opcd.voxel_down_sample(voxel_size=1.5)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=4))

# o3d.visualization.draw_geometries([pcd], "Normal points", width=800, height=600, left=50, top=50,
#                                   point_show_normal=True, mesh_show_wireframe=False, mesh_show_back_face=False)
# pcd1 = np.asarray(pcd.points).reshape((-1, 3))
# pcd1 = np.asarray(pcd.normals)[:, :]
pcd1 = pcd

k = 5
dis = 20
i = 1

samplepointsNormal = []
pcd1.paint_uniform_color([1, 0, 0])
# print('pcd1 =', len(pcd1.points))
# o3d.visualization.draw_geometries([pcd], "kdtree points", width=800, height=600, left=50, top=50,
#                                   point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False)
for i in range(0, len(pcd1.points)):  # pcd1 480 points
    pcd_tree = o3d.geometry.KDTreeFlann(pcd1)
    # knn, C = search_knn_vector_3d(pcd1, pcd_tree, k, i)
    knn, C = search_radius_vector_3d(pcd1, pcd_tree, k, i, dis)

    # o3d.visualization.draw_geometries([pcd1], "kdtree points", width=800, height=600, left=50, top=50,
    #                                   point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False)

    # print('i = ', i)
    # print('i = ', pcd1.points[i])
    # print('knn = ', len(knn))

    # print('knn_number = ', knn_number)
    # print(pcd1.points[knn[knn_number]])
    p = pcd1.points[i]
    v = pcd1.points[i] - C[0]
    n = pcd1.normals[i]

    d_v = math.sqrt(Sq2(v[0]) + Sq2(v[1]) + Sq2(v[2]))
    d_n = math.sqrt(Sq2(n[0]) + Sq2(n[1]) + Sq2(n[2]))

    # print('v = ', v)
    # print('n = ', n)
    angle = (math.acos(np.dot(v, n) / d_v * d_n))
    angle = (angle * 180) / math.pi
    # print('angle = ', angle)
    if angle > 88:
        pcd1.normals[i][0] = pcd1.normals[i][0]
        pcd1.normals[i][1] = pcd1.normals[i][1]
        pcd1.normals[i][2] = pcd1.normals[i][2]
    # print('n = ',n)
    # print('pcd1.points[i] = ',pcd1.points[i])

    p = p.tolist()
    samplepointsNormal.append(p)
    samplepointsNormal[i].append(n[0])
    samplepointsNormal[i].append(n[1])
    samplepointsNormal[i].append(n[2])

    # pcd1.points[i] = pcd1.points[i].tolist()
    # pcd1.points[i].append(pcd1.normals[i][0])
    # pcd1.points[i].append(pcd1.normals[i][1])
    # pcd1.points[i].append(pcd1.normals[i][2])
    # print('pcd1.points =',pcd1.points[i])
# pcd1.paint_uniform_color([1, 0, 0])

# o3d.visualization.draw_geometries([pcd1], "change the normal way", width=800, height=600, left=50, top=50,
#                                   point_show_normal=True, mesh_show_wireframe=False, mesh_show_back_face=False)
# print('samplepointsNormal =', samplepointsNormal)

# SaveFile('Source File/SF_Grid_List_9.xyz', samplepointsNormal)
SaveFile('cluster/refCluster{}.xyz'.format(no), samplepointsNormal)
print('this cluster have normal\n')

print('---- Program Finish ---------')

