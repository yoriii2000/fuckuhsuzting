import File
import open3d as o3d
import os as os
import glob

import os as os
import open3d as o3d
import numpy as np
import math
from numpy.linalg import inv

import time
import shutil
import re

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


def ReadXyzNorFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList


# protrusion_numaber = 3
a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1

for protruison_no in range(0, protrusion_numaber):
    print('protruison_no = ', protruison_no)

    f0 = sorted(glob.glob(os.path.join("grindcoor{}/".format(protruison_no), "*.xyz")), key=lambda x: (
        int(re.split('cTt{}_|_|.xyz'.format(protruison_no), x)[1]),
        int(re.split('cTt{}_|_|.xyz'.format(protruison_no), x)[2]),
        int(re.split('cTt{}_|_|.xyz'.format(protruison_no), x)[1])))

    shutil.rmtree('/Users/yxc/研究所/Layering/rTe_matric{}'.format(protruison_no))
    os.makedirs('/Users/yxc/研究所/Layering/rTe_matric{}'.format(protruison_no))
    print('point_no = ', f0)
    ##
    # o0 = sorted(glob.glob(os.path.join("grindcoor{}/".format(protruison_no), "*cTt{}_*_2_*".format( protruison_no))), key=os.path.getmtime)
    # print('o0 = ', len(o0))
    ##
    for file0_no in range(0, len(f0)):

        # 軌跡點相對模型原點 cTt -----------------------------------------------------------------------------------------------
        file = open(f0[file0_no], "r")
        print('------------------------------------------', f0[file0_no])
        lines = file.readlines()
        # print('No of Points [XYZ]:', len(lines))
        transf = []
        for x in range(0, len(lines)):
            RawData = lines[x].strip().split(" ")  # [x y z] from File
            transf.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])

        cTt = np.asarray(transf)
        # print(cTt)

        tTc = inv(cTt)
        # print(transfM)
        # print('tTc =\n', tTc)

        # 模型原點相對法蘭面 eTc -----------------------------------------------------------------------------------------------
        eTc = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1.8385e+02], [0, 0, 0, 1]])  # code 裡單位為毫米mm  golfmodel
        # eTc = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1.3965e+02], [0, 0, 0, 1]])  # code 裡單位為毫米mm

        # print('eTC =\n', eTc)

        cTe = inv(eTc)
        # print('cTe =\n', cTe)

        # 研磨點相對手臂 rTt ---------------------------------------------------------------------------------------------------
        # # 砂輪研磨點角度
        alpha = -((math.pi / 180) * 105.1)
        beta = -((math.pi / 180) * 0.0000009)
        gamma = -((math.pi / 180) * 0.000012)
        Rx = np.array([[1, 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
        Ry = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0], [-math.sin(beta), 0, math.cos(beta)]])
        Rz = np.array([[math.cos(gamma), -math.sin(gamma), 0], [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])

        rTT = np.dot(np.dot(Rx, Ry), Rz)


        # t = np.array([[-1.159e+02, 1.2900e+03, 2.1780e+01]])       # 研磨點 後退0.3mm
        # t = np.array([[-1.159e+02, 1.2901e+03, 2.1755e+01]])       # 研磨點 後退0.2mm

        # t = np.array([[-8.9511e+01, 1.2916e+03, 2.5758e+01]])       # 研磨點 後退0.15mm  model8長方形凸點
        t = np.array([[-9.0322e+01, 1.2920e+03, 2.5538e+01]])       # 研磨點 原始點位

        # t = np.array([[-8.9511e+01, 1.2918e+03, 2.5702e+01]])       # 研磨點 前進0.1mm                                    model8長方形凸點 過摩0.25
        # t = np.array([[-8.9511e+01, 1.2919e+03, 2.5679e+01]])         # 研磨點 前進0.2mm      # golfmodel     model_0
        # t = np.array([[-8.9511e+01, 1.2919e+03, 2.5668e+01]])          # 研磨點 前進0.25mm    # model8三角形凸點
        # t = np.array([[-8.9511e+01, 1.2921e+03, 2.5623e+01]])          # 研磨點 前進0.45mm    # model8三角形凸點 過摩 0.2    model8長方形凸點 過摩0.6 多0.35
        
        # t = np.array([[-8.9511e+01, 1.2923e+03, 2.5578e+01]])          # 研磨點 前進0.65mm  # model8三角形凸點 過摩 0.4
        # t = np.array([[-8.9511e+01, 1.2924e+03, 2.5556e+01]])          # 研磨點 前進0.75mm                                  model9_3

        # t = np.array([[-8.9511e+01, 1.2926e+03, 2.5511e+01]])          # 研磨點 前進0.85mm    # model8三角形凸點 過摩 0.6     model9_2、model9_4
        # t = np.array([[-8.9511e+01, 1.2927e+03, 2.5488e+01]])          # 研磨點 前進0.95mm
        # t = np.array([[-8.9511e+01, 1.2928e+03, 2.5466e+01]])          # 研磨點 前進1.05mm    #model9_1

        # if file0_no > len(f0) - 30:
        #     t = np.array([[-8.9511e+01, 1.2917e+03, 2.5724e+01]])       # 研磨點 原始點位

        rTT = np.c_[rTT, t.T]
        e = [[0, 0, 0, 1]]
        rTT = np.r_[rTT, e]
        # print('rTt =\n', rTT)

        # 法蘭面相對手臂 rTe ---------------------------------------------------------------------------------------------------

        rTe = np.dot(np.dot(rTT, tTc), cTe)
        # rTe = rTe[0:3, 0:3]
        # print('rTe =\n', rTe)

        SaveFile('rTe_matric{}/rTe{}_{}.xyz'.format(protruison_no, protruison_no, file0_no), rTe)

        # time.sleep(0.01)


