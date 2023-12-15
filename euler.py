import numpy as np
import math
import glob
import os as os

import time
import shutil
import re
# import numba as nb

# @nb.jit
def SaveFile(Pcd_File_Name, PCDList):


    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1
shutil.rmtree('/Users/yxc/研究所/Layering/xyzRxyz')
os.makedirs('/Users/yxc/研究所/Layering/xyzRxyz')

for protruison_no in range(0, protrusion_numaber):
    print('protrusion_numaber = ', protruison_no)

    Afile0 = sorted(glob.glob(os.path.join("rTe_matric{}/".format(protruison_no), "*.xyz")), key=os.path.getmtime)
    XYZfile0 = sorted(glob.glob(os.path.join("rTe_matric{}/".format(protruison_no), "*.xyz")), key=os.path.getmtime)
    AllxyzRxyz0 = []
    no = 1
    print(len(Afile0))
    for Afile0_no in range(0, len(Afile0)):  # len(Afile0)

        # file = open('rTe_matric/rTe0_0.xyz')
        file = open(Afile0[Afile0_no], "r")
        # print('------------------------------------------',Afile0[Afile0_no])

        lines = file.readlines()
        # print('No of Points [XYZ]:', len(lines))
        transf = []
        for x in range(0, len(lines)):
            RawData = lines[x].strip().split(" ")  # [x y z] from File
            transf.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])

        rTe = np.asarray(transf)
        # print('rTe =\n', rTe)

        R = rTe
        if R[2, 0] < 1:
            if R[2, 0] > -1:
                thetaY = math.asin(-R[2, 0])
                thetaZ = math.atan2(R[1, 0], R[0, 0])
                thetaX = math.atan2(R[2, 1], R[2, 2])
            else:
                thetaY = math.pi / 2
                thetaZ = -math.atan2(-R[1, 2], R[1, 1])
                thetaX = 0
        else:
            thetaY = -(math.pi / 2)
            thetaZ = math.atan2(-R[1, 2], R[1, 1])
            thetaX = 0

        thetaX = thetaX * 180 / math.pi
        thetaY = thetaY * 180 / math.pi
        thetaZ = thetaZ * 180 / math.pi
        # print('euler angle = ', thetaX * 180 / math.pi, thetaY * 180 / math.pi, thetaZ * 180 / math.pi)
        # print(thetaX, thetaY, thetaZ)
        Rxyz = [thetaX, thetaY, thetaZ]
        # print('Rxyz =', Rxyz)

        xyzfile = open(XYZfile0[Afile0_no], "r")
        lines = xyzfile.readlines()
        # print('No of Points [XYZ]:', len(lines))
        transf = []
        for x in range(0, len(lines)):
            RawData = lines[x].strip().split(" ")
            transf.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])

        rTexyz = np.asarray(transf)

        xyz = np.array(rTexyz[0:3, 3] / 1000)  # 軟體需要，毫米單位轉公尺
        xxyyzz = []

        xxyyzz0 = xyz.reshape(1, -1)
        xxyyzz0 = xxyyzz0.flatten()
        xxyyzz0 = xxyyzz0.tolist()
        xxyyzz.append(xxyyzz0)
        # SaveFile('xxyyzz_{}.xyz'.format(Afile0_no), xxyyzz)

        xyzRxyz = np.append(xyz, Rxyz)
        # print('xyzRxyz =', xyzRxyz)

        # SaveFile('xyzRxyz0/xyzRxyz0_{}.xyz'.format(Afile0_no), xyzRxyz)

        #
        allxyzRxyz = xyzRxyz.reshape(1, -1)
        allxyzRxyz = allxyzRxyz.flatten()
        allxyzRxyz = allxyzRxyz.tolist()
        AllxyzRxyz0.append(allxyzRxyz)
        All = AllxyzRxyz0

        time.sleep(0.01)
    #     relay_no = Afile0_no + 1
    #     if relay_no % 12 == 0:
    #         insert_no = Afile0_no + no
    #         no += 1
    #                                                 # -101.404, -55.159, 51.422
    #         relay_point = [-0.4011, 0.9951, 0.2327, 0, -87.641, -68.043]  # 中繼點位置XYZRXYZ，單位為公尺
    #         # print('insert_no', insert_no)
    #         AllxyzRxyz0.insert(insert_no, relay_point)
    #         # print('insert finish')
    #
    # relay_point = [-0.4011, 0.9951, 0.2327, 0, -87.641, -68.043]   # 中繼點位置XYZRXYZ，單位為公尺
    # AllxyzRxyz0.insert(0, relay_point)
    #

    # # firstpoint = [-0.1880, 1.1405, 0.0605, 153.673, -80.460, 130.544]
    # firstpoint = [-0.2759, 1.0441, 0.1053, 178.835, -83.137, 101.256]
    #
    # AllxyzRxyz0.insert(1, firstpoint)

    # AllxyzRxyz0.insert(len(AllxyzRxyz0), relay_point)

    SaveFile('xyzRxyz/AllxyzRxyz{}.xyz'.format(protruison_no), AllxyzRxyz0)






