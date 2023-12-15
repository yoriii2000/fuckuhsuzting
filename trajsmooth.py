import math
import scipy.linalg
import numpy as np
import open3d as o3d
import os as os

# import transformat as trans


import glob
import shutil
import re

def ReadXyzFile(filename):
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

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def Sq2(value):

    return value*value

def Nuvw(PointBase, Point):

    # SaveFile('PointBase.xyz', PointBase)

    for i in range(1, len(Point)):
        # print(Point[i])
        # Distance
        d = math.sqrt(Sq2(Point[i][0] - PointBase[0]) + Sq2(Point[i][1] - PointBase[1]) + Sq2(Point[i][2] - PointBase[2]))

        # Get the vector reference which has smallest distance from base point
        if i == 1:  # initial
            d_min = d
            Point_ref = [Point[i][0], Point[i][1], Point[i][2]]
        else:
            if d < d_min:
                d_min = d
                Point_ref = [Point[i][0], Point[i][1], Point[i][2]]

    # print('\nPoint_ref and Point_base')
    # print(Point_ref)
    # print(PointBase)

    # Get the vector base and point reference (A)
    A = np.subtract(Point_ref, PointBase)

    ChosenPoint = []
    ChosenPointDot = []
    for PointNo in range(1, len(Point)):
        # Get the vector base and another point
        B = np.subtract(Point[PointNo], PointBase)
        # print('\nDot product to point:', Point[PointNo])
        AB_dot = np.dot(A, B)
        # print('dotprod:', AB_dot)

        # Sort the dot prod from smallest to largest:

        DotProdIsSmaller_sign = False
        # Initial value for chosen point
        if PointNo == 0:
            ChosenPointDot.append(AB_dot)  # Store the dotprod score
            ChosenPoint.append(Point[PointNo])  # Store the point

        else:
            # Compare the current dotprod with the latest dotprod
            for ChosenPointDot_no in range(0, len(ChosenPointDot)):
                if AB_dot < ChosenPointDot[ChosenPointDot_no]:
                    ChosenPointDot.insert(ChosenPointDot_no, AB_dot)
                    ChosenPoint.insert(ChosenPointDot_no, Point[PointNo])
                    DotProdIsSmaller_sign = True
                    break

            if DotProdIsSmaller_sign == False:
                ChosenPointDot.append(AB_dot)
                ChosenPoint.append(Point[PointNo])

        # print('\n Chosen point dot')
        # print(ChosenPointDot)

    # Just take the 3 smallest dot prod value
    ChosenPoint = ChosenPoint[:3]
    # print('\nChosen points:')
    # print(ChosenPoint)

    # Filter the chosen point which has smallest distance (max 2 points)
    FinalChosenPointD_list = []
    FinalChosenPoint = []
    # append = FinalChosenPoint.append

    for i in range(0, len(ChosenPoint)):
        # Distance
        d = math.sqrt(Sq2(ChosenPoint[i][0] - PointBase[0]) + Sq2(ChosenPoint[i][1] - PointBase[1]) + Sq2(ChosenPoint[i][2] - PointBase[2]))

        DistIsSmaller_Sign = False

        if i == 0:  # initial
            FinalChosenPointD_list.append(d)
            FinalChosenPoint.append([ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])

        else:
            for FinalChosenPoint_no in range(0, len(FinalChosenPoint)):
                if d < FinalChosenPointD_list[FinalChosenPoint_no]:
                    FinalChosenPointD_list.insert(FinalChosenPoint_no, d)
                    FinalChosenPoint.insert(FinalChosenPoint_no,[ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])
                    DistIsSmaller_Sign = True
                    break

            if DistIsSmaller_Sign == False:
                FinalChosenPointD_list.append(d)
                FinalChosenPoint.append([ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])

    FinalChosenPoint[2] = Point_ref

    # print('\n Final Chosen point')
    # print('xyz points = ', FinalChosenPoint)
    # SaveFile('FinalChosenPoint.xyz', FinalChosenPoint)

    # print('Base Point')
    # print('原點 = ', PointBase)

    return FinalChosenPoint

a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluster_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluster_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1

shutil.rmtree('/Users/yxc/研究所/Layering/Output File')
os.makedirs('/Users/yxc/研究所/Layering/Output File')

for protruison_no in range(0, protrusion_numaber):

    print('protruison_no = ', protruison_no)
    ba = sorted(glob.glob(os.path.join("Process/", "*layer_after{}*".format(protruison_no))), key=lambda x: (int(re.split('layer_after|_|.xyz', x)[2])))
    # sa = sorted(glob.glob(os.path.join("twotraj/", "*twodtraj{}*".format(protruison_no))), key=os.path.getmtime)

    print('filename = ', ba)
    for Bfile0_no in range(0, len(ba)):
        # -----------------------------------------------------------------------------------------------------------------

        sa = sorted(glob.glob(os.path.join("twotraj/", "*twodtraj{}_{}_*".format(protruison_no, Bfile0_no))), key=lambda x: (int(re.split('twodtraj|_|.xyz', x)[2]),
                                                                                                                             int(re.split('twodtraj|_|.xyz', x)[3])))
        print('sa =', sa)

        for layerline_no in range(0, len(sa)):
            print('line = ', layerline_no)

            bf = open(ba[Bfile0_no], "r")
            blines = bf.readlines()
            # print('No of Points [XYZ]:', len(blines))
            Bfile = []
            for x in range(0, len(blines)):
                RawData = blines[x].strip().split(" ")  # [x y z] from File
                Bfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

            sf = open(sa[layerline_no], "r")
            slines = sf.readlines()
            # print('No of Points [XYZ]:', len(slines))
            Sfile = []
            for x in range(0, len(slines)):
                RawData = slines[x].strip().split(" ")  # [x y z] from File
                Sfile.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

            data = np.asarray(Bfile)
            source = np.asarray(Sfile)

            X = []
            Y = []
            XX = []
            YY = []

            for i in range(0, len(source)):
                X.append(source[i][0])
                Y.append(source[i][1])

            XX = np.asarray(X)
            YY = np.asarray(Y)

            A = np.c_[
                np.ones(data.shape[0]), data[:, :2], np.prod(data[:, :2], axis=1), data[:, :2] ** 2, data[:, :2] ** 3]
            C, _, _, _ = scipy.linalg.lstsq(A, data[:, 2])

            Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX * YY, XX ** 2, YY ** 2, XX ** 3, YY ** 3], C)
            ZZ = Z.flatten()

            Grid = []
            for pointNo in range(0, len(XX)):
                Grid.append([XX[pointNo], YY[pointNo], ZZ[pointNo]])

            SaveFile('Process/grid{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), Grid)

            tra_inv = np.genfromtxt("transform/tra_inv_{}.xyz".format(protruison_no), dtype=None, comments='#',
                                    delimiter=' ')
            traj_before = o3d.io.read_point_cloud(
                'Process/grid{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1))
            traj_after = traj_before.transform(tra_inv)

            ntraj_after = np.asarray(traj_after.points)
            newtraj_after = ntraj_after.tolist()
            if newtraj_after[0][2] < newtraj_after[len(newtraj_after) - 1][2]:
                newtraj_after.reverse()
                SaveFile('Output File/traj{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), newtraj_after)
            else:
                SaveFile('Output File/traj{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), newtraj_after)
