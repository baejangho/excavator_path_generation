import numpy as np
from math import *
import open3d as o3d

def loading_point(points, h):
    # 포인트들의 중간점을 계산하기 위한 초기 값 설정
    Point_dept = max(points[2][1],points[3][1]) + h * 1.1

    centerPoint_dist = (points[2][2] + points[3][2]) * 0.5 + 1.3
    centerPoint_side = (points[2][0] + points[3][0]) * 0.5 - 1.1

    delta_swing = 90-degrees(atan2(centerPoint_dist, centerPoint_side))

    delta_swing = np.clip(delta_swing, -7.5, 7.5)
    return Point_dept, delta_swing

def loading_point2(pcAdress, theta, xleft, xright, zfar, znear, heightlimit, loadinglevel):
    pcd = o3d.io.read_point_cloud(pcAdress)
    xyz = np.asarray(pcd.points)

    indexX, indexY, indexZ = [], [], []
    horizontal, vertical = [], []

    for a in range(len(xyz)):
        z = xyz[a][2]
        y = xyz[a][1]

        horizontal_val = -z * cos(radians(theta)) + y * sin(radians(theta)) + 1.190
        vertical_val = z * sin(radians(theta)) + y * cos(radians(theta)) + 1.184

        xyz[a][2] = horizontal_val
        xyz[a][1] = vertical_val

        if xyz[a][0] < xleft or xyz[a][0] > xright:
            indexX.append(a)
    reducedPCX = np.delete(xyz, indexX, 0)

    for b in range(len(reducedPCX)):
        if reducedPCX[b][1] > 1.2:
            indexY.append(b)
    reducedPC1 = np.delete(reducedPCX, indexY, 0)

    for c in range(len(reducedPC1)):
        if reducedPC1[c][2] > zfar or reducedPC1[c][2] < znear:
            indexZ.append(c)
    reducedPC2 = np.delete(reducedPC1, indexZ, 0)

    for d in range(len(reducedPC2)):
        horizontal.append(reducedPC2[d][2])
        vertical.append(reducedPC2[d][1])

    combined = np.vstack((horizontal, vertical)).T
    sortedcombined = combined[combined[:,0].argsort()]

    SortedHorizontal, SortedVertical = [], []
    for e in range(len(sortedcombined)):
        SortedHorizontal.append(sortedcombined[e][0])
        SortedVertical.append(sortedcombined[e][1])

    HeightSoil_Truck = max(SortedVertical)

    if HeightSoil_Truck >= heightlimit and loadinglevel <= 2:
        loadinglevel += 1

    if loadinglevel > 2:
        loadinglevel = 3

    return loadinglevel

def loading_point2_vir(HeightSoil_Truck, heightlimit, loadinglevel):

    if HeightSoil_Truck >= heightlimit and loadinglevel <= 2:
        loadinglevel += 1

    if loadinglevel > 2:
        loadinglevel = 3

    return loadinglevel
