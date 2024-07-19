import numpy as np
import open3d as o3d
from math import *
import warnings
warnings.simplefilter('ignore',np.RankWarning)

def terrainFeature(routePLY, theta):
    pcd = o3d.io.read_point_cloud(routePLY)
    xyz = np.asarray(pcd.points)

    indexX, indexY, indexZ = [], [], []
    horizontal, vertical, index_x_005, index_y_005, index3 = [], [], [], [], []

    for a in range(len(xyz)):
        z = xyz[a][2]
        y = xyz[a][1]

        horizontal_val = -z * cos(radians(theta)) + y * sin(radians(theta)) + 1.190
        vertical_val = z * sin(radians(theta)) + y * cos(radians(theta)) + 1.184

        xyz[a][2] = horizontal_val
        xyz[a][1] = vertical_val

        #if xyz[a][0] < 0.6 or xyz[a][0] > 1.6:
        if xyz[a][0] < -1.4 or xyz[a][0] > 3.6:
            indexX.append(a)
    reducedPCX = np.delete(xyz, indexX, 0)

    for b in range(len(reducedPCX)):
        if reducedPCX[b][1] > 2:
            indexY.append(b)
    reducedPC1 = np.delete(reducedPCX, indexY, 0)

    for c in range(len(reducedPC1)):
        if reducedPC1[c][2] > 9:
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

    xlowlimit = round(min(SortedHorizontal),1)
    xhighlimit = round(max(SortedHorizontal),1)

    number = int((xhighlimit-xlowlimit)/0.1)

    for f in range(number):
        lowlimit = xlowlimit + f*0.1
        highlimit = xlowlimit + (f+1)*0.1
        versum, ci = 0,0

        for g in range(len(SortedHorizontal)):
            if SortedHorizontal[g] >= lowlimit and SortedHorizontal[g] < highlimit:
                versum += SortedVertical[g]
                ci += 1

        if ci != 0:
            avg = versum/ci
        else:
            avg = -2
            index3.append(f)

        index_x_005.append(round((lowlimit+highlimit)/2,2))
        index_y_005.append(round(avg,3))

    lowend, highend = [], []
    if len(index3) != 0:
        for h in range(len(index3)):
            if h < len(index3)-1:
                if index3[h+1] - index3[h] >= 2:
                    highend.append(index3[h])
            elif h == len(index3)-1:
                highend.append(index3[h])

            if h >= 1:
                if index3[h] - index3[h-1] >= 2:
                    lowend.append(index3[h])
            else:
                lowend.append(index3[h])

        for j in range(len(lowend)):
            for k in range(len(index3)):
                if index3[k] <= highend[j] and index3[k] >= lowend[j]:
                    height = ((index_x_005[index3[k]] - index_x_005[lowend[j] - 1]) * index_y_005[highend[j] + 1]
                              + (index_x_005[highend[j] + 1] - index_x_005[index3[k]]) * index_y_005[lowend[j] - 1]) / (index_x_005[highend[j] + 1] - index_x_005[lowend[j] - 1])
                    index_y_005[index3[k]] = height

    index_x_05, index_y_05, terrainTangent = [],[],[]
    terrainlowx = 3
    terrainhighx = 10

    plus = 0
    nn = int(terrainhighx - terrainlowx)

    for l in range(nn+1):
        horsection, versection = [],[]
        verversum, size, tangent = 0,0,0

        lowlimit1 = terrainlowx + plus
        highlimit1 = terrainlowx + plus + 1

        for m in range(len(index_x_005)):
            if index_x_005[m] >= lowlimit1 and index_x_005[m] < highlimit1:
                verversum += index_y_005[m]
                size += 1
                horsection.append(index_x_005[m])
                versection.append(index_y_005[m])

        if size == 0:
            avgavg = -2
        else:
            avgavg = round(verversum / size, 2)
            tangentarr = np.polyfit(horsection, versection, 1)
            tangent = tangentarr[0]

        index_x_05.append(round((lowlimit1 + highlimit1)/2,1))
        index_y_05.append(avgavg)

        terrainTangent.append(tangent)
        plus += 1

    '''for n in range(len(index_y_05)):
        if index_y_05[n] == 0:
            for o in [1,-1]:
                if index_y_05[n+o] != 0:
                    yintersect = index_y_05[n+o] - terrainTangent[n+o]*index_x_05[n+o]
                    index_y_05[n] = round(terrainTangent[n+o]*index_x_05[n] + yintersect,3)
                    break'''

    return index_x_05, index_y_05, index_x_005, index_y_005












