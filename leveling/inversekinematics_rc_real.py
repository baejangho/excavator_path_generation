from math import *
import numpy as np

rcl1=0.56
rcl2=0.3
rcl3=0.165

#l1=6.246
#l2=3.112
#l3=1.913

l1 = 6.24565
l2 = 3.11364
l3 = 1.910051

def inversekinematics_rc(X, Y, Ang):
    # aoa 단위 : -0.1rad
    # x 단위 : m
    # y 단위 : m
    AOA = -Ang*10 # 단위 맞춰줌

    P_x = X - rcl3 * float(cos(AOA))  #Px'
    P_y = Y - rcl3 * float(sin(AOA))  #Py'

    rcl4 = sqrt(P_x**2+P_y**2)

    Ang_Boom2 = acos(np.clip((rcl1 ** 2 + rcl4 ** 2 - rcl2 ** 2)/(2 * rcl1 * rcl4), -1, 1))  # 세타_1'
    Ang_Boom3 = atan2(P_y,P_x)  # 세타_1''

    Ang_Boom = Ang_Boom2 + Ang_Boom3 #세타1
    Ang_Arm = -(pi - acos(np.clip((rcl1 ** 2 + rcl2 ** 2 - rcl4 ** 2)/(2 * rcl1 * rcl2), -1, 1)))   #세타2

    Ang_Bkt = AOA - Ang_Boom - Ang_Arm

    return round(Ang_Boom, 3), round(Ang_Arm, 3), round(Ang_Bkt, 3)

def inversekinematics_real(X, Y, AOA):
    P_x = X - l3 * float(cos(AOA))  # Px'
    P_y = Y - l3 * float(sin(AOA))  # Py'

    l4 = sqrt(P_x ** 2 + P_y ** 2)

    Ang_Boom2 = acos(np.clip((l1 ** 2 + l4 ** 2 - l2 ** 2) / (2 * l1 * l4), -1, 1))  # 세타_1'
    Ang_Boom3 = atan2(P_y, P_x)  # 세타_1''

    Ang_Boom = Ang_Boom2 + Ang_Boom3  # 세타1
    Ang_Arm = -(pi - acos(np.clip((l1 ** 2 + l2 ** 2 - l4 ** 2) / (2 * l1 * l2), -1, 1)))  # 세타2

    Ang_Bkt = AOA - Ang_Boom - Ang_Arm

    return round(Ang_Boom, 3), round(Ang_Arm, 3), round(Ang_Bkt, 3)

def inversekinematics_real2(X, Y, bktAng):

    l2_new  = sqrt(l2 ** 2 + l3 ** 2 - 2*l2*l3*cos(bktAng))
    l_ground = sqrt(X ** 2 + Y ** 2)

    Ang_Arm = -(pi - acos(np.clip((l1**2 + l2_new **2 - l_ground ** 2)/(2*l1*l2_new),-1,1)))
    Ang_Boom = acos(np.clip((l1**2 + l_ground **2 - l2_new ** 2)/(2*l1*l_ground),-1,1)) + atan2(Y,X)

    return round(Ang_Boom, 3), round(Ang_Arm, 3)


def forwardkinematics_rc(Ang_Boom, Ang_Arm, Ang_Bkt):

    X = rcl1*cos(Ang_Boom) + rcl2*cos((Ang_Boom+Ang_Arm)) + rcl3*cos((Ang_Boom+Ang_Arm+Ang_Bkt))
    Y = rcl1*sin(Ang_Boom) + rcl2*sin((Ang_Boom+Ang_Arm)) + rcl3*sin((Ang_Boom+Ang_Arm+Ang_Bkt))
    AOA = Ang_Boom + Ang_Arm + Ang_Bkt

    return X, Y, AOA

def forwardkinematics_real(Ang_Boom, Ang_Arm, Ang_Bkt):

    X = l1 * cos(Ang_Boom) + l2 * cos((Ang_Boom + Ang_Arm)) + l3 * cos((Ang_Boom + Ang_Arm + Ang_Bkt))
    Y = l1 * sin(Ang_Boom) + l2 * sin((Ang_Boom + Ang_Arm)) + l3 * sin((Ang_Boom + Ang_Arm + Ang_Bkt))
    AOA = Ang_Boom + Ang_Arm + Ang_Bkt

    return X, Y, AOA