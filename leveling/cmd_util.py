import math

def swingToboom(X_sw, Y_sw, Z_sw):
    ### 단위: m  ###
    Ang_Swing_rad = math.atan2(Y_sw,X_sw) + math.asin(0.030/math.sqrt(X_sw**2+Y_sw**2)) # 단위: rad
    Ang_Swing_degree = Ang_Swing_rad*180/math.pi
    X_Bm = math.sqrt(X_sw**2+Y_sw**2-0.030**2) - 0.120  
    Z_Bm = Z_sw
    
    return X_Bm, Z_Bm, Ang_Swing_degree

def excavation_location(ROS_cmd, idx):

    ExcLocation = ROS_cmd["location"][idx]
    '''변환'''
    first_location = [ExcLocation["firstPoint"]["N"], ExcLocation["firstPoint"]["E"], ExcLocation["firstPoint"]["Z"]]
    second_location = [ExcLocation["secondPoint"]["N"], ExcLocation["secondPoint"]["E"], ExcLocation["secondPoint"]["Z"]]
    third_location = [ExcLocation["thirdPoint"]["N"], ExcLocation["thirdPoint"]["E"], ExcLocation["thirdPoint"]["Z"]]
    fourth_location = [ExcLocation["fourthPoint"]["N"], ExcLocation["fourthPoint"]["E"], ExcLocation["fourthPoint"]["Z"]]

    dist_1_location = first_location[0]**2 + first_location[1]**2
    dist_2_location = second_location[0]**2 + second_location[1]**2
    dist_3_location = third_location[0]**2 + third_location[1]**2
    dist_4_location = fourth_location[0]**2 + fourth_location[1]**2

    first_location = [dist_1_location, ExcLocation["firstPoint"]["N"], ExcLocation["firstPoint"]["E"], ExcLocation["firstPoint"]["Z"]]
    second_location = [dist_2_location, ExcLocation["secondPoint"]["N"], ExcLocation["secondPoint"]["E"], ExcLocation["secondPoint"]["Z"]]
    third_location = [dist_3_location, ExcLocation["thirdPoint"]["N"], ExcLocation["thirdPoint"]["E"], ExcLocation["thirdPoint"]["Z"]]
    fourth_location = [dist_4_location, ExcLocation["fourthPoint"]["N"], ExcLocation["fourthPoint"]["E"], ExcLocation["fourthPoint"]["Z"]]

    ExcLocation_list = [first_location, second_location, third_location, fourth_location]
    ExcLocation_list = sorted(ExcLocation_list, key=lambda x: -x[0])
    #print(ExcLocation_list)
    
    location_X_Far = (ExcLocation_list[0][1]+ExcLocation_list[1][1])/2
    location_X_Near = (ExcLocation_list[2][1]+ExcLocation_list[3][1])/2
    location_Y_Far = (ExcLocation_list[0][2]+ExcLocation_list[1][2])/2
    location_Y_Near = (ExcLocation_list[2][2]+ExcLocation_list[3][2])/2
    location_Z_Far = (ExcLocation_list[0][3]+ExcLocation_list[1][3])/2
    location_Z_Near = (ExcLocation_list[2][3]+ExcLocation_list[3][3])/2
    
    location_distFar, design_depthFar, swing_angleFar = swingToboom(location_X_Far, location_Y_Far,location_Z_Far)
    location_distNear, design_depthNear, swing_angleNear = swingToboom(location_X_Near, location_Y_Near,location_Z_Near)
    
    if abs(swing_angleFar - swing_angleNear) < 0.1 :
        exc_swing_angle = swing_angleFar
    else:
        print("y좌표에 대한 swing 값 변환이 다름 : ", abs(swing_angleFar - swing_angleNear))
        exc_swing_angle = (swing_angleFar + swing_angleNear)/2
    
    if abs(design_depthFar - design_depthNear) < 0.05:
        design_depth = design_depthFar
    else:
        print("design_depth가 앞,뒤 서로 다름")
        design_depth = (design_depthFar + design_depthNear)/2

    return location_distFar, location_distNear, design_depth, design_depthFar, design_depthNear, exc_swing_angle

        
def dump_location(ROS_cmd):
    DumpLocation = ROS_cmd["DumpLocation"]        # 상차 위치 딕셔너리
    '''변환'''
    location_list = [DumpLocation["N"], DumpLocation["E"], DumpLocation["Z"]]
    DumpDist, DumpDepth, DumpSwingAngle = swingToboom(location_list[0],location_list[1],location_list[2])
    
    
    return DumpDist, DumpDepth, DumpSwingAngle

def calc_num_iteration(distFar, disNear, depth, cur_depth = -2.0):

    S = (distFar - disNear) * (cur_depth-depth) + (cur_depth-depth) * 1
    num_iteration = math.floor(S+0.5)
    #num_iteration = math.ceil(S)

    return num_iteration
