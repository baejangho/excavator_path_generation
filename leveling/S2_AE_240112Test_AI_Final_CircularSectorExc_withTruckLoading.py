## 시나리오 시연 8월 30일 ##

### CAN 관련 ###
import can.interfaces.pcan
from ctypes import c_uint16 as unsigned_byte  # C 16비트 unsigned int 데이터형
from ctypes import c_int16 as signed_byte  # C signed int 데이터형을 나타냅니다.
### CAN 관련 ###

### 스레딩 관련###
import threading
### 스레딩 관련###

### TCP/IP ###
from websock.WebSocketServer import WebSocketServer
import KU_MSG_JSON as ku_msg
import json
import cmd_util
### TCP/IP ###

### UI 관련 ###
import tkinter as tk
from tkinter import ttk
### UI 관련 ###

### AI 경로 생성 관련 ###
from AIbasedPathGeneration1R18_rule import *
from Loaidng_DecisionR7 import *
from truck_detection_Final_Replicatruck import *
### AI 경로 생성 관련 ###

### 카메라 관련 ###
import math
import pyrealsense2 as rs
import os
from pcToValueR3 import *
import cv2
### 카메라 관련 ###

### 경로데이터 저장관련 ###
import pandas as pd
from collections import deque
### 경로데이터 저장관련 ###

### 시뮬레이션 관련 : 건품연 명령 예시 ###
import virtual_cmdR1
import time
### 시뮬레이션 관련 ###

#################### 잘 안바뀌는 초기값 ####################
### 테스트 모드 선택 ###
CAN_TEST = False            # 전북대와 CAN 통신 활성화 시 True / 가상 CAN 통신으로 코드 테스트 시 False
TCPIP_TEST = False        # 건품연과 TCPIP 통신 활성화 시 True / TCPIP 통신없이 코드 테스트 시 False
cameraON = False             # 카메라 사용 시 True, 안 사용할 시 False
### 테스트 모드 선택 ###

### KU server IP, PORT ###
#HOST_S = '192.168.110.34'
if TCPIP_TEST:
    HOST_S = '192.168.110.48'
elif CAN_TEST:
    HOST_S = '127.0.0.1'
else:
    HOST_S = '127.0.0.1'
PORT_S = 9999                       
### KU server IP, PORT ###

### AI path generator 모듈 객체 선언 ###
path = ai_excavation_path()
### AI path generator 모듈 객체 선언 ###

####Tresh Value for initiation of tensorflow####
#아무짝에도 쓸모없는 값입니다. 그저 텐서플로 기반 AI모델을 미리 구동시켜서 작업 소요시간을 줄이기 위함입니다.
pos_beforeSwing = [3.2, 2.2, radians(-167)]
path.basicinput(2.9, -2.1, -2.1, -3, 9.3, pos_beforeSwing, 0.8)
tresh_joint, tresh_Traj, _ = path.pathgeneration(10)
####Tresh Value for initiation of tensorflow####

#################### 카메라 연결 관련 코드 start ####################
class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)
#################### 카메라 연결 관련 코드 start ####################

class Autonomous_Excavator():
    def __init__(self):

        self.stop_event = threading.Event()
        self.start_path_gen_event = threading.Event()
        
        self.EXC_state = "Initialization" # "Waiting, "Ready", "Working", "Emergency"
        
        self.logging_trg = False #데이터 로깅 트리거

        # 조인트/버켓 위치 및 aoa 경로 데이터 목록, csv 파일 저장될 시, 헤더 역할
        self.headers = ['time(s)','Dist_cmd(m)', 'Dept_cmd(m)', 'AOA_cmd(Deg)', 'SwingAng_cmd(Deg)', 'BoomAng_cmd(Deg)', 'ArmAng_cmd(Deg)', 'BktAng_cmd(Deg)',
                        'Dist_cur(m)', 'Dept_cur(m)', 'AOA_cur(Deg)', 'SwingAng_cur(Deg)', 'BoomAng_cur(Deg)', 'ArmAng_cur(Deg)', 'BktAng_cur(Deg)',
                        'excavation state']

        self.DataRoute = 'C:/Users/cksdu/Desktop/240523/testdata/PointCloud/20240523/recordAI_excLoading.csv'  # 굴삭 기록 저장시 조인트/버켓 위치 및 aoa 경로 데이터 파일(csv) 저장되는 주소

        self.pcroute = 'C:/Users/cksdu/Desktop/240523/testdata/PointCloud/20240523/w2pointcloud.ply'  # Poincloud 파일이 저장되는 주소

        self.log_deque = deque() #데이터 로깅 큐

        self.camCapture = False #카메라 촬영 버튼 누르기(True), 안 누르기(False)

        self.state_log = "준비"
        self.errorReset = False

        ### 전북대에 주는 초기 경로 리스트 및 트리거 ###
        self.cmd_data_list = [[10, 10, -150, -270]]
        self.trigger = 0

        self.initialize_GUI()

        if CAN_TEST:
            ### 실제 실차 테스트 ###
            self.bus = can.interfaces.pcan.pcan.PcanBus(interface='pcan', channel='PCAN_PCIBUS1', receive_own_messages=False, bitrate=500000, f_clock_mhz=80, auto_reset=True)
            RXFilter = [{"can_id": 0x01F7, "can_mask": 0xffff, "extended": False}]
            self.bus.set_filters(filters=RXFilter)

        else:
            ### 가상 CAN 서버(건품연 제어기 없이 시뮬레이션)
            self.bus = can.interface.Bus('test',interface='virtual',receive_own_messages=False)
            RXFilter = [{"can_id": 0x01F7, "can_mask": 0xffff, "extended": False}]
            self.bus.set_filters(filters=RXFilter)
            self.bus_virtual = can.interface.Bus('test',interface='virtual',receive_own_messages=False)
            RXFilter_virtual = [{"can_id": 0x01F8, "can_mask": 0xffff, "extended": False}]
            self.bus_virtual.set_filters(filters=RXFilter_virtual)
            virtual_CAN_th = threading.Thread(target=self.th_VirtualCAN,args=())          # CAN 통신 스레드 : 100ms 마다 데이터 수신 후 joints 명령 송신
            virtual_CAN_th.start()

        if TCPIP_TEST:
            ### 실제 실차 테스트 ###
            self.S_ip = HOST_S
            self.S_port = PORT_S
            
            print('waiting client ...')
            self.server = WebSocketServer(ip=self.S_ip, port=PORT_S,
                            on_data_receive=self.th_TCPIP_Rx,
                            on_connection_open=self.on_connection_open)
            server_thread = threading.Thread(target=self.server.serve_forever, args=(), daemon=True)
            server_thread.start()
        else:
            ### tcp/ip 통신 안할 때 ###
            #self.ROS_cmd = virtual_cmdR1.work_cmd_trenching()              # 건품연 굴착(트렌칭 굴착)위치 명령 예시 호출
            self.ROS_cmd = virtual_cmdR1.work_cmd_SE()                      # 건품연 굴착(부채꼴 굴착)위치 명령 예시 호출

        print('tcpip 스레드 시작')
        TCPIP_Tx_th = threading.Thread(target=self.th_TCPIP_Tx,args=())        # TCP/IP TX 스레드 : 필요할 때 TX 송신
        TCPIP_Tx_th.start()   ##캔통신 테스트시 주석

        CanRx_th = threading.Thread(target=self.th_CanRx,args=())          # CAN 통신 스레드 : 10ms 마다 joints 명령 수신
        CanRx_th.start()

        CanTx_th = threading.Thread(target=self.th_CanTx, args=())         # CAN 통신 스레드 : 100ms  joints 명령 송신
        CanTx_th.start()

        record_th = threading.Thread(target=self.th_recordData, args=())   
        record_th.start()

        if cameraON:
            cameraPCget_th = threading.Thread(target=self.cameraPCget, args=())
            cameraPCget_th.start()

    ###################################### 쓰레드 함수 ###################################################
    def th_TCPIP_Tx(self):
        print("PATH generator thread start!")
        prev_exc_state = ""
        LocalWork = 1

        while not self.stop_event.is_set():
            if self.start_path_gen_event.is_set():                          # CAN으로부터 굴착기 조인트 각도 수신하면 True

                cur_exc_state = self.EXC_state

                if prev_exc_state != cur_exc_state:                         # EXC_state가 변화되었는지 확인
                    state_flag = True
                    cur_joint_angle = self.save_current_pos()               # 실행되는 순간의 굴착기 조인트 각도 저장
                    prev_exc_state = cur_exc_state
                else:
                    state_flag = False

                if cur_exc_state == "Initialization":
                    print("초기화 작업 중")
                    if state_flag:
                        self.cmd_data_list = cur_joint_angle
                        self.trigger = 0
                        self.state_log = "init_state"
                        prev_exc_state = cur_exc_state

                elif cur_exc_state == "Working":
                    print("자율작업 중")
                    ROS_CMD = self.ROS_cmd

                    ######################### 굴착 작업 정보 획득 #########################
                    self.location = ROS_CMD["location"]
                    print(self.location)
                    num_UnitWork = len(self.location)

                    self.location_distFar_list = []
                    self.location_distNear_list = []
                    self.design_depth_list = []
                    self.design_depthFar_list = []
                    self.design_depthNear_list = []
                    self.design_swingAngle_list = []
                    for i in range(num_UnitWork):  
                        self.location_distFar, self.location_distNear, self.design_depth, self.design_depthFar, self.design_depthNear, self.design_swingAngle \
                            = cmd_util.excavation_location(ROS_CMD,i)
                        self.location_distFar_list.append(round(self.location_distFar,2))
                        self.location_distNear_list.append(round(self.location_distNear,2))
                        self.design_depth_list.append(round(self.design_depth,2))
                        self.design_depthFar_list.append(round(self.design_depthFar, 2))
                        self.design_depthNear_list.append(round(self.design_depthNear, 2))
                        self.design_swingAngle_list.append(round(self.design_swingAngle,2))
                    #self.design_depth_list = [-3, -3, -3, -3, -3]
                    print(self.location_distFar_list)
                    print(self.location_distNear_list)  
                    print(self.design_swingAngle_list)      
                    self.DumpDist, self.DumpDepth, self.DumpSwingAngle = cmd_util.dump_location(ROS_CMD)
                    ######################### 굴착 작업 정보 획득 #########################

                    ######################### UI 업데이트 #########################
                    self.unit_info_UI_MSG.set(f'Dist Far : {(self.location_distFar_list)}\n'
                                            f'Dist Near : {(self.location_distNear_list)}\n'
                                            f'Design Depth : {(self.design_depth_list)}\n'
                                            f'Swing Angle : {(self.design_swingAngle_list)}')
                    self.targetDepth_UI_MSG.set(f'Dump dist : {round(self.DumpDist,2)}\n'
                                            f'Dump depth : {round(self.DumpDepth,2)}\n'
                                            f'Dump swing angle : {round(self.DumpSwingAngle,2)}\n'
                                            f'Loading level : 0\n')
                    self.Total_UnitIDX_UI_MSG.set(f"total Unit num : {num_UnitWork}")
                    self.UnitIDX_UI_MSG.set(f"current local idx : {LocalWork}")
                    ######################### UI 업데이트 #########################
                    
                    ######################### 스윙파라미터 #########################
                    swing_num = num_UnitWork
                    swing_angle_list = self.design_swingAngle_list
                    ######################### 스윙파라미터 ########################

                    ######################### 횟수 정하는 알고리즘 ################
                    num_iteration_list = []
                    for num in range(num_UnitWork):
                        print(num)
                        print(self.location_distNear_list[num])
                        num_iteration = \
                            cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.design_depth_list[num], cur_depth = -2.0)
                        print(num_iteration)
                        num_iteration_list.append(num_iteration)
                        #num_iteration = 4
                    print("num interation list:",num_iteration_list) 
                    num_iteration_max = round(max(num_iteration_list),2)
                    num_iteration_min = round(min(num_iteration_list),2)
                    ######################## 횟수 정하는 알고리즘 ##################

                    if LocalWork == 1:
                        ### 건품연에 자동굴착 전체 작업 시작신호 "Started" 신호 송신 ###
                        KU_msg = ku_msg.state(type = "working", workingidx = 1, msg = "Started")
                        KU_msg = json.dumps(KU_msg)
                        if TCPIP_TEST:
                            time.sleep(1)
                            self.server.send(self.client, KU_msg)
                            
                        ### 건품연에 자동굴착 전체 작업 시작신호 "Started" 신호 송신 ###
                        print("자동굴착 전체 작업 시작 신호 송신, type = working, msg : Started")

                    if LocalWork < 2:
                        
                        ### 건품연에 자동굴착 단위 작업 시작신호 "Started" 신호 송신 ###
                        KU_msg = ku_msg.state(type = "squareidx", workingidx = 1, msg = "Started")
                        KU_msg = json.dumps(KU_msg)
                        if TCPIP_TEST:
                            time.sleep(1)
                            self.server.send(self.client, KU_msg)     
                        ### 건품연에 자동굴착 단위 작업 시작신호 "Started" 신호 송신 ###
                        print("자동굴착 단위 작업 시작 신호 송신, type = squareidx, msg : Started")

                        curswing, curboom, curarm, curbkt = self.swingangle, self.boomangle, self.armangle, self.bktangle
                        curdist, curdept, curaoa = forwardkinematics_real(radians(curboom), radians(curarm), radians(curbkt))

                        self.Total_cur_num_exc_UI_MSG.set(f"total exc num per unit : {num_iteration_min},{num_iteration_max}")
                        cur_auto_state_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13"]

                        ground_depth = -2

                        ######### 중요 ##########
                        self.prev_auto_state = ""
                        self.cur_auto_state = "0"
                        self.isReach = False
                        self.excavation_idx = 0
                        self.iteration = 1
                        self.swing_idx = 1
                        self.loading_iteration = 0
                        #loading_iter = 0
                        self.loadingLimit = 15
                        self.loadinglevel = 1
                        self.lift_dept = 2.2
                        self.load_rad = 1.6
                        self.zfar = 8.8
                        self.swingError = 1.1
                        ######### 중요 ###########

                        self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                        f"current sector index : {self.swing_idx}\n"
                                                        f"current excavation index : {self.iteration}\n"
                                                        )
                        
                        ### unit excvation 완료 시 while문 종료 ###
                        while True and not self.stop_event.is_set():
                            if TCPIP_TEST == False and CAN_TEST == True:
                                self.trigger = 1
                            self.cur_num_exc_UI_MSG.set(f'current exc num : {self.iteration}')
                            
                            if self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "0":
                                """
                                트렌칭 작업의 경우 swing 각이 0도가 아닌경우 swing cmd 생성
                                타 작업의 경우 초기 위치로 이동하기 위한 swing cmd 생성
                                """
                                self.state_log = "0-init swing"
                                self.swingError = 1.1

                                if abs(self.swingangle-swing_angle_list[(self.swing_idx-1)%swing_num]) < 1:
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                else:
                                    self.exc_seq_MSG.set('굴착단계 : 초기 swing 자세 이동')
                                    swingvel, swinggoal = 10, swing_angle_list[(self.swing_idx-1)%swing_num]  # deg/s, degree
                                    path.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = path.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "1":
                                self.swingError = 1.1
                                if self.iteration == 1 and self.loading_iteration == 0:
                                    self.state_log = "1-capture_position"
                                    self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동')
                                    lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 8.5, 0, radians(-75)
                                    path.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                    self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept, curaoa)
                                    print("카메라촬영 위치 이동", len(self.cmd_data_list))
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                else:
                                    self.state_log = "1-capture_position pass"
                                    self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동 pass')
                                    pass
                                self.prev_auto_state = self.cur_auto_state
                            
                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "2":
                                """
                                촬영 후 평균값 계산
                                """
                                self.swingError = 1.1
                                ### 촬영 사진 저장 ###
                                ################################################################################################
                                if self.iteration == 1 and self.loading_iteration == 0:
                                    self.state_log = "2-calc avg height"
                                    cameraTheta = 30
                                    self.soil_x_05, self.soil_y_05, _, _ = terrainFeature(self.pcroute, cameraTheta)
                                    print("Terrain point cloud generated")

                                    self.soilHeightAdjustment = (self.soil_y_05[0] + self.soil_y_05[1] + self.soil_y_05[2] +self.soil_y_05[3]
                                                                 + self.soil_y_05[4] + self.soil_y_05[5] + self.soil_y_05[6])/7
                                    #self.soilHeightAdjustment = -2.1

                                else:
                                    self.state_log = "2-calc avg height pass"
                                    pass
                                ################################################################################################
                                print(f"soil height :{self.soilHeightAdjustment}")
                                self.cmd_data_list = [self.cmd_data_list[-1]]
                                self.rev_auto_state = self.cur_auto_state
                            
                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "3":
                                self.swingError = 7
                                self.exc_seq_MSG.set('굴착단계 : 굴착 시작점 이동')
                                self.state_log = "3-init exc position"
                                goal_depth = round(self.design_depth_list[(self.swing_idx - 1) % swing_num], 2)
                                dragLengthList = [2.9, 4.2, 4.2, 3.9, 3.8, 3.8, 3.8, 2.9, 2.9, 2.9]
                                soildepthList = [-2.1, -2.2, -2.4, -2.5, -2.6, -2.8, -3.0, -3.2, -3.2, -3.2]

                                if self.iteration % 2 == 1 and self.iteration < num_iteration_list[(self.swing_idx - 1) % swing_num]:
                                    dragLength = dragLengthList[self.iteration - 1]
                                    soildepth = soildepthList[self.iteration - 1]
                                    initdist = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2) + \
                                               (goal_depth - soildepth) * 0.5 / (goal_depth - ground_depth)
                                elif self.iteration % 2 == 0 and self.iteration < num_iteration_list[(self.swing_idx - 1) % swing_num]:
                                    dragLength = dragLengthList[self.iteration - 1]
                                    soildepth = soildepthList[self.iteration - 1]
                                    initdist = round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2) - \
                                               ((goal_depth - soildepth) * 0.5 / (goal_depth - ground_depth)) + dragLength
                                elif self.iteration == num_iteration_max:
                                    soildepth = goal_depth
                                    initdist = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2)

                                LineEndDist = initdist
                                LineEndDept = soildepth + (self.soilHeightAdjustment + 2.1)
                                l1, l2, l3= 6.24565, 3.11364, 1.910051

                                lambda_theta = np.clip(
                                    (LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta) + radians(10)
                                linevel = 1.1
                                path.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA) # 초기위치

                                self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept, curaoa)
                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "4":
                                self.swingError = 1.1
                                # 굴착 경로 경로 생성
                                LiftVel = 0.7
                                if self.iteration == num_iteration_max:
                                    self.exc_seq_MSG.set('굴착단계 : 다듬기작업')
                                    FinalDist, Lift_dist, Lift_dept, Lift_aoa = \
                                        round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2), 3.2, self.lift_dept, radians(-167)
                                    self.cmd_data_list, Trajectory, _ = \
                                        path.final_path(curswing, curdist, curdept, curaoa, FinalDist, goal_depth, Lift_dist, Lift_dept, Lift_aoa, LiftVel)
                                    print("다듬기 경로 생성")
                                    self.state_log = "4-Final path"

                                elif self.iteration < num_iteration_max:
                                    self.exc_seq_MSG.set('굴착단계 : 굴착경로작업')
                                    pos_beforeSwing = [3.2, self.lift_dept, radians(-167)]
                                    path.basicinput(dragLength, soildepth, soildepth + (self.soilHeightAdjustment + 2.1), goal_depth, initdist, pos_beforeSwing, LiftVel)
                                    self.cmd_data_list, Trajectory, _ = path.pathgeneration(curswing)
                                    print("AI 경로 생성")
                                    self.state_log = "4-AI path"

                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "5":
                                self.swingError = 1.1
                                # 스윙 to 상차지점
                                self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                self.state_log = "5-swing to loading"
                                swingvel, swinggoal = 15, self.DumpSwingAngle  # deg/s, degree
                                path.swingInfo(swingvel, swinggoal)
                                self.cmd_data_list, Trajectory, _ = path.swingPath(curswing, curdist, curdept, curaoa)
                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "6":
                                self.swingError = 1.1
                                if self.loading_iteration <= 1:
                                    self.exc_seq_MSG.set('상차단계 : 덤프트럭 촬영')
                                    self.state_log = "6-Truck identification"
                                    self.truckBodypoints = truck_pointcloud(self.pcroute)
                                    print("Truck point cloud generated")
                                    #self.truckBodypoints = [(-1, 0.7, 9.3), (1, 0.7, 9.3), (-1, 0.7, 4.6), (1, 0.7, 4.6)]

                                self.LoadingTruck4PointLocation.set(f"< Truck 4 points(side, height, dist) >\n"
                                                                    f"1st point : {self.truckBodypoints[0]}\n"
                                                                    f"2nd point : {self.truckBodypoints[1]}\n"
                                                                    f"3rd point : {self.truckBodypoints[2]}\n"
                                                                    f"4th point : {self.truckBodypoints[3]}\n"
                                                                    )
                                loadingInit = max(self.truckBodypoints[2][2], self.truckBodypoints[3][2]) + 0.3
                                if self.loadinglevel == 1:
                                    self.znear = loadingInit + (8.8 - loadingInit) / 2
                                    height_limit = max(self.truckBodypoints[2][1], self.truckBodypoints[3][1]) + 0.35

                                xleft = min(self.truckBodypoints[2][0], self.truckBodypoints[3][0]) + 0.15
                                xright = max(self.truckBodypoints[2][0], self.truckBodypoints[3][0]) - 0.15
                                self.loadinglevel = loading_point2(self.pcroute, cameraTheta, xleft, xright, self.zfar, self.znear, height_limit, self.loadinglevel)

                                '''HeightSoil_Truck = -2 + 0.5*loading_iter
                                prior_loadinglevel = self.loadinglevel
                                self.loadinglevel = loading_point2_vir(HeightSoil_Truck, height_limit, self.loadinglevel)
                                if prior_loadinglevel != self.loadinglevel:
                                    loading_iter=0'''

                                if self.loadinglevel >= 2:
                                    self.znear = loadingInit
                                    self.zfar = loadingInit + (8.8 - loadingInit) / 2
                                    height_limit = max(self.truckBodypoints[2][1], self.truckBodypoints[3][1]) + 0.45

                                print(f"loading level : {self.loadinglevel}")
                                self.cmd_data_list = [self.cmd_data_list[-1]]
                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "7":
                                self.swingError = 1.1
                                # 상차지점으로(직선경로)
                                if self.loading_iteration == 0 :
                                    self.exc_seq_MSG.set('상차단계 : 스윙 조정')
                                    self.state_log = "7-mini alingmnet of swing"

                                    self.margin_height = 1.3

                                    ##################################################
                                    self.DumpHeight, DeltaSwingAng = loading_point(self.truckBodypoints, self.margin_height)
                                    ##################################################

                                    swingvel = 7.5# deg/s, degree
                                    self.DumpSwingAngle = self.DumpSwingAngle - DeltaSwingAng
                                    path.swingInfo(swingvel, self.DumpSwingAngle)
                                    self.cmd_data_list, Trajectory, _ = path.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                else:
                                    self.exc_seq_MSG.set('상차단계 : 스윙 조정 pass')
                                    self.state_log = "7-Pass"
                                    self.cmd_data_list = [self.cmd_data_list[-1]]

                                self.loading_iteration += 1
                                print(f"loading iteration : {self.loading_iteration}")
                                #loading_iter += 1
                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "8":
                                self.swingError = 1.1
                                # 상차지점으로(직선경로)
                                self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                                self.state_log = "8-loading position"

                                loadVel1, loadDept1, loadAOA1 = 0.6, self.DumpHeight, radians(-160)

                                if self.loadinglevel <= 1:
                                    loadInitDist1 = self.zfar - self.load_rad - 0.5
                                elif self.loadinglevel > 1:
                                    self.load_rad = 2.2
                                    loadInitDist1 = self.zfar - self.load_rad

                                path.lineBasicInfo(loadVel1, loadInitDist1, loadDept1, loadAOA1)
                                self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept, curaoa)

                                for b in range(len(Trajectory)):
                                    if Trajectory[b][1] < self.DumpHeight:
                                        del self.cmd_data_list[b:len(Trajectory)]
                                        del Trajectory[b:len(Trajectory)]
                                        break

                                self.LoadingPointLocation.set(f"loading point(dist, dept, aoa) :"
                                                              f"{[round(Trajectory[-1][0], 2), round(Trajectory[-1][1], 2), round(degrees(Trajectory[-1][2]),2)]}")

                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "9":
                                self.swingError = 1.1
                                # 상차(버켓 회전 only)
                                self.exc_seq_MSG.set('상차단계')
                                self.state_log = "9-loading"
                                load_init_dist, load_init_dept = curdist, curdept
                                load_vel = 0.5
                                #load_vel, load_final_aoa = 0.5, radians(-15)  # 1.6

                                lambda_theta =\
                                    np.clip((self.zfar ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(self.zfar ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)

                                if self.loadinglevel > 1:
                                    load_final_aoa = atan2(load_init_dept, self.zfar) - acos(lambda_theta) + radians(40)
                                else:
                                    load_final_aoa = atan2(load_init_dept, self.zfar) - acos(lambda_theta) + radians(10)

                                path.loading_Track_info(load_init_dist, load_init_dept, self.zfar, self.load_rad, load_vel, load_final_aoa)
                                self.cmd_data_list, Trajectory, _ = path.loading_Track(curswing, curdist, curdept, curaoa)
                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "10":
                                self.swingError = 1.1
                                # 패스 or 평탄화
                                if self.loadinglevel == 3 or self.loading_iteration == self.loadingLimit:
                                    self.exc_seq_MSG.set('상차단계 : 평탄화 시작점으로 이동 ')
                                    self.state_log = "10-leveling initiation"
                                    lineVel, LevelingInitDist, LevelingInitDept = 1.1, 8.9, self.DumpHeight - (self.margin_height*0.8) + 0.2

                                    lambda_theta1 = np.clip(
                                        (LevelingInitDist ** 2 + LevelingInitDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LevelingInitDist ** 2 + LevelingInitDept ** 2) * (l2 + l3)), -1, 1)
                                    LevelingInitAOA = atan2(LevelingInitDept, LevelingInitDist) - acos(lambda_theta1)

                                    path.lineBasicInfo(lineVel, LevelingInitDist, LevelingInitDept, LevelingInitAOA)
                                    self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                else:
                                    self.exc_seq_MSG.set('상차단계 : 평탄화 시작점으로 이동 pass ')
                                    self.state_log = "10-pass"
                                    self.cmd_data_list = [self.cmd_data_list[-1]]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "11":
                                self.swingError = 1.1
                                # 패스 or 평탄화
                                if self.loadinglevel == 3 or self.loading_iteration == self.loadingLimit:
                                    self.exc_seq_MSG.set('상차단계 : 트럭에서 평탄화')
                                    self.state_log = "11-leveling"
                                    ####################################################
                                    lineVel, LevelingFinalDist, LevelingFinalDept, LevelingFinalAOA = 0.7, load_init_dist+0.1, \
                                                                                                      self.DumpHeight - (self.margin_height*0.8) + 0.2, radians(-90)
                                    path.lineBasicInfo(lineVel, LevelingFinalDist, LevelingFinalDept, LevelingFinalAOA)
                                    self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept,  curaoa)
                                    ###################################################
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                else:
                                    self.exc_seq_MSG.set('상차단계 : 트럭에서 평탄화 pass')
                                    self.state_log = "11-pass"
                                    self.cmd_data_list = [self.cmd_data_list[-1]]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "12":
                                self.swingError = 1.1
                                if self.loadinglevel == 3 or self.loading_iteration == self.loadingLimit:
                                    self.exc_seq_MSG.set('평탄화 후 오므리기')
                                    self.state_log = "12-bkt fold after leveling"
                                    lineVel, bktFoldDist, bktFoldDept, bktFoldAOA = 1.1, 3.2, 2.2, radians(-165)

                                    path.lineBasicInfo(lineVel, bktFoldDist, bktFoldDept, bktFoldAOA)
                                    self.cmd_data_list, Trajectory, _ = path.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                else:
                                    self.exc_seq_MSG.set('pass')
                                    self.state_log = "12-pass"
                                    self.cmd_data_list = [self.cmd_data_list[-1]]

                                self.prev_auto_state = self.cur_auto_state

                            elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "13":
                                self.swingError = 1.1
                                # 스윙 to 카메라촬영&굴착
                                self.exc_seq_MSG.set('상차단계 : 스윙 to 굴착')
                                self.state_log = "13-swing to exc"
                                if (self.swing_idx == swing_num) and (self.iteration == num_iteration_max):
                                    swingvel, swinggoal = 15, 0  # deg/s, degree
                                else:
                                    swingvel, swinggoal = 25, swing_angle_list[(self.swing_idx) % swing_num]  # deg/s, degree
                                path.swingInfo(swingvel, swinggoal)
                                self.cmd_data_list, Trajectory, _ = path.swingPath(curswing, curdist, curdept, curaoa)
                                curswing = self.cmd_data_list[-1][0]
                                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                self.prev_auto_state = self.cur_auto_state
                            else:
                                pass

                            swing_error = abs(self.cmd_data_list[-1][0]-self.swingangle)
                            boom_error = abs(self.cmd_data_list[-1][1]-self.boomangle)
                            arm_error = abs(self.cmd_data_list[-1][2]-self.armangle)
                            bkt_error = abs(self.cmd_data_list[-1][3]-self.bktangle)
                            error_state = swing_error < self.swingError and boom_error < 3 and arm_error < 3 and bkt_error < 10

                            if error_state or self.errorReset:
                                print("스윙붐암버켓 도착")
                                self.isReach = True
                                #time.sleep(1)
                                self.errorReset = False

                            else:
                                print("스윙붐암버켓 아직 도착 안함")
                                pass
                             
                            if self.isReach:
                                if self.excavation_idx < 13:
                                    print(f"현재 굴착 {self.cur_auto_state} 단계완료")
                                    self.excavation_idx = self.excavation_idx + 1
                                    self.cur_auto_state = cur_auto_state_list[self.excavation_idx]

                                    if self.prev_auto_state == "1" and self.iteration == 1:
                                        self.pcroute = 'C:/Users/cksdu/Desktop/240523/testdata/PointCloud/20240523/CircularSector_Exc.ply'
                                        self.camCapture = True

                                    if self.prev_auto_state == "5":
                                        self.pcroute = 'C:/Users/cksdu/Desktop/240523/testdata/PointCloud/20240523/CircularSector_TruckLoad.ply'
                                        self.camCapture = True

                                    self.isReach = False
                                    
                                else:
                                    if self.loadinglevel == 3 or self.loading_iteration == self.loadingLimit:
                                        # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                        LocalWork = LocalWork + 1
                                        # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                        KU_msg = ku_msg.state(type="squareidx", workingidx=0, msg="Done")
                                        KU_msg = json.dumps(KU_msg)
                                        if TCPIP_TEST:
                                            self.server.send(self.client, KU_msg)
                                        # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                        print("자동굴착 단위 작업 완료 신호 송신, type = squareidx, msg : Done")
                                        self.isReach = False
                                        break

                                    if self.swing_idx == swing_num:
                                        if self.iteration < num_iteration_max: # 단위작업 하나에 1회 굴착완료
                                            if self.iteration == num_iteration_min:
                                                max_idx = swing_angle_list.index(max(swing_angle_list))
                                                swing_angle_list.pop(max_idx)
                                                self.location_distFar_list.pop(max_idx)
                                                min_idx = swing_angle_list.index(min(swing_angle_list))
                                                swing_angle_list.pop(min_idx)
                                                self.location_distFar_list.pop(min_idx)
                                                
                                                swing_num = len(swing_angle_list)
                                            self.iteration += 1
                                            self.excavation_idx = 0
                                            self.cur_auto_state = cur_auto_state_list[self.excavation_idx]
                                        elif self.iteration == num_iteration_max : # 단위작업 하나에 1회 굴착완료
                                            LocalWork = LocalWork + 1
                                            # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                            KU_msg = ku_msg.state(type = "squareidx", workingidx = 0, msg = "Done")
                                            KU_msg = json.dumps(KU_msg)     
                                            if TCPIP_TEST:
                                                self.server.send(self.client, KU_msg)
                                            # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                            print("자동굴착 단위 작업 완료 신호 송신, type = squareidx, msg : Done")
                                            self.isReach = False
                                            break
                                        self.swing_idx = 1

                                    else:
                                        self.swing_idx = self.swing_idx + 1
                                        self.excavation_idx = 0
                                    self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                                  f"current sector index : {self.swing_idx}\n"
                                                                  f"current excavation index : {self.iteration}\n"
                                                                  )
                            else:
                                pass
                                #print('도착하지 않음')
                            time.sleep(0.1)

                    if LocalWork == 2:
                        # 건품연에 자동굴착 전체 작업 완료신호 "Done" 신호 송신
                        KU_msg = ku_msg.state(type = "working", workingidx = 0, msg = "Done")
                        KU_msg = json.dumps(KU_msg)
                        if TCPIP_TEST:
                            self.server.send(self.client, KU_msg)
                        # 건품연에 자동굴착 전체 작업 완료신호 "Done" 신호 송신   

                        print("자동굴착 전체 작업 완료 신호 송신, type = working, msg : Done")
                        print("Waiting으로 변경")
                        self.EXC_state = "Waiting"
                        self.logging_trg = False
                        self.trigger = 0
                        LocalWork = 1

            else:
                print("굴착기 현재값이 아직 도착하지 않았습니다.")
            time.sleep(0.1)
        else:
            print('Path generator 스레드 종료')

    def th_TCPIP_Rx(self, client, data):    
        """
        건품연에서 작업명령을 보내오면 실행되는 함수(데이터 받을 때 마다)
        """
        self.client = client
        print("ROS에서 보낸 데이터 수신!")
        if self.EXC_state == "Waiting":                         # 굴착기가 작업데이터를 대기하는 상태이면 건품연 '작업명령' 수신
            print("data:", data)
            self.ROS_cmd = json.loads(data)                          # 수신된 JSON 데이터를 dict으로 변경
            
            print("작업데이터 수신완료")
            time.sleep(1)
            KU_msg = ku_msg.cmd_resp(type="working", msg="Ready")               # 작업데이터 수신 후 건품연에 'Ready' 신호 송신
            KU_msg = json.dumps(KU_msg)
            
            self.server.send(client, KU_msg)
            print("수신완료 및 굴착준비완료 신호 송신")
            
            self.EXC_state = "Ready"

        elif self.EXC_state == "Ready":                         # 굴착기가 작업 준비 상태이면 건품연으로부터 "Start" 신호 수신
            ROS_cmd = json.loads(data)
            time.sleep(1)
            if ROS_cmd["msg"] == "Start":                       # "Start" 신호 수신 확인 후 exc_state를 working으로 변경
                self.RCV_MSG.set(f'msg : {ROS_cmd["msg"]}')     # UI에 TCPIP msg 표시
                self.trigger = 1
                self.EXC_state = "Working"
                
            else:                                               # 예상 답변이 오지 않는다면 에러 메시지 프린트
                print("메시지 오류 : 예상 답변이 오지 않았습니다.")

        else:                                                   # 굴착기가 작업 준비 상태가 아니면 건품에 "Retransmittion" 신호 송신
            KU_msg = ku_msg.cmd_resp(msg="Retransmittion")
            KU_msg = json.dumps(KU_msg)
            self.server.send(client, KU_msg)
            self.SEND_MSG.set(f'{KU_msg["msg"]}')

            time.sleep(2)
        print('test')

    def th_VirtualCAN(self):
        
        current_data = [0,50,-100,-10]

        upperSwing = unsigned_byte(int(current_data[0]*10)).value >> 8
        lowerSwing = unsigned_byte(int(current_data[0]*10)).value & 0x00ff 
        
        upperBoom = unsigned_byte(int(current_data[1]*10)).value >> 8
        lowerBoom = unsigned_byte(int(current_data[1]*10)).value & 0x00ff

        upperArm = unsigned_byte(int(current_data[2]*10)).value >> 8
        lowerArm = unsigned_byte(int(current_data[2]*10)).value & 0x00ff

        upperBkt = unsigned_byte(int(current_data[3]*10)).value >> 8
        lowerBkt = unsigned_byte(int(current_data[3]*10)).value & 0x00ff

        virtual_excavator_msg = can.Message(arbitration_id=0x01F7, is_extended_id=False, data=[lowerSwing, upperSwing, lowerBoom, upperBoom, lowerArm, upperArm, lowerBkt, upperBkt])
        self.bus_virtual.send(virtual_excavator_msg)
        print("VirtualCAN 스레드 : 1가상의 굴착기 정보 송신")
        time.sleep(0.01)

        while not self.stop_event.is_set():
            
            self.bus_virtual.send(virtual_excavator_msg)
            cmd_data = self.bus_virtual.recv()
            self.conv_recv_cmd(cmd_data)                    # 고려대로부터 받은 데이터를 가상의 현재값으로 변경
            cmd_data = [self.swing_cmd, self.boom_cmd, self.arm_cmd, self.bkt_cmd]
            if cmd_data is not None:
                #print(cmd_data)
                upperSwing = unsigned_byte(int((cmd_data[0]-0.1)*10)).value >> 8
                lowerSwing = unsigned_byte(int((cmd_data[0]-0.1)*10)).value & 0x00ff 
                
                upperBoom = unsigned_byte(int((cmd_data[1]-0.1)*10)).value >> 8
                lowerBoom = unsigned_byte(int((cmd_data[1]-0.1)*10)).value & 0x00ff

                upperArm = unsigned_byte(int((cmd_data[2]-0.1)*10)).value >> 8
                lowerArm = unsigned_byte(int((cmd_data[2]-0.1)*10)).value & 0x00ff

                upperBkt = unsigned_byte(int((cmd_data[3]-0.1)*10)).value >> 8
                lowerBkt = unsigned_byte(int((cmd_data[3]-0.1)*10)).value & 0x00ff

                virtual_excavator_msg = can.Message(arbitration_id=0x01F7, is_extended_id=False, data=[lowerSwing, upperSwing, lowerBoom, upperBoom, lowerArm, upperArm, lowerBkt, upperBkt])
    
            else:
                print("VirtualCAN 스레드 : 고려대에서 명령을 보내지 않습니다.")

        print('VirtualCAN 스레드 종료')

    def th_CanTx(self):
        time.sleep(0.1)
        self.start_path_gen_event.wait()                          # CAN으로부터 굴착기 조인트 각도 수신하면 True
        print("th_CAN_Tx : CAN 데이터를 받기 시작함")
            
        # self.start_path_gen_event.set() # 왜 여기있지? CanRx(self)로 이동함
        self.cmd_data_list = [[self.swingangle, self.boomangle, self.armangle, self.bktangle]]
        prev_cmd_data_list = [[self.swingangle, self.boomangle, self.armangle, self.bktangle]]
        num_list = len(self.cmd_data_list)
        i = 0
        while not self.stop_event.is_set():
            start_time = time.time()
            cmd_data_list = self.cmd_data_list  # 경로생성 loop에서 생성된 cmd_data_list를 cmd_data_list 변수에 저장
            trig_data = self.trigger  # 트리거 신호 0, 1

            if cmd_data_list == prev_cmd_data_list:  # 이전 경로와 같은 경우 하나씩 꺼내서 제어기로 보냄
                if i < num_list:
                    cmd_data = cmd_data_list[i]  # 생성된 경로 리스트 [swing, boom, arm, bucket]
                    i = i + 1
                else:
                    cmd_data = cmd_data_list[num_list - 1]

            else:  # 이전 경로 리스트와 같지 않다면 리셋
                cmd_data = prev_cmd_data_list[-1]
                num_list = len(cmd_data_list)
                i = 0

            cmd_dist, cmd_dept, cmd_aoa = forwardkinematics_real(radians(cmd_data[1]), radians(cmd_data[2]), radians(cmd_data[3]))
            cmd_aoa = degrees(cmd_aoa)

            cur_dist, cur_dept, cur_aoa = forwardkinematics_real(radians(self.boomangle), radians(self.armangle), radians(self.bktangle))
            cur_aoa = degrees(cur_aoa)

            upperSwing = unsigned_byte(int(cmd_data[0] * 10)).value >> 8
            lowerSwing = unsigned_byte(int(cmd_data[0] * 10)).value & 0x00ff

            upperBoom = unsigned_byte(int(cmd_data[1] * 10)).value >> 8
            lowerBoom = unsigned_byte(int(cmd_data[1] * 10)).value & 0x00ff

            upperArm = unsigned_byte(int(cmd_data[2] * 10)).value >> 8
            lowerArm = unsigned_byte(int(cmd_data[2] * 10)).value & 0x00ff

            upperBkt = unsigned_byte(int(cmd_data[3] * 10)).value >> 8
            lowerBkt = unsigned_byte(int(cmd_data[3] * 10)).value & 0x00ff

            cmd_msg = can.Message(arbitration_id=0x01F8, is_extended_id=False, data=[lowerSwing, upperSwing, lowerBoom,
                                                                                     upperBoom, lowerArm, upperArm, lowerBkt, upperBkt])
            trig_msg = can.Message(arbitration_id=0x01F9, is_extended_id=False, data=[trig_data])
            cmd_msg.timestamp = time.time() - start_time
            
            self.bus.send(trig_msg)
            self.bus.send(cmd_msg)
            if self.logging_trg: ## 데이터 로깅 버튼을 통해 On ##
                self.logging_Data = [time.time(), cmd_dist, cmd_dept, cmd_aoa, cmd_data[0], cmd_data[1], cmd_data[2], cmd_data[3],
                                    cur_dist, cur_dept, cur_aoa, self.swingangle, self.boomangle, self.armangle, self.bktangle, self.state_log]
                self.log_deque.appendleft(self.logging_Data)
            else:
                pass
            ### Autobox에 보내는 명령을 UI에 표시 ###
            self.SEND_CMD.set(f'current swing angle : {round(cmd_data[0],2)}\n'
                            f'current boom angle : {round(cmd_data[1],2)}\n'
                            f'current arm angle : {round(cmd_data[2],2)}\n'
                            f'current bucket angle : {round(cmd_data[3],2)}')
            self.EXC_STATE_MSG.set(f'current excavator state : {self.EXC_state}')

            prev_cmd_data_list = cmd_data_list

            end_time = time.time()
            #print("th_CanTx lap time :",end_time-start_time)
            if CAN_TEST:
                sampling_time = 0.1
            else:
                sampling_time = 0.01

            if start_time-end_time <= 0.1:
                time.sleep(sampling_time-(start_time-end_time))
            else:
                "100ms을 지키지 못함"

        trig_msg = can.Message(arbitration_id=0x01F9, is_extended_id=False, data=[0])
        self.bus.send(trig_msg)
        print('CanTx 스레드 종료')
            
    def th_CanRx(self):
        print("CanRx 스레드 : CAN데이터를 기다리고 있습니다.")
        excavator_data = self.bus.recv()                        # Autobox에서 데이터가 넘어올때까지 대기
        print("CanRx 스레드 : 첫 CAN데이터 도착.")
        print("CanRx 스레드 :", excavator_data)
        self.conv_recv_data(excavator_data)                     # Autobox에서 받은 데이터 변환
        self.start_path_gen_event.set()                         # CAN 데이터 수신 후 TCPIP_Tx 스타트 신호
        while not self.stop_event.is_set():
            start_time = time.time()

            try:
                excavator_data = self.bus.recv(1)  # 1초 내로 받지 못하면 Autobox가 데이터를 보내고 있지 않음으로 판단
            except Exception:
                pass
            
            if excavator_data is not None:
                self.conv_recv_data(excavator_data)             # 받은 데이터(joint angle) 단위 및 forward kinematics 변환
                
                ## UI에 현재값 출력 ##
                self.RCV_DATA.set(f'current swing angle : {self.swingangle}\n'
                                    f'current boom angle : {self.boomangle}\n'
                                    f'current arm angle : {self.armangle}\n'
                                    f'current bucket angle : {self.bktangle}')

                curDist, curDept, curAOA = forwardkinematics_real(radians(self.boomangle), radians(self.armangle), radians(self.bktangle))
                curAOA = degrees(curAOA)
                self.currentBKTtipPos.set(f'current Bucket tip pos : {[round(self.swingangle,2), round(curDist,2), round(curDept+2.1,2), round(curAOA,2)]}')

            else:
                print('CanRx 스레드 :Autobox가 데이터를 보내지 않습니다.')
        
        print('CanRx 스레드 종료')

    def th_recordData(self):
        while not self.stop_event.is_set():
            if len(self.log_deque) > 0 and self.logging_trg:
                pathdata = self.log_deque.pop()
                recordInfo = pd.DataFrame(pathdata)
                recordInfo.T.to_csv(self.DataRoute, mode='a', index=False, header=self.headers)
                self.headers = False
            time.sleep(0.01)

    def on_connection_open(self, client):
        # Called by the WebSocket server when a new connection is opened. 한번만 실행
        self.TCP_CONECT.set('Connected!!')

    def cameraPCget(self):
        state = AppState()

        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)

        # Get stream profile and camera intrinsics
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        colorizer = rs.colorizer()

        while not self.stop_event.is_set():
            # 카메라 데이터 확보 관련 코드 start
            if not state.paused:
                frames = pipeline.wait_for_frames()

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                depth_frame = decimate.process(depth_frame)

                # Grab new intrinsics (may be changed by decimation)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w, h = depth_intrinsics.width, depth_intrinsics.height

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_colormap = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data())

                if state.color:
                    mapped_frame, color_source = color_frame, color_image

                points = pc.calculate(depth_frame)
                pc.map_to(mapped_frame)
            # 카메라 데이터 확보 관련 코드 end
            key = cv2.waitKey(2)
            if self.camCapture:
                print("camstate :", self.camCapture)
                points.export_to_ply(self.pcroute, mapped_frame)
                self.camCapture = False
                if os.path.isfile(self.pcroute):
                    year = time.localtime().tm_year
                    month = time.localtime().tm_mon
                    day = time.localtime().tm_mday
                    hour = time.localtime().tm_hour
                    minute = time.localtime().tm_min
                    seconds = time.localtime().tm_sec
                    self.cameraCaptureTime.set(f'Camera Capture Time : {year}.{month}.{day}, {hour}:{minute}:{seconds}')

        pipeline.stop()

    def conv_recv_data(self,recv_data):         ### Autobox에서 받은 데이터 변환 함수
        cuSwingAng = recv_data.data[1] << 8 | recv_data.data[0]
        cuBoomAng = recv_data.data[3] << 8 | recv_data.data[2]
        cuArmAng = recv_data.data[5] << 8 | recv_data.data[4]
        cuBktAng = recv_data.data[7] << 8 | recv_data.data[6]

        currentSwingAngle = signed_byte(cuSwingAng).value
        currentBoomAngle = signed_byte(cuBoomAng).value
        currentArmAngle = signed_byte(cuArmAng).value
        currentBktAngle = signed_byte(cuBktAng).value

        self.swingangle = currentSwingAngle / 10
        self.boomangle = currentBoomAngle / 10
        self.armangle = currentArmAngle / 10
        self.bktangle = currentBktAngle / 10

        self.curdist, self.curdept, self.curaoa = forwardkinematics_real(radians(self.boomangle), radians(self.armangle), radians(self.bktangle))

    def conv_recv_cmd(self,recv_data):         # 가상의 CAN 버스에서 고려대로부터 받은 cmd
        cuSwingAng = recv_data.data[1] << 8 | recv_data.data[0]
        cuBoomAng = recv_data.data[3] << 8 | recv_data.data[2]
        cuArmAng = recv_data.data[5] << 8 | recv_data.data[4]
        cuBktAng = recv_data.data[7] << 8 | recv_data.data[6]

        currentSwingAngle = signed_byte(cuSwingAng).value
        currentBoomAngle = signed_byte(cuBoomAng).value
        currentArmAngle = signed_byte(cuArmAng).value
        currentBktAngle = signed_byte(cuBktAng).value

        self.swing_cmd = currentSwingAngle / 10
        self.boom_cmd = currentBoomAngle / 10
        self.arm_cmd = currentArmAngle / 10
        self.bkt_cmd = currentBktAngle / 10

    def save_current_pos(self):                 # 실행되는 순간의 굴착기 조인트 각도 저장
        return [[self.swingangle, self.boomangle, self.armangle, self.bktangle]]

###################################### UI 관련 ######################################################   
    
    def initialize_GUI(self):

        self.SV_GUI = tk.Tk()
        self.SV_GUI.geometry('1000x800')
        self.SV_GUI.title("KU_Auto_Excavator")

        self.cameraCaptureTime = tk.StringVar()
        self.cameraCaptureTime.set(f'Camera Capture Time : 0000.00.00, 00:00:00')

        self.RCV_DATA = tk.StringVar()
        self.RCV_DATA.set(f'current swing angle : sw0\n'
                            f'current boom angle : bm0\n'
                            f'current arm angle : arm0\n'
                            f'current bucket angle : bkt0')

        self.currentBKTtipPos = tk.StringVar()
        self.currentBKTtipPos.set(f'current Bucket tip pos : None')

        self.SEND_CMD = tk.StringVar()
        self.SEND_CMD.set(f'command swing angle : sw0\n'
                            f'command boom angle : bm0\n'
                            f'command arm angle : arm0\n'
                            f'command bucket angle : bkt0')
        
        self.RCV_MSG = tk.StringVar()
        self.RCV_MSG.set('excavator location')
        
        self.SEND_MSG = tk.StringVar()
        self.SEND_MSG.set('Retransmittion')

        self.EXC_STATE_MSG = tk.StringVar()
        self.EXC_STATE_MSG.set('Initialization')

        self.unit_info_UI_MSG = tk.StringVar()
        self.unit_info_UI_MSG.set(f'distFar : 0\n'
                                  f'distNear : 0\n'
                                  f'designDepth : 0\n'
                                  f'swingAngle : 0')

        self.UnitIDX_UI_MSG = tk.StringVar()
        self.UnitIDX_UI_MSG.set('Unit IDX : 0')

        self.cur_num_exc_UI_MSG = tk.StringVar()
        self.cur_num_exc_UI_MSG.set('cur exc num : 0')

        self.Total_UnitIDX_UI_MSG = tk.StringVar()
        self.Total_UnitIDX_UI_MSG.set('total Unit num : 0')

        self.Total_cur_num_exc_UI_MSG = tk.StringVar()
        self.Total_cur_num_exc_UI_MSG.set('total cur_num_exc : 0')

        self.LoadingTruck4PointLocation = tk.StringVar()
        self.LoadingTruck4PointLocation.set(f'< Truck 4 points(side, height, dist) >\n'
                                            f'1st point : None\n'
                                            f'2nd point : None\n'
                                            f'3rd point : None\n'
                                            f'4th point : None')

        self.LoadingPointLocation = tk.StringVar()
        self.LoadingPointLocation.set(f"loading point(dist, dept, aoa) : None")

        self.exc_seq_MSG = tk.StringVar()
        self.exc_seq_MSG.set('굴착단계 : 대기')

        self.TCP_CONECT = tk.StringVar()
        self.TCP_CONECT.set('연결되지 않았습니다.')

        self.targetDepth_UI_MSG = tk.StringVar()
        self.targetDepth_UI_MSG.set('아직 값 없음')

        Fr = []
        #print(Fr) 
        W = 450
        H = 450
        W_gap = 15
        H_gap = 15                                                                                                          # Frame 리스트
        Fr.append(ttk.LabelFrame(self.SV_GUI,width=100, height=1000, text='CAN 통신',relief=tk.SOLID,labelanchor='n'))
        Fr[0].grid(column=1, row=0, padx=10, pady=10, sticky='n')                                                                          # padx / pady 외부여백
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='TCP/IP 통신',relief=tk.SOLID,labelanchor='n'))
        Fr[1].grid(column=1, row=1, padx=10, pady=10, sticky='n')                                                                         # padx / pady 외부여백
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='EXC_state 설정',relief=tk.SOLID,labelanchor='n'))
        Fr[2].grid(column=1, row=2, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='EXC_state',relief=tk.SOLID,labelanchor='n'))
        Fr[3].grid(column=1, row=3, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='가상테스트',relief=tk.SOLID,labelanchor='n'))
        Fr[4].grid(column=1, row=4, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='종료',relief=tk.SOLID,labelanchor='n'))
        Fr[5].grid(column=1, row=5, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='그래프',relief=tk.SOLID,labelanchor='n'))
        Fr[6].grid(column=1, row=6, padx=10, pady=10, sticky='n')


        self.Rcv1 = ttk.Label(Fr[0],text = '< 수신메시지 from Autobox >')
        self.Rcv1.grid(column=0, row=0, sticky='n', padx=10, pady=5)    
        self.RcvMsg1 = ttk.Label(Fr[0],textvariable = self.RCV_DATA, wraplength=250)
        self.RcvMsg1.grid(column=0, row=1, sticky='n', padx=20, pady=5)

        self.Rcv2 = ttk.Label(Fr[0], text='< 현재 버켓 팁 위치(스윙각(deg), x, y, aoa(deg) >')
        self.Rcv2.grid(column=1, row=0, sticky='n', padx=10, pady=5)
        self.RcvMsg2 = ttk.Label(Fr[0],textvariable = self.currentBKTtipPos, wraplength=250)
        self.RcvMsg2.grid(column=1, row=1, sticky='n', padx=20, pady=5)

        self.Send1 = ttk.Label(Fr[0],text = '< 송신메시지  to  Autobox >')
        self.Send1.grid(column=2, row=0, sticky='n', padx=10, pady=5)
        self.SendMsg1 = ttk.Label(Fr[0],textvariable = self.SEND_CMD, wraplength=250)
        self.SendMsg1.grid(column=2, row=1, sticky='n', padx=20, pady=5)

        self.connection = ttk.Label(Fr[1],text = '< 연결상태 >')
        self.connection.grid(column=0, row=0, sticky='n', padx=10, pady=5)

        self.connection = ttk.Label(Fr[1],textvariable = self.TCP_CONECT, wraplength=250)
        self.connection.grid(column=0, row=1, sticky='n', padx=10, pady=5)

        self.Rcv2 = ttk.Label(Fr[1],text = '< 수신메시지 from ROS >')
        self.Rcv2.grid(column=1, row=0, sticky='n', padx=10, pady=5)

        self.RcvMsg2 = ttk.Label(Fr[1],textvariable = self.RCV_MSG, wraplength=250)
        self.RcvMsg2.grid(column=1, row=1, sticky='n', padx=20, pady=5)

        self.Send2 = ttk.Label(Fr[1],text = '< 송신메시지  to  ROS >')
        self.Send2.grid(column=2, row=0, sticky='n', padx=10, pady=5)

        self.SendMsg2 = ttk.Label(Fr[1],textvariable = self.SEND_MSG, wraplength=250)
        self.SendMsg2.grid(column=2, row=1, sticky='n', padx=20, pady=5)

        self.init_Btn = ttk.Button(Fr[2],text='Init_state',command=self.init_state)
        self.init_Btn.grid(column=0, row=0, sticky='we', padx=10, pady=5)  
        self.ready_Btn = ttk.Button(Fr[2],text='Waiting_state',command=self.waiting_state)
        self.ready_Btn.grid(column=1, row=0, sticky='we', padx=10, pady=5)
        self.ErrorReset_Btn = ttk.Button(Fr[2],text='Error Reset',command=self.error_reset_fn)
        self.ErrorReset_Btn.grid(column=2, row=0, sticky='we', padx=10, pady=5)   
        self.yaw_Btn = ttk.Button(Fr[2],text='logging start',command=self.logging_Start)
        self.yaw_Btn.grid(column=3, row=0, sticky='we', padx=10, pady=5)  
        self.Stop_Btn = ttk.Button(Fr[2],text='logging_stop',command=self.logging_Stop)
        self.Stop_Btn.grid(column=4, row=0, sticky='we', padx=10, pady=5) 
        

        self.Exc_State_UI = ttk.Label(Fr[3],textvariable = self.EXC_STATE_MSG, wraplength=250)
        self.Exc_State_UI.grid(column=0, row=0, sticky='n', padx=20, pady=5)
        self.Exc_State_UI = ttk.Label(Fr[3],textvariable = self.exc_seq_MSG, wraplength=250)
        self.Exc_State_UI.grid(column=2, row=0, sticky='n', padx=20, pady=5)
        self.LocalIDX_UI = ttk.Label(Fr[3],textvariable = self.unit_info_UI_MSG, wraplength=250)
        self.LocalIDX_UI.grid(column=0, row=1, sticky='n', padx=20, pady=5)
        self.targetDepth_UI = ttk.Label(Fr[3], textvariable=self.targetDepth_UI_MSG, wraplength=250)
        self.targetDepth_UI.grid(column=0, row=2, sticky='n', padx=20, pady=5)
        self.UnitIDX_UI = ttk.Label(Fr[3],textvariable = self.UnitIDX_UI_MSG, wraplength=250)
        self.UnitIDX_UI.grid(column=2, row=1, sticky='n', padx=20, pady=5)
        self.cur_num_exc_UI = ttk.Label(Fr[3],textvariable = self.cur_num_exc_UI_MSG, wraplength=250)
        self.cur_num_exc_UI.grid(column=2, row=2, sticky='n', padx=20, pady=5)
        self.Total_UnitIDX_UI = ttk.Label(Fr[3],textvariable = self.Total_UnitIDX_UI_MSG, wraplength=250)
        self.Total_UnitIDX_UI.grid(column=1, row=1, sticky='n', padx=20, pady=5)
        
        self.Total_cur_num_exc_UI = ttk.Label(Fr[3],textvariable = self.Total_cur_num_exc_UI_MSG, wraplength=250)
        self.Total_cur_num_exc_UI.grid(column=1, row=2, sticky='n', padx=20, pady=5)

        self.CameraCaptureTimeDisplay = ttk.Label(Fr[3], textvariable=self.cameraCaptureTime, wraplength=250)
        self.CameraCaptureTimeDisplay.grid(column=3, row=0, sticky='n', padx=20, pady=5)

        self.LoadingTruck4PointLocation_UI = ttk.Label(Fr[3], textvariable=self.LoadingTruck4PointLocation,
                                                       wraplength=250)
        self.LoadingTruck4PointLocation_UI.grid(column=3, row=1, sticky='n', padx=20, pady=5)

        self.LoadingPointLocation_UI = ttk.Label(Fr[3], textvariable=self.LoadingPointLocation, wraplength=250)
        self.LoadingPointLocation_UI.grid(column=3, row=2, sticky='n', padx=20, pady=5)

        self.init_Btn2 = ttk.Button(Fr[4],text='Waiting',command=self.waiting_state)
        self.init_Btn2.grid(column=0, row=0, sticky='n', padx=10, pady=5)
        self.working_Btn = ttk.Button(Fr[4],text='Working',command=self.working_state)
        self.working_Btn.grid(column=1, row=0, sticky='n', padx=10, pady=5)

        self.stop_Btn = ttk.Button(Fr[5],text='프로그램 종료',command=self.stop_program)
        self.stop_Btn.grid(column=0, row=0, sticky='n', padx=10, pady=5)

        self.graph_lbl = ttk.Label(Fr[6])
        self.graph_lbl.grid(row=0, column=0, columnspan=5) 

    def init_state(self):
        self.EXC_state = "Initialization"   # 건품연 작업 명령을 받을 준비가 되지 않았습니다.
       
    def waiting_state(self):
        self.EXC_state = "Waiting"          # 건품연 작업 명령을 받을 준비가 되었습니다.

    def working_state(self):
        self.EXC_state = "Working"          # 건품연 작업 명령을 받을 준비가 되었습니다.

    def logging_Start(self):
        self.logging_trg = True
    
    def logging_Stop(self):
        self.logging_trg = False

    def stop_program(self):
        self.trigger = 0
        self.stop_event.set()

        print('stop')
       
    def error_reset_fn(self):
        self.errorReset = True

###################################### UI 관련 ######################################################   

if __name__ == "__main__":
    AE = Autonomous_Excavator()
    tk.mainloop()

