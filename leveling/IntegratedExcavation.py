## 시나리오 시연 8월 30일 ##
#부체꼴 굴착 관련 
### CAN 관련 ###
import can.interfaces.pcan
#CAN 통신 시 데이터를 16비트로 보내주는 일 
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
import cmd_util  #건품연에서 보내주는 데이터를 받음  굴착영역이랑, 상차위치, 어떤 작업을 할 것인지 
### TCP/IP ###

### UI 관련 ###
import tkinter as tk
from tkinter import ttk
### UI 관련 ###

### AI 경로 생성 관련 ###

from Loaidng_DecisionR7 import *   #상차 위치 결정 함수
from truck_detection_Final_Replicatruck import * #트럭 덤프바디의 4 꼭짓점 위치 예측 함수
### AI 경로 생성 관련 ###

### 카메라 관련 ###
import math
import pyrealsense2 as rs
import os
from pcToValueR3 import *  #지형에 대한 포인트 클라우드에서 지형에 대한 Feature를 얻어내는 함수
import cv2
import shutil
### 카메라 관련 ###

### 경로데이터 저장관련 ###
import pandas as pd
from collections import deque   #데이터 push/pop 연산 및 큐 라이브러리 
### 경로데이터 저장관련 ###

### 시뮬레이션 관련 : 건품연 명령 예시 ###
import virtual_cmdR1    #가상 시뮬레이션 구동 시, 건품연에서 TCP/IP로 보내주는 데이터 관련 함수 
import time
import tensorflow as tf

from inversekinematics_rc_real import * #역기구학, 정기구학 푸는 함수 (RC, 실제굴착기 기준으로 한 코든)
from math import *
import numpy as np

from AIbasedPathGeneration1R18_rule import * 
#################### 잘 안바뀌는 초기값 ####################
### 테스트 모드 선택 ###
CAN_TEST = False            # 전북대와 CAN 통신 활성화 시 True / 가상 CAN 통신으로 코드 테스트 시 False
#이게 우선적으로 참이 되어야 건품연과 같이 있는상황  
TCPIP_TEST = False        # 건품연과 TCPIP 통신 활성화 시 True / TCPIP 통신없이 코드 테스트 시 False (건품연 없이 작동)
#둘다 false면 가상시뮬 
cameraON = False            # 카메라 사용 시 True, 안 사용할 시 False
### 테스트 모드 선택 ###

### KU server IP, PORT ###
#HOST_S = '192.168.110.34's
if TCPIP_TEST:
    HOST_S = '192.168.110.48'   #건품연 임베디드 컴퓨터 아이피 확인해야됨
elif CAN_TEST:              #건품연이 없을때 우리끼리 + 가상시뮬  
    HOST_S = '127.0.0.1'
else:
    HOST_S = '127.0.0.1'
PORT_S = 9999                       
### KU server IP, PORT ###
                                                 
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
                                                                                                          
class Integrated_Autonomous_Excavator():
    def __init__(self, host_IP='127.0.0.1', host_port=9999, ply_save_path = './Data/ply'):
        self.path_generator = ai_excavation_path()
        pos_beforeSwing = [3.2, 2.2, radians(-167)]
        self.path_generator.basicinput(2.9, -2.1, -2.1, -3, 9.3, pos_beforeSwing, 0.8)
        tresh_joint, tresh_Traj, _ = self.path_generator.pathgeneration(10)
        
        self.condition = threading.Condition()
        self.state = 'waiting'
        self.circular_state = False
        self.trenching_state = False 
        self.incline_state = False 
        self.mountain_state = False 

        self.work_type = None 
        #Threading 이벤트 들로 Set여부에 따라 경로생성 함수 작동 여부가 결정됨 
        self.stop_event = threading.Event()
        self.start_path_gen_event = threading.Event()
        
        self.EXC_state = "Initialization" # "Waiting, "Ready", "Working", "Emergency"
        
        self.logging_trg = False #데이터 로깅 트리거/ 데이터 저장 

        # 조인트/버켓 위치 및 aoa 경로 데이터 목록, csv 파일 저장될 시, 헤더 역할
        self.headers = ['time(s)','Dist_cmd(m)', 'Dept_cmd(m)', 'AOA_cmd(Deg)', 'SwingAng_cmd(Deg)', 'BoomAng_cmd(Deg)', 'ArmAng_cmd(Deg)', 'BktAng_cmd(Deg)',
                        'Dist_cur(m)', 'Dept_cur(m)', 'AOA_cur(Deg)', 'SwingAng_cur(Deg)', 'BoomAng_cur(Deg)', 'ArmAng_cur(Deg)', 'BktAng_cur(Deg)',
                        'excavation state'] 

        self.DataRoute = './Data/recordAI_excLoading.csv' #굴삭 기록 저장시 조인트/버켓 위치 및 aoa 경로 데이터 파일(csv) 저장되는 주소

        self.pcroute = './Data/ply/w2Pointcloud_Exc.ply' #Poincloud 파일이 저장되는 주소

        self.log_deque = deque() #데이터 로깅 큐

        self.camCapture = False #카메라 촬영 버튼 누르기(True), 안 누르기(False)
                                #카메라는 항상 켜져있과 그 순간을 캡쳐 하기 위해 
        self.state_log = "준비"  #경로생성 모니터링 창이 열릴 시, 혀재 작업 명칭 Display 하기 위한 변수 
        self.errorReset = False

        self.ply_save_path = ply_save_path

        if not os.path.exists(ply_save_path):
            os.makedirs(self.ply_save_path)
            print(f"{self.ply_save_path}에 폴더가 생성되었습니다")  
        
        else:
            for filename in os.listdir(self.ply_save_path):
                file_path = os.path.join(self.ply_save_path, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    print(f'{file_path}에 있는 폴더를 지우는데 실패하였습니다. Reason: {e}')
            print(f"{self.ply_save_path}에 있는 폴더가 전부 삭제되었습니다") 

        
        ### 전북대에 주는 초기 경로 리스트 및 트리거 ###                    #전북대
        self.cmd_data_list = [[10, 10, -150, -270]]  #CAN 통신 통해 오토박스에 줄 조인트 각도 Control input 
        self.trigger = 0                #CAN 통신 통해 오토박스에 줄 제어 및 구동 시작 여부를 결정해주는 트리거 신호 (안전장치)
        
        self.UI_ON = True
        self.initialize_GUI()
        
        if CAN_TEST:   #self.trigger 도 같이 on이 되어야함 
            ### 실제 실차 테스트 ###
            self.bus = can.interfaces.pcan.pcan.PcanBus(interface='pcan', channel='PCAN_PCIBUS1', receive_own_messages=False, bitrate=500000, f_clock_mhz=80, auto_reset=True)
            RXFilter = [{"can_id": 0x01F7, "can_mask": 0xffff, "extended": False}] #CAN Rx에서 들어오는 데이터들 중 원하는 값만 받기 위함
            self.bus.set_filters(filters=RXFilter)      #조인트 측정값들 

        else:
            ### 가상 CAN 서버(건품연 제어기 없이 시뮬레이션)
            self.bus = can.interface.Bus('test',interface='virtual',receive_own_messages=False)
            RXFilter = [{"can_id": 0x01F7, "can_mask": 0xffff, "extended": False}]
            self.bus.set_filters(filters=RXFilter)
            self.bus_virtual = can.interface.Bus('test',interface='virtual',receive_own_messages=False)
            RXFilter_virtual = [{"can_id": 0x01F8, "can_mask": 0xffff, "extended": False}]
            self.bus_virtual.set_filters(filters=RXFilter_virtual)
            virtual_CAN_th = threading.Thread(target=self.th_VirtualCAN,args=())          # CAN 통신 스레드 : 100ms 마다 데이터 수신 후 joints 명령 송신
            #가상 CAN 통신 스레드 오픈 
            virtual_CAN_th.start()

        if TCPIP_TEST:  #우리가 혼자 할 때에는 이걸 끄면 됨 
            ### 실제 실차 테스트 ###
            #서버 아이피, 서버 포트 정의
            self.S_ip = host_IP
            self.S_port = host_port
            
            print('waiting client ...')         #TCP/IP 통신 설정 
            self.server = WebSocketServer(ip=self.S_ip, port=PORT_S,
                            on_data_receive=self.th_TCPIP_Rx,
                            on_connection_open=self.on_connection_open)
            server_thread = threading.Thread(target=self.server.serve_forever, args=(), daemon=True)
            server_thread.start()
        else:
            #건품연 없이 작업을 할때 하는 작업 , 우리 혼자 있을 때 
            ### tcp/ip 통신 안할 때 ###                 
            #self.ROS_cmd = virtual_cmdR1.work_cmd_trenching()              # 건품연 굴착(트렌칭 굴착)위치 명령 예시 호출
            #가상 시뮬을 하거나 가상시뮬레이션을 할때 미리 저장된 건품연 데이터를 가져옴
            self.ROS_cmd = virtual_cmdR1.work_cmd_SE()                      # 건품연 굴착(부채꼴 굴착)위치 명령 예시 호출
            
        print('tcpip 스레드 시작')
        TCPIP_Tx_th_cir = threading.Thread(target=self.th_TCPIP_Tx_Circular,args=(self.condition,))       
        TCPIP_Tx_th_cir.start()   ##캔통신 테스트시 주석

        TCPIP_Tx_th_inc = threading.Thread(target=self.th_TCPIP_Tx_Incline, args=(self.condition,))        
        TCPIP_Tx_th_inc.start()   ##캔통신 테스트시 주석        

        TCPIP_Tx_th_mount = threading.Thread(target=self.th_TCPIP_Tx_Mountain, args=(self.condition,))        
        TCPIP_Tx_th_mount.start()   ##캔통신 테스트시 주석       

        TCPIP_Tx_th_trench = threading.Thread(target=self.th_TCPIP_Tx_Trenching, args=(self.condition,))        
        TCPIP_Tx_th_trench.start()   ##캔통신 테스트시 주석   

        CanRx_th = threading.Thread(target=self.th_CanRx,args=())          # CAN 통신 스레드 : 10ms 마다 joints 명령 수신
        CanRx_th.start()

        CanTx_th = threading.Thread(target=self.th_CanTx, args=())         # CAN 통신 스레드 : 100ms  joints 명령 송신
        CanTx_th.start()

        record_th = threading.Thread(target=self.th_recordData, args=(self.condition,))   
        record_th.start()

        #state_th = threading.Thread(target=self.state_observer, args=(self.condition,))
        #state_th.start()

        if cameraON:
            cameraPCget_th = threading.Thread(target=self.cameraPCget, args=())
            cameraPCget_th.start()    #포인트 클라우드 얻는 스레드 ON

    ###################################### 쓰레드 함수 ###################################################
    #우리가 가장 많이 수정해야 할 수도 있는 메서드 
    
    def th_TCPIP_Tx_Circular(self, cond):
            print("Circular PATH generator thread start!")
            prev_exc_state = ""
            LocalWork = 1

            #Self.stop_event 의 메서드 is_set() False일 때 작용  True 면 반복문 작동 안함 
            while not self.stop_event.is_set():
                # self.start_path_gen_event 가 set 이면 작동 
                with cond:
                    cond.wait_for(lambda: self.circular_state)

                if self.start_path_gen_event.is_set():                          # CAN으로부터 굴착기 조인트 각도 수신하면 True

                    cur_exc_state = self.EXC_state

                    # self.EXC_state 가 바뀌면 
                    # 뭔가가 바뀌면 트리거를 생성 
                    if prev_exc_state != cur_exc_state:                         # EXC_state가 변화되었는지 확인
                        state_flag = True
                        cur_joint_angle = self.save_current_pos()               # 실행되는 순간의 굴착기 조인트 각도 저장
                        prev_exc_state = cur_exc_state
                    else:
                        state_flag = False

                    if cur_exc_state == "Initialization":                       # 굴삭기 상태 초기화
                        print("부채꼴 초기화 작업 중")
                        #뭔가가 바뀌었을 때 
                        if state_flag:
                            self.cmd_data_list = cur_joint_angle
                            self.trigger = 0 #굴착기가 혹시라도 오작동을 할 수도 있으니 0으로 세팅 
                            self.state_log = "init_state"
                            prev_exc_state = cur_exc_state

                    elif cur_exc_state == "Working":
                        print("자율작업 중")
                        ROS_CMD = self.ROS_cmd

                        ######################### 굴착 작업 정보 획득 #########################
                        self.location = ROS_CMD["location"]  #굴착 영역에 대한 위치정보 (네 꼭지점이 하나의 리스트 안에 들어있음)
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
                        print(self.design_swingAngle_list)                  #대략적인 상치 위치 
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
                        swing_num = num_UnitWork  #굴착 영역의 개수 (ex 부채꼴 굴착의 경우  5개 터파기는 1 개)
                        swing_angle_list = self.design_swingAngle_list
                        ######################### 스윙파라미터 ########################

                        ######################### 횟수 정하는 알고리즘 ################
                        num_iteration_list = []
                        for num in range(num_UnitWork):
                            print(num)
                            print(self.location_distNear_list[num])
                            #굴착영역별 Design Depth 고려, 총 몇번의 굴착을 진행해야 하는지 굴착 횟수 계산 
                            num_iteration = \
                                cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.design_depth_list[num], cur_depth = -2.0)
                            print(num_iteration)
                            num_iteration_list.append(num_iteration)  #각 굴착 영역별 굴착 횟수로 리스트 변수로 만듬
                            #num_iteration = 4
                        print("num interation list:",num_iteration_list) 
                        num_iteration_max = round(max(num_iteration_list),2)  #굴착 횟수 값중 가장 큰 값
                        num_iteration_min = round(min(num_iteration_list),2)  #굴착 횟수 값중 가장 작은 값   
                        ######################## 횟수 정하는 알고리즘 ##################

                        #모인토사, 터파기 등 굴착작업 통합할때 나뉘어야 함
                        if LocalWork == 1:
                            ### 건품연에 자동굴착 전체 작업 시작신호 "Started" 신호 송신 ###
                            KU_msg = ku_msg.state(type = "working", workingidx = 1, msg = "Started")
                            KU_msg = json.dumps(KU_msg)            #작업 전 건품연에 작업 시작한다는 메시지를 보냄(TCP/IP 통신으로)     
                            if TCPIP_TEST:
                                time.sleep(1)
                                self.server.send(self.client, KU_msg)  #설정된 tcp/ip 포트 설정으로 KU msg메시지를 보냄 
                                
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
                            #이것을 어떻게 짜냐에 따라 달라질 수 있음 
                            #법면은 이거로 못함 
                            cur_auto_state_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13"]
                                                                #해당 작업(부채꼴 굴착) 에서 한 번의 굴착&상차 내 프로세스들   
                                                                #굴착 후 상차까지 14개의 구간으로 나누어져 있음
                            ground_depth = -2

                            ######### 중요 ##########
                            #계속 갱신이 되는 경우 
                            self.prev_auto_state = ""
                            self.cur_auto_state = "0"
                            #경로의 마지막 점에 우리가 원하는 영역으로 도달을 했는지 검사
                            self.isReach = False            #현재 경로 프로세스의 끝점에 도달했는지(True) 안 했는지에(False) 관한 변수
                            self.excavation_idx = 0         #경로 프로세스의 index
                            self.iteration = 1              #해당 굴착 영역에서 현재까지의 굴착 횟수 
                            #e.g 부채꼴 굴착 5번 중 현재 몇번째에 있는지
                            self.swing_idx = 1              #현재 굴착영역 index 
                            #이거는 실제 상차를 할 때 주석처리 
                            self.loading_iteration = 0      #현재 상차 횟수 (실제 굴착기 구동시 사용할 것)
                            loading_iter = 0               #실제 상차를 할때 주석을 ON을 해주어야함 
                            self.loadingLimit = 15          #상차 횟수 limit, 임의로 15로 지정함 (보통 10에서 15회 전문가)
                            # 1은 트럭 전방 상차 2는 트럭 후방 부분 상차 3은 평탄화 
                            self.loadinglevel = 1           #상차 모드 (Forward Loading, Rearward Loading, End)
                            #파고나서 버킷 팁 높이를 얼마나 올릴건지 (붐조인트 기준)
                            self.lift_dept = 2.2            
                            self.load_rad = 1.              #상차 경로 중 Curling 반경
                            self.zfar = 8.                  #상차 위치를 결정하기 위한 PointCloud 확보 시, 포인트 들을 포함하는 Limit(제한범위)
                            self.swingError = 1.1           #Desired 와 current의 threashhold 
                            ######### 중요 ###########

                            #UI 층에 뜨는 메시지 
                            self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                            f"current sector index : {self.swing_idx}\n"
                                                            f"current excavation index : {self.iteration}\n"
                                                            )
                            
                            ### unit excvation 완료 시 while문 종료 ###
                            while True and not self.stop_event.is_set():
                                if TCPIP_TEST == False and CAN_TEST == True:
                                    self.trigger = 1   #TCP/IP 통신 없이 고대 단독 실험시 사용하는 IF 루프 
                                self.cur_num_exc_UI_MSG.set(f'current exc num : {self.iteration}')
                                
                                if self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "0":
                                    #한 번의 굴착 & 상차 경로 생성 과정 중 한 프로세스가 중복으로 실행되는 것을 막기 위한 코드 
                                    #다음에 경로 생성이 되는 것을 막기위해 만든 코드
                                    """   
                                    트렌칭 작업의 경우 swing 각이 0도가 아닌경우 swing cmd 생성
                                    타 작업의 경우 초기 위치로 이동하기 위한 swing cmd 생성
                                    """
                                    #굴삭기가 정렬이 되어 있지 않을때 
                                    self.state_log = "0-init swing"
                                    self.swingError = 1.1

                                    #현재의 스윙 각이랑 건품연 데이터 첫번째 굴착 영역에 대한 스윙각도가 일치 여부 
                                    if abs(self.swingangle-swing_angle_list[(self.swing_idx-1)%swing_num]) < 1:
                                        self.cmd_data_list = [self.cmd_data_list[-1]]
                                    else:
                                        self.exc_seq_MSG.set('굴착단계 : 초기 swing 자세 이동')
                                        swingvel, swinggoal = 10, swing_angle_list[(self.swing_idx-1)%swing_num]  # deg/s, degree
                                        self.path_generator.swingInfo(swingvel, swinggoal)
                                        #생성된 경로 저장 
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                        curswing = self.cmd_data_list[-1][0]

                                    self.prev_auto_state = self.cur_auto_state
                                
                                #지형 촬영을 위한 자세로 이동 
                                #해당 굴삭 영역을 처음 촬영 하고 안함 
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "1":
                                    self.swingError = 1.1
                                    if self.iteration == 1 and self.loading_iteration == 0:
                                        self.state_log = "1-capture_position"
                                        self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동')
                                        lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 8.5, 0, radians(-75)
                                        self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                        print("카메라촬영 위치 이동", len(self.cmd_data_list))
                                        curswing = self.cmd_data_list[-1][0]
                                        curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    else:
                                        self.state_log = "1-capture_position pass"
                                        self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동 pass')
                                        pass
                                    self.prev_auto_state = self.cur_auto_state
                                
                                #뎁스 카메라 촬영 후 포인트 클라위드 이용 지형 평균 Depth 계산
                                #포인트 클라우드를 지형 평균 높이로 계산 
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "2":
                                    """
                                    촬영 후 평균값 계산
                                    """
                                    self.swingError = 1.1
                                    ### 촬영 사진 저장 ###
                                    ################################################################################################
                                    if self.iteration == 1 and self.loading_iteration == 0:
                                        self.state_log = "2-calc avg height"
                                        cameraTheta = 40

                                        if cameraON:
                                            self.soil_x_05, self.soil_y_05, _, _ = terrainFeature(self.pcroute, cameraTheta)
                                            self.soilHeightAdjustment = (self.soil_y_05[0] + self.soil_y_05[1] + self.soil_y_05[2] +self.soil_y_05[3]
                                                                    + self.soil_y_05[4] + self.soil_y_05[5] + self.soil_y_05[6])/7
                                        else:
                                            # self.soil_x_05, self.soil_y_05, _, _ = terrainFeature('./Data/ply/w2pointcloud_Exc.ply', cameraTheta)
                                            self.soilHeightAdjustment = -2.1
                                        print("Terrain point cloud generated")

                                    else:
                                        self.state_log = "2-calc avg height pass"
                                        pass
                                    ################################################################################################
                                    print(f"soil height :{self.soilHeightAdjustment}")
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    self.rev_auto_state = self.cur_auto_state
                                
                                #굴착 시작점 정의 후  자세로 이동 (직선경로고 시작 AI경로의 첫번째 점이 계산)
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "3":
                                    print('aa')
                                    self.swingError = 7
                                    self.exc_seq_MSG.set('굴착단계 : 굴착 시작점 이동')
                                    self.state_log = "3-init exc position"
                                    goal_depth = round(self.design_depth_list[(self.swing_idx - 1) % swing_num], 2)
                                    dragLengthList = [2.9, 4.2, 4.2, 3.9, 3.8, 3.8, 3.8, 2.9, 2.9, 2.9]
                                    soildepthList = [-2.1, -2.2, -2.4, -2.5, -2.6, -2.8, -3.0, -3.2, -3.2, -3.2]

                                    #한 구간안에 굴삭을 하는 도중 어디서 시작할지를 계산하는 함수 
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

                                    #굴삭 시작점의 붐조인트 기중 높이가 LineEndDepth  
                    
                                    LineEndDist = initdist
                                    #원래 rulebased 경로를 지면 정보를 이용해서 보정을 하는 항 
                                    LineEndDept = soildepth + (self.soilHeightAdjustment + 2.1)
                                    l1, l2, l3= 6.24565, 3.11364, 1.910051

                                    lambda_theta = np.clip(
                                        (LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                    LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta) + radians(10)
                                    linevel = 1.1
                                    self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA) # 초기위치
                                        #직선경로를 만들기 위해 필요한 정보를 줌 
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    self.prev_auto_state = self.cur_auto_state

                                #굴삭 경로 생성 
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "4":
                                    self.swingError = 1.1
                                    # 굴착 경로 경로 생성
                                    LiftVel = 0.7
                                    if self.iteration == num_iteration_max:
                                        self.exc_seq_MSG.set('굴착단계 : 다듬기작업')
                                        FinalDist, Lift_dist, Lift_dept, Lift_aoa = \
                                            round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2), 3.2, self.lift_dept, radians(-167)
                                        self.cmd_data_list, Trajectory, _ = \
                                            self.path_generator.final_path(curswing, curdist, curdept, curaoa, FinalDist, goal_depth, Lift_dist, Lift_dept, Lift_aoa, LiftVel)
                                        print("다듬기 경로 생성")
                                        self.state_log = "4-Final path"
                                    
                                    elif self.iteration < num_iteration_max:
                                        self.exc_seq_MSG.set('굴착단계 : 굴착경로작업')
                                        pos_beforeSwing = [3.2, self.lift_dept, radians(-167)]
                                        #dragLength 와 soildepth 가 AI 뉴럴 넷에 들어가는 변수   #세번째 항, 다섯번째 항은 만들어진 이차함수 경로를 상황에 맞게 shift   # ㅜ 스윙 전 굴삭기를 어디까지 올릴지 
                                        self.path_generator.basicinput(dragLength, soildepth, soildepth + (self.soilHeightAdjustment + 2.1), goal_depth, initdist, pos_beforeSwing, LiftVel)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.pathgeneration(curswing)
                                        print("AI 경로 생성")
                                        self.state_log = "4-AI path"
                                    
                                    #혹시나 나중에 쓸 수도 있을거 같아서 넣어둠ㅡ
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    self.prev_auto_state = self.cur_auto_state

                                #들어 올리고 상차 영역으로 스윙 (스윙만 함)
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "5":
                                    self.swingError = 1.1
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                    self.state_log = "5-swing to loading"
                                    swingvel, swinggoal = 15, self.DumpSwingAngle  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                #가만히 있고 상차 후 덤프트럭 촬영  #첫 상차와 두번 쨰 상차 시에만 계산  #첫 번쨰는 정확한 상차 위치를 계산 
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "6":
                                # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                    self.state_log = "6-swing to loading"
                                    swingvel, swinggoal = 25, self.DumpSwingAngle  # deg/s, degree
                                    #swingvel, swinggoal = 25, 90
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "7":
                                    # 상차지점으로(직선경로)
                                    self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                                    self.state_log = "7-loading position"

                                    loadVel, loadDist, loadDept, loadAOA = 0.5, self.DumpDist, 0.5, radians(-162)

                                    self.path_generator.lineBasicInfo(loadVel, loadDist, loadDept, loadAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "8":
                                    # 상차(버켓 회전 only)
                                    self.exc_seq_MSG.set('상차단계')
                                    self.state_log = "8-loading"
                                    load_init_dist, load_init_dept = curdist, curdept
                                    load_rad, load_vel = 2.2, 0.7
                                    load_final_dist = 4 + load_rad + 0.2
                                    #load_final_aoa = radians(-30)

                                    lambda_theta = np.clip((load_final_dist ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(load_final_dist ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)
                                    load_final_aoa = atan2(load_init_dept, load_final_dist) - acos(lambda_theta) + radians(40)

                                    self.path_generator.loading_Track_info(load_init_dist, load_init_dept, load_final_dist, load_rad, load_vel, load_final_aoa)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.loading_Track(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.targetDepth_UI_MSG.set(f'Dump dist : {round(load_final_dist, 2)}\n'
                                                                f'Dump depth : {round(load_init_dept, 2)}\n'
                                                                f'Dump swing angle : {round(self.DumpSwingAngle, 2)}\n')

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "9":
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('원위치')
                                    self.state_log = "9-swing before moving"
                                    swingvel, swinggoal = 25, swing_angle_list[(self.swing_idx)%swing_num]  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                else:
                                    pass
                                #생겅된 경로의 끝점에서의 조인트 각도와 측정한 현재 스윙 각도가의 차이 
                                swing_error = abs(self.cmd_data_list[-1][0]-self.swingangle)
                                boom_error = abs(self.cmd_data_list[-1][1]-self.boomangle)
                                arm_error = abs(self.cmd_data_list[-1][2]-self.armangle)
                                bkt_error = abs(self.cmd_data_list[-1][3]-self.bktangle)
                                error_state = swing_error < self.swingError and boom_error < 3 and arm_error < 3 and bkt_error < 10
                                #error_state가 참이면 목표 달성 
                                if error_state or self.errorReset:
                                    print("스윙붐암버켓 도착")
                                    self.isReach = True
                                    #time.sleep(1)
                                    self.errorReset = False

                                else:
                                    print(f"{self.cur_auto_state} 스윙붐암버켓 아직 도착 안함")
                                    pass
                                #도착을 했으면 
                                if self.isReach:
                                    if self.excavation_idx < 13:
                                        print(f"현재 굴착 {self.cur_auto_state} 단계완료")
                                        self.excavation_idx = self.excavation_idx + 1
                                        #현재 굴삭 상태를 업데이트 
                                        self.cur_auto_state = cur_auto_state_list[self.excavation_idx]
                                        
                                        #상태가 1이거나 5일때 포인트 클라우드를 경로에 저장 
                                        if self.prev_auto_state == "1" and self.iteration == 1:
                                            self.pcroute = self.ply_save_path + f'/CircularSector_index{self.swing_idx}_Exc.ply'
                                            self.camCapture = True

                                        if self.prev_auto_state == "5":
                                            self.pcroute = self.ply_save_path + f'/CircularSector_CircularSector_index{self.swing_idx}_Exc.ply'
                                            self.camCapture = True

                                        self.isReach = False

                                    # self.excavation_idx가 13일 때 굴착을 전체 종료할지 아니면 다음 굴삭으로 넘어갈지 
                                    else:
                                        #평탄화가 모두 완료 되었을때 끝내라 굴착 작업이 다 안끝나도 평탄화가 진행됬으면 굴착 전체 프로세스 종료
                                        #이전의 상태를 저장 해 두어야됨 (고쳐야 됨)
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
                                        
                                        #현재 굴착 영역 인덱스 (부체꼴 굴착), 부체꼴 다 찼을 떄 
                                        #현재 스윙 각도가 마지막 굴착 영역 각도일 때 
                                        #피피티 참조 
                                        if self.swing_idx == swing_num:
                                            if self.iteration < num_iteration_max: # 단위작업 하나에 1회 굴착완료
                                                #시나리오가 바뀌면 적용 
                                                #4번과 5번을 제외 4번과 5번을 자름 
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
                                            #5번 영역의 마지막 굴착을 할 때, 다른거 다 끝냄 
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

                                        # 5번이 아닐때 다음 영역으로 넘어가라
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

                        #굴삭 마무리 (LocalWork가 2) 프로그램 굴삭기 중지 
                        if LocalWork == 2:
                            # 건품연에 자동굴착 전체 작업 완료신호 "Done" 신호 송신
                            KU_msg = ku_msg.state(type = "working", workingidx = 0, msg = "Done")
                            KU_msg = json.dumps(KU_msg)
                            if TCPIP_TEST:
                                self.server.send(self.client, KU_msg)
                            # 건품연에 자동굴착 전체 작업 완료신호 "Done" 신호 송신   

                            print("자동굴착 전체 작업 완료 신호 송신, type = working, msg : Done")
                            print("Waiting 으로 변경")
                            self.EXC_state = "Waiting"
                            self.logging_trg = False
                            self.trigger = 0
                            LocalWork = 1

                else:
                    print("굴착기 현재값이 아직 도착하지 않았습니다.")
                time.sleep(0.1)
            else:
                print('Path generator 스레드 종료')

    def th_TCPIP_Tx_Incline(self, cond):
            print("PATH generator thread start!")
            prev_exc_state = ""
            LocalWork = 1
            subcamstate = False

            while not self.stop_event.is_set():
                
                with cond:
                    cond.wait_for(lambda: self.incline_state)

                if self.start_path_gen_event.is_set():                          # CAN으로부터 굴착기 조인트 각도 수신하면 True
                    
                    cur_exc_state = self.EXC_state

                    if prev_exc_state != cur_exc_state:                         # EXC_state가 변화되었는지 확인
                        state_flag = True
                        cur_joint_angle = self.save_current_pos()               # 실행되는 순간의 굴착기 조인트 각도 저장
                        prev_exc_state = cur_exc_state
                    else:
                        state_flag = False

                    if cur_exc_state == "Initialization":
                        print("법면 초기화 작업 중")
                        if state_flag:
                            self.cmd_data_list = cur_joint_angle
                            self.trigger = 0
                            self.state_log = "init_state"
                            prev_exc_state = cur_exc_state

                    elif cur_exc_state == "Working":
                        print("작율작업 중")
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
                            self.location_distFar, self.location_distNear, self.design_depth, self.design_depthFar, self.design_depthNear, self.design_swingAngle = cmd_util.excavation_location(ROS_CMD,i)
                            self.location_distFar_list.append(round(self.location_distFar,2))
                            self.location_distNear_list.append(round(self.location_distNear,2))
                            self.design_depth_list.append(round(self.design_depth,2))
                            self.design_depthFar_list.append(round(self.design_depthFar, 2))
                            self.design_depthNear_list.append(round(self.design_depthNear, 2))
                            self.design_swingAngle_list.append(round(self.design_swingAngle,2))
                        self.DumpDist, self.DumpDepth, self.DumpSwingAngle = cmd_util.dump_location(ROS_CMD)

                        ######################### UI 업데이트 #########################
                        self.unit_info_UI_MSG.set(f'Dist Far : {(self.location_distFar_list)}\n'
                                                f'Dist Near : {(self.location_distNear_list)}\n'
                                                f'Design Depth : {(self.design_depthFar_list)}\n'
                                                f'Swing Angle : {(self.design_swingAngle_list)}')

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
                            num_iteration = cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.design_depth_list[num], cur_depth = -2.0)
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

                            ###################################################################
                            cur_auto_state_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
                            prev_auto_state = ""
                            cur_auto_state = "0"
                            ###################################################################

                            ######### 중요 ###########
                            self.swing_idx = 1
                            isReach = False
                            excavation_idx = 0

                            self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                        f"current sector index : {self.swing_idx}\n"
                                                        f"current excavation index : {self.swing_idx}\n"
                                                        )
                            
                            ### unit excvation 완료 시 while문 종료 ###
                            while True and not self.stop_event.is_set():
                                l1 = 6.24565
                                l2 = 3.11364
                                l3 = 1.910051
                                if TCPIP_TEST == False and CAN_TEST == True:
                                    self.trigger = 1

                                '''
                                for i in range(10):
                                    print('법면 굴착')
                                '''

                                if prev_auto_state != cur_auto_state and cur_auto_state == "0":
                                    self.state_log = "0-DampZone"
                                    self.exc_seq_MSG.set('Pass')
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "1":
                                    self.state_log = "1-ready for excavation"
                                    self.exc_seq_MSG.set('법면 굴착 준비')
                                    lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 3.2, 2.2, radians(-162)
                                    self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "2":
                                    self.state_log = "2-init swing"
                                    self.exc_seq_MSG.set('법면 작영역으로 스윙')
                                    swingvel, swinggoal = 25, swing_angle_list[(self.swing_idx-1)%swing_num] # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]

                                    prev_auto_state = cur_auto_state
                                
                                elif prev_auto_state != cur_auto_state and cur_auto_state == "3":
                                    """
                                    법면 굴착 초기 위치 이동
                                    """
                                    self.exc_seq_MSG.set('법면 굴착 시작점 이동')
                                    self.state_log = "3-init exc position"

                                    ################################## 중요 파라미터 ####################################
                                    LineEndDist, LineEndDept = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2), \
                                                            round(self.design_depthFar_list[(self.swing_idx - 1) % swing_num], 2)

                                    IncExcDist, IncExcDept = round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2), \
                                                            round(self.design_depthNear_list[(self.swing_idx - 1) % swing_num], 2)

                                    IncExcAOA = radians(-140)  # 긁고 지나가는 루트

                                    LineEndDist_orig, LineEndDept_orig = LineEndDist, LineEndDept

                                    for a in range(1,1001):
                                        l2_new = l2 + l3
                                        l_ground = sqrt(LineEndDist ** 2 + LineEndDept ** 2)

                                        Ang_Arm = -(pi - acos(np.clip((l1 ** 2 + l2_new ** 2 - l_ground ** 2) / (2 * l1 * l2_new), -1, 1)))
                                        Ang_Boom = acos(np.clip((l1 ** 2 + l_ground ** 2 - l2_new ** 2) / (2 * l1 * l_ground), -1, 1)) + atan2(LineEndDept, LineEndDist)

                                        Ang_Arm, Ang_Boom = degrees(Ang_Arm), degrees(Ang_Boom)

                                        if Ang_Boom <= 59.6 and Ang_Boom >= 0 and Ang_Arm <= -34.25 and Ang_Arm >= -159.1:
                                            break
                                        else:
                                            LineEndDist = ((1000-a)*LineEndDist_orig + a*IncExcDist)/1000
                                            LineEndDept = ((1000-a)*LineEndDept_orig + a*IncExcDept)/1000

                                    lambda_theta = np.clip((LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                    LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta)  # 긁고 지나가는 루트
                                    ################################## 중요 파라미터 ####################################

                                    linevel = 1.1  # m/s
                                    self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA)  # 초기위치
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "4":
                                    """
                                    AI 경로 생성 or 전문가 경로 생성
                                    """
                                    self.exc_seq_MSG.set('법면 굴착')
                                    self.state_log = "4-Excavation on inclined surface"

                                    IncExcVel = 0.5

                                    self.path_generator.lineBasicInfo(IncExcVel, IncExcDist, IncExcDept, IncExcAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "5":
                                    self.state_log = "5-Lifting"
                                    self.exc_seq_MSG.set('리프팅')
                                    lineVel, camCapDist, camCapDept, camCapAOA = 0.7, 3.2, 2.2, radians(-165)
                                    self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "6":
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                    self.state_log = "6-swing to loading"
                                    swingvel, swinggoal = 25, self.DumpSwingAngle  # deg/s, degree
                                    #swingvel, swinggoal = 25, 90
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "7":
                                    # 상차지점으로(직선경로)
                                    self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                                    self.state_log = "7-loading position"

                                    loadVel, loadDist, loadDept, loadAOA = 0.5, self.DumpDist, 0.5, radians(-162)

                                    self.path_generator.lineBasicInfo(loadVel, loadDist, loadDept, loadAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "8":
                                    # 상차(버켓 회전 only)
                                    self.exc_seq_MSG.set('상차단계')
                                    self.state_log = "8-loading"
                                    load_init_dist, load_init_dept = curdist, curdept
                                    load_rad, load_vel = 2.2, 0.7
                                    load_final_dist = 4 + load_rad + 0.2
                                    #load_final_aoa = radians(-30)

                                    lambda_theta = np.clip((load_final_dist ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(load_final_dist ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)
                                    load_final_aoa = atan2(load_init_dept, load_final_dist) - acos(lambda_theta) + radians(40)

                                    self.path_generator.loading_Track_info(load_init_dist, load_init_dept, load_final_dist, load_rad, load_vel, load_final_aoa)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.loading_Track(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.targetDepth_UI_MSG.set(f'Dump dist : {round(load_final_dist, 2)}\n'
                                                                f'Dump depth : {round(load_init_dept, 2)}\n'
                                                                f'Dump swing angle : {round(self.DumpSwingAngle, 2)}\n')

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "9":
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('원위치')
                                    self.state_log = "9-swing before moving"
                                    swingvel, swinggoal = 25, 0  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                else:
                                    pass

                                swing_error = abs(self.cmd_data_list[-1][0]-self.swingangle)
                                boom_error = abs(self.cmd_data_list[-1][1]-self.boomangle)
                                arm_error = abs(self.cmd_data_list[-1][2]-self.armangle)
                                bkt_error = abs(self.cmd_data_list[-1][3]-self.bktangle)
                                error_state = swing_error < 1.1 and boom_error < 3 and arm_error < 3 and bkt_error < 10

                                if error_state or self.errorReset:
                                    print("스윙붐암버켓 도착")
                                    isReach = True
                                    time.sleep(1)
                                    self.errorReset = False

                                else:
                                    print("스윙붐암버켓 아직 도착 안함")
                                    pass
                                
                                if isReach:
                                    if excavation_idx < 9:
                                        print(f"현재 굴착 {cur_auto_state} 단계완료")
                                        excavation_idx = excavation_idx + 1
                                        cur_auto_state = cur_auto_state_list[excavation_idx]
                                        isReach = False
                                        
                                    else:
                                        LocalWork = LocalWork + 1
                                        # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                        KU_msg = ku_msg.state(type="squareidx", workingidx=0, msg="Done")
                                        KU_msg = json.dumps(KU_msg)
                                        if TCPIP_TEST:
                                            self.server.send(self.client, KU_msg)
                                        # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                        print("자동굴착 단위 작업 완료 신호 송신, type = squareidx, msg : Done")

                                        isReach = False
                                        break

                                        self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                                    f"current sector index : {self.swing_idx}\n"
                                                                    f"current excavation index : {self.swing_idx}\n" )
                                        
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
                            print("Waiting 으로 변경")
                            self.EXC_state = "Waiting"
                            self.logging_trg = False
                            self.trigger = 0
                            LocalWork = 1

                else:
                    print("굴착기 현재값이 아직 도착하지 않았습니다.")
                time.sleep(0.1)
            else:
                print('Path generator 스레드 종료')

    def th_TCPIP_Tx_Mountain(self, cond):
            print("PATH generator thread start!")
            prev_exc_state = ""
            LocalWork = 1
            subcamstate = False

            while not self.stop_event.is_set():
                with cond:
                    cond.wait_for(lambda: self.mountain_state)

                if self.start_path_gen_event.is_set():                          # CAN으로부터 굴착기 조인트 각도 수신하면 True
                    
                    cur_exc_state = self.EXC_state

                    if prev_exc_state != cur_exc_state:                         # EXC_state가 변화되었는지 확인
                        state_flag = True
                        cur_joint_angle = self.save_current_pos()               # 실행되는 순간의 굴착기 조인트 각도 저장
                        prev_exc_state = cur_exc_state
                    else:
                        state_flag = False

                    if cur_exc_state == "Initialization":
                        print("모인토사 초기화 작업 중")
                        if state_flag:
                            self.cmd_data_list = cur_joint_angle
                            self.trigger = 0
                            self.state_log = "init_state"
                            prev_exc_state = cur_exc_state

                    elif cur_exc_state == "Working":
                        print("작율작업 중")
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
                            self.location_distFar, self.location_distNear, self.design_depth, self.design_depthFar, self.design_depthNear, self.design_swingAngle = cmd_util.excavation_location(ROS_CMD,i)
                            #self.design_depth = -3
                            self.location_distFar_list.append(round(self.location_distFar,2))
                            self.location_distNear_list.append(round(self.location_distNear,2))
                            self.design_depth_list.append(round(self.design_depth,2))
                            self.design_depthFar_list.append(round(self.design_depthFar, 2))
                            self.design_depthNear_list.append(round(self.design_depthNear, 2))
                            self.design_swingAngle_list.append(round(self.design_swingAngle,2))
                        print(self.location_distFar_list)
                        print(self.location_distNear_list)  
                        print(self.design_swingAngle_list)      
                        self.DumpDist, self.DumpDepth, self.DumpSwingAngle = cmd_util.dump_location(ROS_CMD)

                        ######################### UI 업데이트 #########################
                        self.unit_info_UI_MSG.set(f'Dist Far : {(self.location_distFar_list)}\n'
                                                f'Dist Near : {(self.location_distNear_list)}\n'
                                                f'Design Depth : {(self.design_depth_list)}\n'
                                                f'Swing Angle : {(self.design_swingAngle_list)}')
                        self.targetDepth_UI_MSG.set(f'Dump dist : {round(self.DumpDist,2)}\n'
                                                f'Dump depth : {round(self.DumpDepth,2)}\n'
                                                f'Dump swing angle : {round(self.DumpSwingAngle,2)}\n')
                        self.Total_UnitIDX_UI_MSG.set(f"total Unit num : {num_UnitWork}")
                        self.UnitIDX_UI_MSG.set(f"current local idx : {LocalWork}")
                        ######################### UI 업데이트 #########################
                        
                        ######################### 스윙파라미터 #########################
                        swing_num = num_UnitWork
                        swing_angle_list1 = self.design_swingAngle_list
                        ######################### 스윙파라미터 ########################
                        ######################### 횟수 정하는 알고리즘 ################
                        num_iteration_list = []
                        for num in range(num_UnitWork):
                            print(num)
                            print(self.location_distNear_list[num])
                            num_iteration = cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.design_depth_list[num], cur_depth = -2.0)
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

                            prev_auto_state = ""
                            ###################################################################
                            cur_auto_state_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"]
                            ###################################################################
                            cur_auto_state = "0"
                            ######### 중요 ###########
                            isReach = False
                            self.loading_iteration = 0
                            excavation_idx = 0
                            self.swing_idx = 1
                            self.loadinglevel = 1
                            loading_iter = 0
                            ######### 중요 ###########
                            sector_idx = 0
                            swing_angle_list2 = [-20, -10, 0, 10, 20,
                                                21, 5, -6, -23]
                            virtual_heightPerSwingAng = [-1.8, -0.8, 0.2, -0.8, -1.8,
                                                        -1.9, -1.9, -1.9, -1.9]
                            sector_excavation_num = 0

                            self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                            f"current sector index : {sector_idx+1}\n"
                                                            f"swing index : {sector_idx+1}\n"
                                                            f"current excavation index : {sector_excavation_num+1}\n"
                                                            )
                            for i in range(10):
                                print('모인토사 굴착')

                            ### unit excvation 완료 시 while문 종료 ###
                            while True and not self.stop_event.is_set():
                                l1 = 6.24565
                                l2 = 3.11364
                                l3 = 1.910051
                                if TCPIP_TEST == False and CAN_TEST == True:
                                    self.trigger = 1
                                self.cur_num_exc_UI_MSG.set(f'current exc num : {sector_excavation_num+1}')
                                
                                if prev_auto_state != cur_auto_state and cur_auto_state == "0":
                                    self.state_log = "0-DampZone"
                                    self.exc_seq_MSG.set('Pass')
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "1":
                                    self.state_log = "1-capture_position"
                                    self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동')
                                    lineVel, camCapDist, camCapDept, camCapAOA = 1, 3.2, 2.2, radians(-162)
                                    self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                                    print("카메라촬영 위치 이동", len(self.cmd_data_list))
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "2":
                                    self.state_log = "2-init swing"

                                    self.exc_seq_MSG.set('굴착단계 : 초기 swing 자세 이동')
                                    swingvel, swinggoal = 20, swing_angle_list1[(self.swing_idx-1)%swing_num] + swing_angle_list2[sector_idx]  # deg/s, degree
                                    # swingvel, swinggoal = 10, self.DumpSwingAngle + swing_angle_list2[sector_idx]  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "3":
                                    ### 촬영 사진 저장 ###
                                    ################################################################################################
                                    if sector_excavation_num == 0 and sector_idx <= 4:
                                        self.state_log = "3-calc soil height"
                                        cameraTheta = 40
                                        if cameraON:
                                            self.soil_x_05, self.soil_y_05, _, _ = terrainFeature(self.pcroute, cameraTheta)
                                            self.soil_mountain_height = self.soil_y_05[0] * 0.05 + self.soil_y_05[1] * 0.15 + \
                                                                        self.soil_y_05[2] * 0.3 + self.soil_y_05[3] * 0.3 + \
                                                                        self.soil_y_05[4] * 0.15 + self.soil_y_05[5] * 0.025 + \
                                                                        self.soil_y_05[6] * 0.025
                                            print(f"soil height : {self.soil_y_05}")
                                            print(f"soil dist : {self.soil_x_05}")
                                        else:
                                            self.soil_mountain_height = virtual_heightPerSwingAng[sector_idx]                               

                                    if sector_idx > 4:
                                        self.state_log = "3-pass"
                                        self.soil_mountain_height = -1.75

                                    #self.soilMountain_heightDisplay.set(f"Soil Mountain Height(m) : {round(self.soil_mountain_height, 2)}")
                                    ################################################################################################

                                    if self.soil_mountain_height <= -1.75:
                                        self.ContinueExc = False

                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "4":
                                    #굴착 초기 위치 이동
                                    if sector_idx <= 4:
                                        self.initdist = 9.2
                                    else:
                                        self.initdist = 9.3

                                    if self.ContinueExc == False:
                                        self.exc_seq_MSG.set('굴착단계 : 다듬기 시작점 이동')
                                        self.state_log = "4-init exc position(Leveling)"
                                        LineEndDist, LineEndDept = self.initdist, -2.0

                                        linevel = 1  # m/s
                                        lambda_theta = np.clip(
                                            (LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                        LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta)

                                        print(linevel, LineEndDist, LineEndDept, LineEndAOA)
                                        self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA)  # 초기위치
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    else:
                                        self.exc_seq_MSG.set('굴착단계 : 굴착 시작점 이동')
                                        self.state_log = "4-init exc position"

                                        LineEndDist, LineEndDept = self.initdist, self.soil_mountain_height

                                        linevel = 1  # m/s
                                        lambda_theta = np.clip(
                                            (LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                        LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta)

                                        print(linevel, LineEndDist, LineEndDept, LineEndAOA)
                                        self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA)  # 초기위치
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    #self.soilMountain_heightDisplay.set(f"Soil Mountain Height(m) : {LineEndDept}")

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "5":
                                    #AI 경로 생성 or 다듬기 경로 생성
                                    goal_depth = -2.05
                                    if self.ContinueExc == False:
                                        FinalDist, Lift_dist, Lift_dept, Lift_aoa, LiftVel = 3.0, 3.2, 2.2, radians(-168), 0.7
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.final_path(curswing, curdist, curdept, curaoa, FinalDist,
                                                                                            goal_depth, Lift_dist, Lift_dept, Lift_aoa, LiftVel)
                                        self.soil_mountain_height = -1.75
                                        print("AI 경로 생성")
                                        self.exc_seq_MSG.set('굴착단계 : 다듬기경로작업(AI)')
                                        self.state_log = "5-Leveling path"
                                    else:
                                        pos_beforeSwing = [3.2, 2.2, radians(-168)]
                                        LiftVel, ExcFinalAOA = 0.7, radians(-150)
                                        self.path_generator.SoilMountainInfo(self.soil_mountain_height, goal_depth, self.initdist, ExcFinalAOA, pos_beforeSwing, LiftVel)
                                        self.cmd_data_list, Trajectory, TipDepth, _ = self.path_generator.SoilMountainExcavation(curswing)
                                        self.soil_mountain_height = min(TipDepth)
                                        print("AI 경로 생성")
                                        self.exc_seq_MSG.set('굴착단계 : 굴착경로작업(AI)')
                                        self.state_log = "5-AI path"
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "6":
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                    self.state_log = "6-swing to loading"
                                    swingvel = 20  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, self.DumpSwingAngle)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    print("스윙 to 상차지점", len(self.cmd_data_list))
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "7":
                                    """  덤프트럭 촬영 """
                                    ### 촬영 사진 저장 ###
                                    ################################################################################################
                                    if self.loading_iteration <= 1:
                                        self.exc_seq_MSG.set('상차단계 : 덤프트럭 촬영')
                                        self.state_log = "7-Truck identification"
                                        if cameraON:
                                        #가상시뮬때 꺼두기
                                            self.truckBodypoints = truck_pointcloud(self.pcroute)
                                        #가상시뮬때 켜두기
                                        else:
                                            self.truckBodypoints = [(-1, -0.5, 10.1), (1, -0.5, 10.1), (-1, -0.5, 3.4), (1, -0.5, 3.4)]
                                            self.truckBodypoints = [(-1, 0.7, 9.3), (1, 0.7, 9.3), (-1, 0.7, 4.6), (1, 0.7, 4.6)]
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                    loadingInit = max(self.truckBodypoints[2][2], self.truckBodypoints[3][2]) + 0.3
                                    if self.loadinglevel == 1:
                                        self.znear = loadingInit + 2 * (8.8 - loadingInit) / 3
                                        self.zfar = 8.8

                                    xleft = min(self.truckBodypoints[2][0], self.truckBodypoints[3][0]) + 0.15
                                    xright = max(self.truckBodypoints[2][0], self.truckBodypoints[3][0]) - 0.15

                                    height_limit = max(self.truckBodypoints[2][1], self.truckBodypoints[3][1]) + 0.5
                                    if cameraON:
                                    ##가상시뮬시 주석##
                                        self.loadinglevel = loading_point2(self.pcroute, cameraTheta, xleft, xright, self.zfar, self.znear, height_limit, self.loadinglevel)
                                    ##가상시뮿시 주석##

                                    else:
                                    ###가상시뮬시 주석 해제###
                                        HeightSoil_Truck = -2 + 0.5*loading_iter
                                        prior_loadinglevel = self.loadinglevel
                                        self.loadinglevel = loading_point2_vir(HeightSoil_Truck, height_limit, self.loadinglevel)

                                        if prior_loadinglevel != self.loadinglevel:
                                            loading_iter = 0
                                    ###가상시뮿시 주석 해제##
                                    if self.loadinglevel == 2:
                                        self.znear = loadingInit + (8.8 - loadingInit) / 3
                                        self.zfar = loadingInit + 2 * (8.8 - loadingInit) / 3
                                    elif self.loadinglevel >= 3:
                                        self.znear = loadingInit
                                        self.zfar = loadingInit + (8.8 - loadingInit) / 3

                                    print(f"loading level : {self.loadinglevel}")
                                    ################################################################################################
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "8":
                                    # 상차지점으로(직선경로)
                                    if self.loading_iteration == 0 :
                                        self.exc_seq_MSG.set('상차단계 : 스윙 조정')
                                        self.state_log = "8-mini alingmnet of swing"

                                        self.margin_height = 1.3

                                        ##################################################
                                        self.DumpHeight, DeltaSwingAng = loading_point(self.truckBodypoints, self.margin_height)
                                        ##################################################

                                        swingvel = 7.5# deg/s, degree
                                        self.DumpSwingAngle = self.DumpSwingAngle - DeltaSwingAng
                                        self.path_generator.swingInfo(swingvel, self.DumpSwingAngle)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                        curswing = self.cmd_data_list[-1][0]
                                        curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    else:
                                        self.exc_seq_MSG.set('상차단계 : 스윙 조정 pass')
                                        self.state_log = "8-Pass"
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                    self.loading_iteration += 1
                                    loading_iter += 1
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "9":
                                    # 상차지점으로(직선경로)
                                    self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                                    self.state_log = "9-loading position1"

                                    loadVel1, loadDept1, loadAOA1 = 0.5, self.DumpHeight, radians(-165)
                                    loadInitDist1 = max(self.truckBodypoints[2][2], self.truckBodypoints[3][2]) + 0.2

                                    self.path_generator.lineBasicInfo(loadVel1, loadInitDist1, loadDept1, loadAOA1)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    loadVel2, loadDept2, loadAOA2 = 0.5, self.DumpHeight - (self.margin_height * 0.2), radians(-160)
                                    loadInitDist2 = loadInitDist1 + 0.1

                                    self.path_generator.lineBasicInfo(loadVel2, loadInitDist2, loadDept2, loadAOA2)
                                    additional_cmd_data_list, additional_Trajectory, _ = self.path_generator.linepath(curswing,
                                                                                                    Trajectory[-1][0],
                                                                                                    Trajectory[-1][1],
                                                                                                    Trajectory[-1][2])

                                    self.LoadingPointLocation.set(f"loading point(dist, dept, aoa) :{[round(loadInitDist2, 2), round(loadDept2, 2), degrees(loadAOA2)]}")

                                    self.cmd_data_list.extend(additional_cmd_data_list)
                                    Trajectory.extend(additional_Trajectory)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "10":
                                    # 상차
                                    self.exc_seq_MSG.set('상차단계')
                                    self.state_log = "10-loading"
                                    load_init_dist, load_init_dept = curdist, curdept
                                    load_rad, load_vel, load_final_aoa = 1.6, 0.8, radians(-15) #1.6

                                    '''swing_angle_list2 = [-20, -10, 0, 10, 20,
                                                        30, 20, 10, 0, -10, -20, -30]'''

                                    if self.loadinglevel <= 2:
                                        load_final_dist = self.zfar
                                    else:
                                        load_rad = 2.2
                                        load_final_dist = load_init_dist + load_rad + 0.2
                                        lambda_theta = np.clip((load_final_dist ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2)/(2 * sqrt(load_final_dist ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)
                                        load_final_aoa = atan2(load_init_dept, load_final_dist) - acos(lambda_theta) + radians(40)

                                    self.path_generator.loading_Track_info(load_init_dist, load_init_dept, load_final_dist, load_rad, load_vel, load_final_aoa)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.loading_Track(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "11":
                                    # 패스 or 평탄화
                                    #if self.loadinglevel == 4 and self.ContinueExc == False:
                                    if sector_idx == len(swing_angle_list2) - 1 and self.ContinueExc == False:
                                        self.exc_seq_MSG.set('상차단계 : 평탄화 시작점으로 이동 ')
                                        self.state_log = "11-leveling initiation"
                                        lineVel, LevelingInitDist, LevelingInitDept = 1, 8.8, self.DumpHeight - self.margin_height - 0.4

                                        lambda_theta1 = np.clip(
                                            (LevelingInitDist ** 2 + LevelingInitDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LevelingInitDist ** 2 + LevelingInitDept ** 2) * (l2 + l3)), -1, 1)
                                        LevelingInitAOA = atan2(LevelingInitDept, LevelingInitDist) - acos(lambda_theta1)

                                        self.path_generator.lineBasicInfo(lineVel, LevelingInitDist, LevelingInitDept, LevelingInitAOA)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                        curswing = self.cmd_data_list[-1][0]
                                        curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    else:
                                        self.exc_seq_MSG.set('상차단계 : 평탄화 시작점으로 이동 pass ')
                                        self.state_log = "11-pass"
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "12":
                                    # 패스 or 평탄화
                                    self.exc_seq_MSG.set('상차단계 : Holding')
                                    self.state_log = "12-Holding"
                                    #if self.loadinglevel == 4 or sector_idx == len(swing_angle_list2)-1 and self.ContinueExc == False:
                                    if sector_idx == len(swing_angle_list2) - 1 and self.ContinueExc == False:
                                        time.sleep(1)
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "13":
                                    # 패스 or 평탄화
                                    #if self.loadinglevel == 4 and self.ContinueExc == False:
                                    if sector_idx == len(swing_angle_list2) - 1 and self.ContinueExc == False:
                                        self.exc_seq_MSG.set('상차단계 : 트럭에서 평탄화')
                                        self.state_log = "13-leveling"
                                        ####################################################
                                        lineVel, LevelingFinalDist, LevelingFinalDept, LevelingFinalAOA = 0.7, load_init_dist+0.5, self.DumpHeight - self.margin_height - 0.4, radians(-90)
                                        self.path_generator.lineBasicInfo(lineVel, LevelingFinalDist, LevelingFinalDept, LevelingFinalAOA)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept,  curaoa)
                                        ###################################################
                                        curswing = self.cmd_data_list[-1][0]
                                        curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    else:
                                        self.exc_seq_MSG.set('상차단계 : 트럭에서 평탄화 pass')
                                        self.state_log = "13-pass"
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                    prev_auto_state = cur_auto_state

                                elif prev_auto_state != cur_auto_state and cur_auto_state == "14":
                                    self.exc_seq_MSG.set('상차단계 : 스윙 전 오므리기')
                                    self.state_log = "14-loading to bkt fold"
                                    lineVel, bktFoldDist, bktFoldDept, bktFoldAOA = 1.1, 3.2, 2.2, radians(-165)

                                    self.path_generator.lineBasicInfo(lineVel, bktFoldDist, bktFoldDept, bktFoldAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    #
                                    # 0621일 수정내용
                                    # if sector_idx == len(swing_angle_list2) - 1 and self.ContinueExc == False:
                                    #     swingvel, swinggoal = 20, 0 # 스윙 0으로 이동
                                    #     # swingvel, swinggoal = 10, self.DumpSwingAngle + swing_angle_list2[sector_idx]  # deg/s, degree
                                    #     self.path_generator.swingInfo(swingvel, swinggoal)
                                    #     cmd_data_temp, Trajectory, _ = self.path_generator.swingPath(curswing, curdist,
                                    #                                                                       curdept, curaoa)
                                    #     self.cmd_data_list.append(cmd_data_temp)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    prev_auto_state = cur_auto_state

                                else:
                                    pass

                                swing_error = abs(self.cmd_data_list[-1][0]-self.swingangle)
                                boom_error = abs(self.cmd_data_list[-1][1]-self.boomangle)
                                arm_error = abs(self.cmd_data_list[-1][2]-self.armangle)
                                bkt_error = abs(self.cmd_data_list[-1][3]-self.bktangle)
                                error_state = swing_error < 1.1 and boom_error < 3 and arm_error < 3 and bkt_error < 10

                                if error_state or self.errorReset:
                                    print("스윙붐암버켓 도착")
                                    isReach = True
                                    time.sleep(1)
                                    self.errorReset = False
                                    
                                else:
                                    print("스윙붐암버켓 아직 도착 안함")
                                    pass
                                
                                if isReach:
                                    if excavation_idx < 14:
                                        print(f"현재 굴착 {cur_auto_state} 단계완료")
                                        excavation_idx = excavation_idx + 1
                                        cur_auto_state = cur_auto_state_list[excavation_idx]
                                        if prev_auto_state == "2" and sector_excavation_num == 0 and sector_idx <=4:
                                            self.pcroute = self.ply_save_path + f'/w2pointcloud_soilMountainTruck1_{sector_idx+1}sw_{sector_excavation_num+1}exc.ply'
                                            self.camCapture = True

                                        if prev_auto_state == "6":
                                            self.pcroute = self.ply_save_path + '/w2pointcloud_SoilMountainExcTruck1Load.ply'
                                            #self.pcroute ='C:/Users/HJM/Desktop/yoloexam/pythonProject/pointcloud/PCtest_231024/w2pointcloud_SoilMountainExcTruckLoad.ply'
                                            self.camCapture = True

                                        isReach = False
                                        
                                    else:
                                        sector_excavation_num += 1
                                        if sector_idx == len(swing_angle_list2)-1:
                                            if self.ContinueExc: # 단위작업 하나에 1회 굴착완료
                                                excavation_idx = 0
                                                cur_auto_state = cur_auto_state_list[excavation_idx]
                                            elif self.ContinueExc == False: # 단위작업 하나에 1회 굴착완료
                                                LocalWork = LocalWork + 1
                                                # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                                KU_msg = ku_msg.state(type = "squareidx", workingidx = 0, msg = "Done")
                                                KU_msg = json.dumps(KU_msg)     
                                                if TCPIP_TEST:
                                                    self.server.send(self.client, KU_msg)
                                                # 건품연에 자동굴착 단위 작업 완료신호 "Done" 신호 송신
                                                print("자동굴착 단위 작업 완료 신호 송신, type = squareidx, msg : Done")
                                                
                                                isReach = False
                                                break
                                        else:
                                            excavation_idx = 0
                                            cur_auto_state = cur_auto_state_list[excavation_idx]
                                            if self.ContinueExc == False:
                                                sector_idx += 1
                                                sector_excavation_num = 0
                                                self.ContinueExc = True
                                        self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                                    f"current sector index : {sector_idx+1}\n"
                                                                    f"swing index : {sector_idx+1}\n"
                                                                    f"current excavation index : {sector_excavation_num+1}\n"
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
                            print("Initialization 으로 변경")
                            self.EXC_state = "Waiting"
                            self.logging_trg = False
                            self.trigger = 0
                            LocalWork = 1

                else:
                    print("굴착기 현재값이 아직 도착하지 않았습니다.")
                time.sleep(0.1)
            else:
                print('Path generator 스레드 종료')

    def th_TCPIP_Tx_Trenching(self, cond):
            print("PATH generator thread start!")
            prev_exc_state = ""
            LocalWork = 1

            while not self.stop_event.is_set():
                with cond:
                    cond.wait_for(lambda: self.trenching_state)
                
                if self.start_path_gen_event.is_set():                          # CAN으로부터 굴착기 조인트 각도 수신하면 True

                    cur_exc_state = self.EXC_state

                    if prev_exc_state != cur_exc_state:                         # EXC_state가 변화되었는지 확인
                        state_flag = True
                        cur_joint_angle = self.save_current_pos()               # 실행되는 순간의 굴착기 조인트 각도 저장
                        prev_exc_state = cur_exc_state
                    else:
                        state_flag = False

                    if cur_exc_state == "Initialization":
                        print("터파기 초기화 작업 중")
                        if state_flag:
                            self.cmd_data_list = cur_joint_angle
                            self.trigger = 0
                            self.state_log = "init_state"
                            prev_exc_state = cur_exc_state

                    elif cur_exc_state == "Working":
                        print("작율작업 중")
                        ROS_CMD = self.ROS_cmd

                        ######################### 굴착 작업 정보 획득 #########################
                        self.location = ROS_CMD["location"] #건품연에서 준 위치 지령 
                        print(self.location)
                        num_UnitWork = len(self.location)
                        self.location_distFar_list = []
                        self.location_distNear_list = []
                        self.design_depth_list = []
                        self.design_depthFar_list = []
                        self.design_depthNear_list = []
                        self.design_swingAngle_list = []

                        for i in range(num_UnitWork):  
                            self.location_distFar, self.location_distNear, self.design_depth, self.design_depthFar, self.design_depthNear, self.design_swingAngle = cmd_util.excavation_location(ROS_CMD,i)
                            #self.design_depth = -3.5
                            self.location_distFar_list.append(round(self.location_distFar,2))
                            self.location_distNear_list.append(round(self.location_distNear,2))
                            self.design_depth_list.append(round(self.design_depth,2))
                            self.design_depthFar_list.append(round(self.design_depthFar, 2))
                            self.design_depthNear_list.append(round(self.design_depthNear, 2))
                            self.design_swingAngle_list.append(round(self.design_swingAngle,2))
                        print(self.location_distFar_list)
                        print(self.location_distNear_list)  
                        print(self.design_swingAngle_list)      
                        self.DumpDist, self.DumpDepth, self.DumpSwingAngle = cmd_util.dump_location(ROS_CMD)
                        #self.design_depth_list = [-3.5, -3.5, -3.5, -3.5, -3.5]
                        ######################### 굴착 작업 정보 획득 #########################
                        
                        ######################### UI 업데이트 #########################
                        self.unit_info_UI_MSG.set(f'Dist Far : {(self.location_distFar_list)}\n'
                                                f'Dist Near : {(self.location_distNear_list)}\n'
                                                f'Design Depth : {(self.design_depth_list)}\n'
                                                f'Swing Angle : {(self.design_swingAngle_list)}')
                        self.targetDepth_UI_MSG.set(f'Dump dist : {round(self.DumpDist,2)}\n'
                                                f'Dump depth : {round(self.DumpDepth,2)}\n'
                                                f'Dump swing angle : {round(self.DumpSwingAngle,2)}\n')
                        self.Total_UnitIDX_UI_MSG.set(f"total Unit num : {num_UnitWork}")
                        self.UnitIDX_UI_MSG.set(f"current local idx : {LocalWork}")
                        ######################### UI 업데이트 #########################
                        
                        ######################### 스윙파라미터 #########################
                        swing_num = num_UnitWork
                        # swing_num = 5
                        # angle_unitsector = 8
                        # self.design_swingAngle = -(floor(swing_num/2))*angle_unitsector
                        # init_swingAngle = -(floor(swing_num/2))*angle_unitsector
                        # swing_angle_list = []
                        # for i in range(0,swing_num):
                        #    swing_angle_list.append(init_swingAngle)
                        #    init_swingAngle = init_swingAngle + 10
                        swing_angle_list = self.design_swingAngle_list
                        ######################### 스윙파라미터 ########################
                        ######################### 횟수 정하는 알고리즘 ################
                        num_iteration_list = []
                        for num in range(num_UnitWork):
                            print(num)
                            print(self.location_distNear_list[num])
                            num_iteration = cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.design_depth_list[num], cur_depth = -2.0)
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
                            cur_auto_state_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"]

                            ground_depth = -2
                            self.loadingLimit = 16

                            ######### 중요 ##########
                            self.prev_auto_state = ""
                            self.cur_auto_state = "0"
                            self.isReach = False
                            self.excavation_idx = 0
                            self.loading_iteration = 0
                            self.iteration = 1
                            self.swing_idx = 1
                            ######### 중요 ###########

                            self.Total_UnitIDX_UI_MSG.set(f"total sector number : {num_UnitWork}\n"
                                                            f"current sector index : {self.swing_idx}\n"
                                                            f"swing index : {self.swing_idx}\n"
                                                            f"current excavation index : {self.iteration}\n"
                                                            )
                            for i in range(10):
                                print('터파기 굴착')
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

                                    if abs(self.swingangle-swing_angle_list[(self.swing_idx-1)%swing_num]) < 1:
                                        self.cmd_data_list = [self.cmd_data_list[-1]]
                                    else:
                                        self.exc_seq_MSG.set('굴착단계 : 초기 swing 자세 이동')
                                        swingvel, swinggoal = 10, swing_angle_list[(self.swing_idx-1)%swing_num]  # deg/s, degree
                                        self.path_generator.swingInfo(swingvel, swinggoal)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                        #print(Trajectory)
                                        curswing = self.cmd_data_list[-1][0]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "1":
                                    """
                                    지형 촬영 초기 자세 이동
                                    """
                                    if self.iteration == 1:
                                        self.state_log = "1-capture_position"
                                        self.exc_seq_MSG.set('굴착단계 : 지형 촬영 자세 이동')
                                        lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 8.5, 0, radians(-75)
                                        self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
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
                                    print("지형 사진 찰칵, point cloud generated")
                                    print("point cloud generated")
                                    ### 촬영 사진 저장 ###
                                    ################################################################################################
                                    if self.iteration == 1:
                                        self.state_log = "2-calc avg height"
                                        cameraTheta = 40
                                        if cameraON:
                                            self.soil_x_05, self.soil_y_05, _, _ = terrainFeature('./Data/ply/w2Pointcloud_TrenchingExc.ply', cameraTheta)

                                            self.soilHeightAdjustment = (self.soil_y_05[0] + self.soil_y_05[1] + self.soil_y_05[2] +self.soil_y_05[3]
                                                                    + self.soil_y_05[4] + self.soil_y_05[5] + self.soil_y_05[6])/7
                                        else:
                                            self.soilHeightAdjustment = -2.1

                                    else:
                                        self.state_log = "2-calc avg height pass"
                                        pass
                                    ################################################################################################
                                    print(f"soil height :{self.soilHeightAdjustment}")
                                    self.cmd_data_list = [self.cmd_data_list[-1]]
                                    self.rev_auto_state = self.cur_auto_state
                                
                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "3":
                                    """
                                    굴착 초기 위치 이동
                                    """
                                    self.exc_seq_MSG.set('굴착단계 : 굴착 시작점 이동')
                                    self.state_log = "3-init exc position"
                                    goal_distance = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num],2) - round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2)
                                    goal_depth = round(self.design_depth_list[(self.swing_idx - 1) % swing_num], 2)
                                    print("goal_distance:", goal_distance, "\ngoal_depth :", goal_depth)

                                    dragLengthList = [3.0, 4.2, 4.2, 3.9, 3.8, 3.8, 3.8, 2.9, 2.9, 2.9]
                                    soildepthList = [-2.1, -2.2, -2.4, -2.5, -2.6, -2.8, -3.0, -3.2, -3.2, -3.2]

                                    if self.iteration % 2 == 1 and self.iteration < num_iteration_list[(self.swing_idx - 1) % swing_num]:
                                        # 시작점#
                                        dragLength = dragLengthList[self.iteration - 1]
                                        soildepth = soildepthList[self.iteration - 1]
                                        initdist = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2) + (goal_depth - soildepth) * 0.5 / (goal_depth - ground_depth)
                                        LiftVel = 0.7
                                    elif self.iteration % 2 == 0 and self.iteration < num_iteration_list[(self.swing_idx - 1) % swing_num]:
                                        dragLength = dragLengthList[self.iteration - 1]
                                        soildepth = soildepthList[self.iteration - 1]
                                        initdist = round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2) - ((goal_depth - soildepth) * 0.5 / (goal_depth - ground_depth)) + dragLength
                                        LiftVel = 0.7
                                    elif self.iteration == num_iteration_list[(self.swing_idx - 1) % swing_num]:
                                        soildepth = goal_depth
                                        initdist = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2)
                                        LiftVel = 0.7
                                        # 굴착경로 파라미터#

                                    LineEndDist = initdist
                                    LineEndDept = soildepth + (self.soilHeightAdjustment + 2.1)
                                    l1 = 6.24565
                                    l2 = 3.11364
                                    l3 = 1.910051

                                    lambda_theta = np.clip(
                                        (LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
                                    LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta) + radians(10)
                                    linevel = 1

                                    self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA) # 초기위치
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "4":
                                    """
                                    AI 경로 생성 or 전문가 경로 생성
                                    """
                                    if self.iteration == num_iteration_max:
                                        self.exc_seq_MSG.set('굴착단계 : 다듬기작업')
                                        FinalDist, Lift_dist, Lift_dept, Lift_aoa = round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2), 3.2, 2.5, radians(-170)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.final_path(curswing, curdist, curdept, curaoa, FinalDist, goal_depth, Lift_dist, Lift_dept, Lift_aoa, LiftVel)
                                        print("다듬기 경로 생성")
                                        self.state_log = "4-Final path"

                                    elif self.iteration < num_iteration_max:
                                        self.exc_seq_MSG.set('굴착단계 : 굴착경로작업')
                                        pos_beforeSwing = [3.2, 2.5, radians(-170)]
                                        self.path_generator.basicinput(dragLength, soildepth, soildepth + (self.soilHeightAdjustment + 2.1), goal_depth, initdist, pos_beforeSwing, LiftVel)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.pathgeneration(curswing)
                                        print("AI 경로 생성")
                                        self.state_log = "4-AI path"

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "5":
                                    # 스윙 to 상차지점
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                                    self.state_log = "5-swing to loading"
                                    swingvel, swinggoal = 15, self.DumpSwingAngle  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "6":
                                    """  덤프트럭 촬영 """
                                    ### 촬영 사진 저장 ###
                                    ################################################################################################
                                    if self.loading_iteration == 0:
                                        self.exc_seq_MSG.set('상차단계 : 덤프트럭 촬영')
                                        self.state_log = "6-Truck identification"
                                        if cameraON:
                                        #가상 시뮬시 꺼놔야됨
                                            self.truckBodypoints = truck_pointcloud(self.pcroute)
                                            print(f"truck points : {self.truckBodypoints}")
                                        #가상 시뮬시 켜놔야됨
                                        else:
                                            self.truckBodypoints = [(-1, -0.5, 10.1), (1, -0.5, 10.1), (-1, -0.5, 3.4), (1, -0.5, 3.4)]
                                            self.truckBodypoints = [(-1, 0.7, 9.3), (1, 0.7, 9.3), (-1, 0.7, 4.6), (1, 0.7, 4.6)]
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                        self.LoadingTruck4PointLocation.set(f"1st point : {self.truckBodypoints[0]}\n"
                                                                            f"2nd point : {self.truckBodypoints[1]}\n"
                                                                            f"3rd point : {self.truckBodypoints[2]}\n"
                                                                            f"4th point : {self.truckBodypoints[3]}\n"
                                                                            )
                                    else:
                                        self.exc_seq_MSG.set('상차단계 : 덤프트럭 촬영 pass')
                                        self.state_log = "6-Pass"
                                        self.cmd_data_list = [self.cmd_data_list[-1]]
                                    ################################################################################################
                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "7":
                                    # 상차지점으로(직선경로)
                                    if self.loading_iteration == 0:
                                        self.exc_seq_MSG.set('상차단계 : 스윙 조정')
                                        self.state_log = "7-mini alingmnet of swing"

                                        self.margin_height = 1.3

                                        ##################################################
                                        self.DumpHeight, DeltaSwingAng = loading_point(self.truckBodypoints, self.margin_height)
                                        ##################################################

                                        swingvel = 7.5  # deg/s, degree
                                        self.DumpSwingAngle = self.DumpSwingAngle - DeltaSwingAng
                                        self.path_generator.swingInfo(swingvel, self.DumpSwingAngle)
                                        self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                        curswing = self.cmd_data_list[-1][0]
                                        curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]
                                    else:
                                        self.exc_seq_MSG.set('상차단계 : 스윙 조정 pass')
                                        self.state_log = "7-Pass"
                                        self.cmd_data_list = [self.cmd_data_list[-1]]

                                    self.loading_iteration += 1
                                    print(f"loading iteration : {self.loading_iteration}")
                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "8":
                                    self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                                    self.state_log = "8-loading position1"

                                    loadVel1, loadDept1, loadAOA1 = 0.5, self.DumpHeight, radians(-165)
                                    loadInitDist1 = max(self.truckBodypoints[2][2], self.truckBodypoints[3][2]) + 0.2

                                    self.path_generator.lineBasicInfo(loadVel1, loadInitDist1, loadDept1, loadAOA1)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    loadVel2, loadDept2, loadAOA2 = 0.5, self.DumpHeight - (self.margin_height * 0.1), radians(-160)
                                    loadInitDist2 = loadInitDist1 + 0.1

                                    self.path_generator.lineBasicInfo(loadVel2, loadInitDist2, loadDept2, loadAOA2)
                                    additional_cmd_data_list, additional_Trajectory, _ = self.path_generator.linepath(curswing,
                                                                                                    Trajectory[-1][0],
                                                                                                    Trajectory[-1][1],
                                                                                                    Trajectory[-1][2])

                                    self.cmd_data_list.extend(additional_cmd_data_list)
                                    Trajectory.extend(additional_Trajectory)

                                    self.LoadingPointLocation.set(f"loading point(dist, dept, aoa) :{[round(loadInitDist2, 2), round(loadDept2, 2), degrees(loadAOA2)]}")

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "9":
                                    # 상차(버켓 회전 only)
                                    self.exc_seq_MSG.set('상차단계')
                                    self.state_log = "9-loading"
                                    load_init_dist, load_init_dept = curdist, curdept
                                    load_rad, load_vel, load_final_aoa = 1.6, 0.6, radians(-15)

                                    if self.loading_iteration <= 6:
                                        load_final_dist = 8.8
                                    elif self.loading_iteration > 6 and self.loading_iteration <= 8:
                                        load_final_dist = (8.8 + load_init_dist + load_rad + 0.5) / 2
                                    else:
                                        load_rad = 2.2
                                        load_final_dist = load_init_dist + load_rad + 0.5
                                        lambda_theta = np.clip((load_final_dist ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(load_final_dist ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)
                                        load_final_aoa = atan2(load_init_dept, load_final_dist) - acos(lambda_theta) + radians(40)

                                    self.path_generator.loading_Track_info(load_init_dist, load_init_dept, load_final_dist, load_rad, load_vel, load_final_aoa)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.loading_Track(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "10":
                                    self.exc_seq_MSG.set('상차단계 : 스윙 전 오므리기')
                                    self.state_log = "10-loading to bkt fold"
                                    lineVel, bktFoldDist, bktFoldDept, bktFoldAOA = 1.1, 3.2, 2.5, radians(-165)

                                    self.path_generator.lineBasicInfo(lineVel, bktFoldDist, bktFoldDept, bktFoldAOA)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)

                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state

                                elif self.prev_auto_state != self.cur_auto_state and self.cur_auto_state == "11":
                                    # 스윙 to 카메라촬영&굴착
                                    self.exc_seq_MSG.set('상차단계 : 스윙 to 굴착')
                                    self.state_log = "11-swing to exc"
                                    if (self.swing_idx == swing_num) and (self.iteration == num_iteration_max):
                                        swingvel, swinggoal = 25, 0  # deg/s, degree
                                    else:
                                        swingvel, swinggoal = 25, swing_angle_list[(self.swing_idx) % swing_num]  # deg/s, degree
                                    self.path_generator.swingInfo(swingvel, swinggoal)
                                    self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                                    curswing = self.cmd_data_list[-1][0]
                                    curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                                    self.prev_auto_state = self.cur_auto_state
                                else:
                                    pass

                                swing_error = abs(self.cmd_data_list[-1][0]-self.swingangle)
                                boom_error = abs(self.cmd_data_list[-1][1]-self.boomangle)
                                arm_error = abs(self.cmd_data_list[-1][2]-self.armangle)
                                bkt_error = abs(self.cmd_data_list[-1][3]-self.bktangle)
                                error_state = swing_error < 1.1 and boom_error < 3 and arm_error < 3 and bkt_error < 10

                                if error_state or self.errorReset:
                                    print("스윙붐암버켓 도착")
                                    self.isReach = True
                                    time.sleep(1)
                                    self.errorReset = False

                                else:
                                    print("스윙붐암버켓 아직 도착 안함")
                                    pass
                                
                                if self.isReach:
                                    if self.excavation_idx < 11:
                                        print(f"현재 굴착 {self.cur_auto_state} 단계완료")
                                        self.excavation_idx = self.excavation_idx + 1
                                        self.cur_auto_state = cur_auto_state_list[self.excavation_idx]

                                        if self.prev_auto_state == "1" and self.iteration == 1:
                                            self.pcroute = self.ply_save_path + '/w2pointcloud_TrenchingExc.ply'
                                            #self.pcroute = 'C:/Users/HJM/Desktop/yoloexam/pythonProject/pointcloud/PCtest_231024/w2pointcloud_Exc.ply'
                                            self.camCapture = True

                                        if self.prev_auto_state == "5" and self.loading_iteration == 0:
                                            self.pcroute = self.ply_save_path + '/w2pointcloud_TrenchingTruck.ply'
                                            #self.pcroute ='C:/Users/HJM/Desktop/yoloexam/pythonProject/pointcloud/PCtest_231024/w2pointcloud_ExcTruckLoad.ply'
                                            self.camCapture = True

                                        self.isReach = False
                                        
                                    else:

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
                                                #self.camCapture = True ########문의
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
                                                                    f"swing index : {self.swing_idx}\n"
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
                            print("Waiting 으로 변경")
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
            
            if self.ROS_cmd["type"] == 'workorder1':
                self.change_state('incline')

            elif self.ROS_cmd["type"] == 'workorder2':
                self.change_state('circular')

            elif self.ROS_cmd["type"] == 'workorder3':
                self.change_state('mountain')
            
            elif self.ROS_cmd["type"] == 'workorder4':
                self.change_state('trenching')

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

    #위치 지령을 보내주는 함수 
    def th_CanTx(self):
        time.sleep(0.1)
        self.start_path_gen_event.wait()                          # CAN으로부터 굴착기 조인트 각도 수신하면 True
        print("th_CAN_Tx : CAN 데이터를 받기 시작함")
            
        # self.start_path_gen_event.set() # 왜 여기있지? CanRx(self)로 이동함
        # 굴착기가 현재 상태를 유지하기 위해 현재 센서 값을 보내 줌 
        self.cmd_data_list = [[self.swingangle, self.boomangle, self.armangle, self.bktangle]]
        prev_cmd_data_list = [[self.swingangle, self.boomangle, self.armangle, self.bktangle]]
        num_list = len(self.cmd_data_list)
        i = 0
        while not self.stop_event.is_set():
            start_time = time.time()
            cmd_data_list = self.cmd_data_list  # 경로생성 loop에서 생성된 cmd_data_list를 cmd_data_list 변수에 저장
            trig_data = self.trigger  # 트리거 신호 0, 1

            #조인트 각도 Command set가 업데이트 되지 않은 경우 
            #cmd_data_list는 여러 경로점의 집합이고, cmd_data는 현재 보내줄 경로 하나 
            if cmd_data_list == prev_cmd_data_list:  # 이전 경로와 같은 경우 하나씩 꺼내서 제어기로 보냄
                if i < num_list:
                    cmd_data = cmd_data_list[i]  # 생성된 경로 리스트 [swing, boom, arm, bucket]
                    i = i + 1
                else:
                    cmd_data = cmd_data_list[num_list - 1]

            #조인트 각도 Command set 이 새로 업데이트 된 경우 
            else:  # 이전 경로 리스트와 같지 않다면 리셋
                cmd_data = prev_cmd_data_list[-1]  #다음 경로 프로세스의 첫 경로점 = 이전 경로 프로세스의 끝 경로점 
                num_list = len(cmd_data_list)
                i = 0

            cmd_dist, cmd_dept, cmd_aoa = forwardkinematics_real(radians(cmd_data[1]), radians(cmd_data[2]), radians(cmd_data[3]))
            cmd_aoa = degrees(cmd_aoa)

            cur_dist, cur_dept, cur_aoa = forwardkinematics_real(radians(self.boomangle), radians(self.armangle), radians(self.bktangle))
            cur_aoa = degrees(cur_aoa)

            #CAN 프로토콜에 맞게 바꿔줌 
            #해상도가 0.1도이기에 10을 바꿔줌 
            upperSwing = unsigned_byte(int(cmd_data[0] * 10)).value >> 8
            lowerSwing = unsigned_byte(int(cmd_data[0] * 10)).value & 0x00ff

            upperBoom = unsigned_byte(int(cmd_data[1] * 10)).value >> 8
            lowerBoom = unsigned_byte(int(cmd_data[1] * 10)).value & 0x00ff

            upperArm = unsigned_byte(int(cmd_data[2] * 10)).value >> 8
            lowerArm = unsigned_byte(int(cmd_data[2] * 10)).value & 0x00ff

            upperBkt = unsigned_byte(int(cmd_data[3] * 10)).value >> 8
            lowerBkt = unsigned_byte(int(cmd_data[3] * 10)).value & 0x00ff

            #조인트 각도 command 보내기 
            cmd_msg = can.Message(arbitration_id=0x01F8, is_extended_id=False, data=[lowerSwing, upperSwing, lowerBoom,
                                                                                     upperBoom, lowerArm, upperArm, lowerBkt, upperBkt])
            #trig_msg가 1이여야 굴착기가 작도암 
            trig_msg = can.Message(arbitration_id=0x01F9, is_extended_id=False, data=[trig_data])
            cmd_msg.timestamp = time.time() - start_time
            #CAN 데이터 보냄 
            self.bus.send(trig_msg)
            self.bus.send(cmd_msg)
            #self.recorddata 의 버퍼 
            if self.logging_trg: ## 데이터 로깅 버튼을 통해 On ##
                self.logging_Data = [time.time(), cmd_dist, cmd_dept, cmd_aoa, cmd_data[0], cmd_data[1], cmd_data[2], cmd_data[3],
                                    cur_dist, cur_dept, cur_aoa, self.swingangle, self.boomangle, self.armangle, self.bktangle, self.state_log]
                self.log_deque.appendleft(self.logging_Data)
            else:
                pass
            ### Autobox에 보내는 명령을 UI에 표시 ###
            if self.UI_ON is True:
                self.SEND_CMD.set(f'current swing angle : {round(cmd_data[0],2)}\n'
                                f'current boom angle : {round(cmd_data[1],2)}\n'
                                f'current arm angle : {round(cmd_data[2],2)}\n'
                                f'current bucket angle : {round(cmd_data[3],2)}')
                self.EXC_STATE_MSG.set(f'current excavator state : {self.EXC_state}')
            
            prev_cmd_data_list = cmd_data_list

            end_time = time.time()  #CAN통신으로 데이터 보낸 후 시각
            #print("th_CanTx lap time :",end_time-start_time)
            #100ms 주시로 조인트 커멘드 값 보냄 
            if CAN_TEST:
                sampling_time = 0.1
            #가상 시뮬로 할 경우 10ms로 설정(시뮬 시간을 단축하기 위함)
            else:
                sampling_time = 0.01
            #주기가 0.1초가 안되면 기다림 
            if start_time-end_time <= 0.1:
                time.sleep(sampling_time-(start_time-end_time))
            else:
                "100ms을 지키지 못함"

        trig_msg = can.Message(arbitration_id=0x01F9, is_extended_id=False, data=[0])
        self.bus.send(trig_msg)
        print('CanTx 스레드 종료')
            
    def th_CanRx(self):

        print("CanRx 스레드 : CAN데이터를 기다리고 있습니다.")
        #cab bus를 통해 들어오는 값 스윙, 붐, 암, 버킷(16진수로 되어 있음)
        excavator_data = self.bus.recv()                        # Autobox에서 데이터가 넘어올때까지 대기
        print("CanRx 스레드 : 첫 CAN데이터 도착.")
        print("CanRx 스레드 :", excavator_data)
        self.conv_recv_data(excavator_data)                     # Autobox에서 받은 데이터 변환
        self.start_path_gen_event.set()                         # CAN 데이터 수신 후 TCPIP_Tx 스타트 신호
        while not self.stop_event.is_set():
            start_time = time.time()
            #프로그램이 안 멈추기 위한 예외처리  
            try:
                excavator_data = self.bus.recv(1)  # 1초 내로 받지 못하면 Autobox가 데이터를 보내고 있지 않음으로 판단
            except Exception:
                pass
            
            if excavator_data is not None:
                #can통신을 통해서 받은 데이터를 10진수 degree로 바꿔줌 
                self.conv_recv_data(excavator_data)             # 받은 데이터(joint angle) 단위 및 forward kinematics 변환
                
                ## UI에 현재값 출력 ##
                if self.UI_ON is True:
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

    #굴착 경로 데이터를 저장하고 상태를 업데이트 해주는 스레드 
    def th_recordData(self, cond):
        while not self.stop_event.is_set():
            with cond:
                if self.state == 'circular':
                    self.circular_state = True 
                    self.trenching_state = False 
                    self.incline_state = False 
                    self.mountain_state = False 

                elif self.state == 'trenching':
                    self.circular_state = False 
                    self.trenching_state = True 
                    self.incline_state = False 
                    self.mountain_state = False 

                elif self.state == 'incline':
                    self.circular_state = False 
                    self.trenching_state = False
                    self.incline_state = True 
                    self.mountain_state = False        

                elif self.state == 'mountain':
                    self.circular_state = False 
                    self.trenching_state = False
                    self.incline_state = False 
                    self.mountain_state = True  

                elif self.state == 'waiting':
                    self.circular_state = False 
                    self.trenching_state = False
                    self.incline_state = False 
                    self.mountain_state = False  

                cond.notify_all() 

            if len(self.log_deque) > 0 and self.logging_trg:
                pathdata = self.log_deque.pop()
                recordInfo = pd.DataFrame(pathdata)
                recordInfo.T.to_csv(self.DataRoute, mode='a', index=False, header=self.headers)
                self.headers = False
            #빈 큐에서 뭐를 뽑는 것을 방지하기 위해 
            time.sleep(0.01)

    def on_connection_open(self, client):
        # Called by the WebSocket server when a new connection is opened. 한번만 실행
        self.TCP_CONECT.set('Connected!!')
    
    #카메라 연결 및 실시간 포인트 클라우드 확인하는 영상 
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
            #지형 촬영 자세 혹은 트럭 촬영 자세로 이동 완료한 경우, self.camCapture가 True로 설정됨 
            #그리고 찍은 포인트 클라우드를 ply 형식으로 저장 
            #autostate 1참조
            if self.camCapture:
                print("camstate :", self.camCapture)
                #time.sleep(1)
                points.export_to_ply(self.pcroute, mapped_frame)
                self.camCapture = False
                #밑의 정보들이 UI에 뜸 
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
        #8바이트 시리얼 통신 2개씩 끊어서 해석 
        cuSwingAng = recv_data.data[1] << 8 | recv_data.data[0]
        cuBoomAng = recv_data.data[3] << 8 | recv_data.data[2]
        cuArmAng = recv_data.data[5] << 8 | recv_data.data[4]
        cuBktAng = recv_data.data[7] << 8 | recv_data.data[6]
        
        #10진수 값으로 변환해줌 (우리가 원하는 값에 10이 곱해져서 옴)
        currentSwingAngle = signed_byte(cuSwingAng).value
        currentBoomAngle = signed_byte(cuBoomAng).value
        currentArmAngle = signed_byte(cuArmAng).value
        currentBktAngle = signed_byte(cuBktAng).value

        #그래서 10으로 나누어줌 
        #각 멤버 변수는 현재 파라미터 
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
        self.SV_GUI.geometry('1200x800')
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
        
        self.STATE_MSG = tk.StringVar()
        self.STATE_MSG.set('State')

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
        Fr[5].grid(column=2, row=4, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='현재작업',relief=tk.SOLID,labelanchor='n'))
        Fr[6].grid(column=1, row=6, padx=10, pady=10, sticky='n')
        Fr.append(ttk.LabelFrame(self.SV_GUI,text='작업선택',relief=tk.SOLID,labelanchor='n'))
        Fr[7].grid(column=1, row=7, padx=10, pady=10, sticky='n')


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
        
        self.graphCanvas = tk.Canvas(Fr[7], width=400, height=100)
        self.graphCanvas.grid(row=1, column=0, columnspan=5, padx=20, pady=20)

        self.circles = []
        radius = 20
        spacing = 80
        titles = ["부채꼴 굴착", "터파기 굴착", "법면 굴착", "모인토사 굴착"]
        for i in range(4):
            x0 = 60 + i * spacing
            y0 = 40
            x1 = x0 + 2 * radius
            y1 = y0 + 2 * radius
            circle = self.graphCanvas.create_oval(x0, y0, x1, y1, fill='red', outline='black')
            self.circles.append(circle)
            self.graphCanvas.create_text(x0 + radius, y0 - 25, text=titles[i], font=("Helvetica", 10))
        
        if self.state == 'circular':
            self.graphCanvas.itemconfig(self.circles[0], fill='green')
        
        elif self.state == 'trenching':
            self.graphCanvas.itemconfig(self.circles[1], fill='green')

        elif self.state == 'incline':
            self.graphCanvas.itemconfig(self.circles[2], fill='green')

        elif self.state == 'mountain':
            self.graphCanvas.itemconfig(self.circles[3], fill='green')

        # 버튼 초기화 및 콜백 연결
        self.circular_Btn = ttk.Button(Fr[6], text='부채꼴 굴착', command=lambda: self.change_state('circular'))
        self.circular_Btn.grid(column=0, row=0, sticky='we', padx=10, pady=5)

        self.circular_Btn = ttk.Button(Fr[6], text='터파기 굴착', command=lambda: self.change_state('trenching'))
        self.circular_Btn.grid(column=1, row=0, sticky='we', padx=10, pady=5)

        self.circular_Btn = ttk.Button(Fr[6], text='법면 굴착', command=lambda: self.change_state('incline'))
        self.circular_Btn.grid(column=2, row=0, sticky='we', padx=10, pady=5)

        self.circular_Btn = ttk.Button(Fr[6], text='모인토사 굴착', command=lambda: self.change_state('mountain'))
        self.circular_Btn.grid(column=3, row=0, sticky='we', padx=10, pady=5)

        self.circular_Btn = ttk.Button(Fr[6], text='대기', command=lambda: self.change_state('waiting'))
        self.circular_Btn.grid(column=4, row=0, sticky='we', padx=10, pady=5)

    def init_state(self):
        self.EXC_state = "Initialization"   # 건품연 작업 명령을 받을 준비가 되지 않았습니다.
       
    def waiting_state(self):
        self.EXC_state = "Waiting"          # 건품연 작업 명령을 받을 준비가 되었습니다.

    def working_state(self):
        self.EXC_state = "Working"          # 건품연 작업 명령을 받을 준비가 되었습니다.
    #이거 누르고 가만히 있으면 됨 
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

    def change_state(self, state):
        self.state = state

        if self.state == 'circular':
            self.graphCanvas.itemconfig(self.circles[0], fill='green')
            self.graphCanvas.itemconfig(self.circles[1], fill='red')
            self.graphCanvas.itemconfig(self.circles[2], fill='red')
            self.graphCanvas.itemconfig(self.circles[3], fill='red')

        elif self.state == 'trenching':
            self.graphCanvas.itemconfig(self.circles[0], fill='red')
            self.graphCanvas.itemconfig(self.circles[1], fill='green')
            self.graphCanvas.itemconfig(self.circles[2], fill='red')
            self.graphCanvas.itemconfig(self.circles[3], fill='red')

        elif self.state == 'incline':
            self.graphCanvas.itemconfig(self.circles[0], fill='red')
            self.graphCanvas.itemconfig(self.circles[1], fill='red')
            self.graphCanvas.itemconfig(self.circles[2], fill='green')
            self.graphCanvas.itemconfig(self.circles[3], fill='red')

        elif self.state == 'mountain':
            self.graphCanvas.itemconfig(self.circles[0], fill='red')
            self.graphCanvas.itemconfig(self.circles[1], fill='red')
            self.graphCanvas.itemconfig(self.circles[2], fill='red')
            self.graphCanvas.itemconfig(self.circles[3], fill='green')

        elif self.state == 'waiting':
            self.graphCanvas.itemconfig(self.circles[0], fill='red')
            self.graphCanvas.itemconfig(self.circles[1], fill='red')
            self.graphCanvas.itemconfig(self.circles[2], fill='red')
            self.graphCanvas.itemconfig(self.circles[3], fill='red')

        
if __name__ == '__main__':
    AE = Integrated_Autonomous_Excavator(host_IP=HOST_S, host_port=9999)
    tk.mainloop()
