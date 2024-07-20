## 평탄화 작업 경로 생성 모듈 ##

import threading
import json
import time

## 건품연에서 보내주는 프로토콜을 경로생성을 위한 값으로 변환해주는 모듈
import cmd_util   

## 경로데이터 저장관련 ###
import pandas as pd
from collections import deque   #데이터 push/pop 연산 및 큐 라이브러리 

## 건품연 명령 예시 ###
import virtual_cmd     

## 굴착기 역기구학, 정기구학 푸는 함수
from inversekinematics_rc_real import * 
from math import *
import numpy as np

## 다양한 경로를 생성하기 위한 모듈
import PathGenerationRule as PG
               
class LEVELING():
    def __init__(self):
        self.path_generator = PG.excavation_path()
        pos_beforeSwing = [3.2, 2.2, radians(-167)]
        self.path_generator.basicinput(2.9, -2.1, -2.1, -3, 9.3, pos_beforeSwing, 0.8)
        # tresh_joint, tresh_Traj, _ = self.path_generator.pathgeneration(10)
        
        # self.condition = threading.Condition()

        self.EXC_state = "Initialization" # "Waiting, "Ready", "Working", "Emergency"

        self.logging_trg = False #데이터 로깅 트리거/ 데이터 저장 

        # 조인트/버켓 위치 및 aoa 경로 데이터 목록, csv 파일 저장될 시, 헤더 역할
        self.headers = ['time(s)','x(m)','y(m)','z(m)','SwingAng_cmd(Deg)','BoomAng_cmd(Deg)','ArmAng_cmd(Deg)','BktAng_cmd(Deg)'] 

        #굴삭 기록 저장시 조인트/버켓 위치 및 aoa 경로 데이터 파일(csv) 저장되는 주소
        self.DataRoute = './Data/record_path_leveling.csv'

        self.log_deque = deque() #데이터 로깅 큐

        # 도달하지 않은 경우
        self.errorReset = False
        
        # 가상 굴착기 정보를 위한 스레드 오픈 
        virtual_CAN_th = threading.Thread(target=self.th_VirtualCAN,args=())          
        virtual_CAN_th.start()

        record_th = threading.Thread(target=self.th_recordData,args=())     # 데이터 로깅 스레드 : 100ms 마다 데이터 로깅
        record_th.start()

    def path_generation_leveling(self, cond):
        
        #############  평탄화(법면) 작업 단계(exc_phase : 1 - 8단계로 가정)  #######################
        # 0단계 : 현재 자세에서 대기 자세로 버켓 이동              (대기 자세 이동 작업 : 처음에만 시행)
        # 1단계 : 대기 자세에서 평탄화(법면) 작업 위치로 스윙 이동    (평탄화(법면) 작업 시작을 위한 스윙 작업)
        # 2단계 : 대기(현재) 자세에서 평탄화 초기 위치로 버켓 이동    (평탄화(법면) 초기 위치 이동 작업)
        # 3단계 : 평탄화 초기 위치에서 평탄화 종료 위치로 버켓 이동    (평탄화(법면) 작업)
        # 4단계 : 평탄화 종료 위치에서 스윙(상차) 자세로 버켓 이동    (스윙(상차) 자세 이동 작업)
        
        ## 상차가 있는 경우 5-8단계 포함 ##
        # 5단계 : 스윙(상차) 자세에서 상차를 위해 스윙              (상차를 위한 스윙 작업)
        # 6단계 : 스윙 후 상차 위치로 버켓 이동                   (상차 위치 이동 작업)
        # 7단계 : 토사를 버리기 위한 버켓 이동                      (상차 작업)
        # 8단계 : 상차 후 스윙(복귀) 자세로 버켓 이동               (스윙(상차) 자세 이동 작업)
        
        # 9단계 : 다음 평탄화 작업을 위한 스윙 위치로 스윙          (다음 unit work 또는 종료를 위한 스윙 작업)
        # 10단계 : 종료 시 초기 대기 자세로 버켓 이동               (초기 대기 자세 이동 작업 : 종료 시에만 시행)
        ######################################################################################
        ### 아래 자세에 대한 정의가 필요함 ###
        # 1. 대기 자세 : 대기 자세로 이동하는 자세
        # 2. 스윙(상차) 자세 : 상차를 위한 스윙 전, 버켓 자세(토사를 가지고 있을 때)
        # 3. 스윙(복귀) 자세 : 복귀를 위한 스윙 전, 버켓 자세
        
        print("PATH generator start!")
        
        # 평탄화 작업 phase에 따른 작업 순서 리스트
        exc_phase_list = ["0","1","2","3","4","5","6","7","8","9","10","11"]
        # 상차가 없는 경우는 아래 리스트 사용 
        # exc_phase_list = ["0","1","2","3","4","9","10"]
        cur_exc_phase = ""
        cur_exc_phase = "0"
        exc_phase_idx = 0
        swing_idx = 0
        
        
        ###################################################################
        # 모든 unit(sector)에 대한 작업정보 저장 cf)트렌칭의 경우 무조건 1개
        location = EXC_CMD["location"]
        # unit(sector) work 개수 저장
        num_UnitWork = len(location)


        # 작업명령 저장(이전 TCP/IP 통신으로 받은 프로토콜로 명령 저장)
        EXC_CMD = self.work_input()            
        
        # 전체 작업에 대한 flag (0 : 단위 작업중, 1 : 전체 작업 완료)
        LocalWork = 0
        

        ######################### 굴착 작업 정보 획득 #########################
        # 모든 unit(sector)에 대한 작업정보 저장 cf)트렌칭의 경우 무조건 1개
        location = EXC_CMD["location"]
        # unit(sector) work 개수 저장
        num_UnitWork = len(location)
        
        location_distFar_list = []
        location_distNear_list = []
        desired_depth_list = []
        desired_depthFar_list = []
        desired_depthNear_list = []
        desired_swingAngle_list = []
        for i in range(num_UnitWork):  
            location_distFar, location_distNear, desired_depth, desired_depthFar, desired_depthNear, desired_swingAngle = cmd_util.excavation_location(EXC_CMD,i)
            location_distFar_list.append(round(location_distFar,2))
            location_distNear_list.append(round(location_distNear,2))
            desired_depth_list.append(round(desired_depth,2))
            desired_depthFar_list.append(round(desired_depthFar, 2))
            desired_depthNear_list.append(round(desired_depthNear, 2))
            desired_swingAngle_list.append(round(desired_swingAngle,2))
        DumpDist, DumpDepth, DumpSwingAngle = cmd_util.dump_location(EXC_CMD)
        ######################### 굴착 작업 정보 획득 #########################
        
        ######################### 스윙파라미터 #########################
        swing_num = num_UnitWork
        
        ############ 부채꼴 깊이에 따른 로컬 굴착 횟수 정하는 알고리즘 #############
        # num_iteration_list = []
        # for num in range(num_UnitWork):
        #     print(num)
        #     print(self.location_distNear_list[num])
        #     num_iteration = cmd_util.calc_num_iteration(self.location_distFar_list[num], self.location_distNear_list[num], self.desired_depth_list[num], cur_depth = -2.0)
        #     print(num_iteration)
        #     num_iteration_list.append(num_iteration)
        #     #num_iteration = 4
        # print("num interation list:",num_iteration_list) 
        
        ## 실행되는 순간의 굴착기 현재 조인트 각도 저장
        cur_joint_angle = self.save_current_pos()   
        curswing, curboom, curarm, curbkt = cur_joint_angle[0], cur_joint_angle[1], cur_joint_angle[2], cur_joint_angle[3]
        # 현재 조인트 각도에 따른 굴착기 x,z,AOA 위치
        curdist, curdept, curaoa = forwardkinematics_real(radians(curboom), radians(curarm), radians(curbkt))
        
        l1 = 6.24565
        l2 = 3.11364
        l3 = 1.910051
        
        ### exc_phase-0 ###
        print("exc_phase 0 : 평탄화 작업 준비 경로 생성")
        # 대기 위치 및 이동 속도 설정
        lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 3.2, 2.2, radians(-162)
        # 현재 위치에서 대기 위치로 이동하는 경로 생성
        self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
        cmd_data_list_0, Trajectory_0, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)      
        print("exc_phase 0 : 완료")
        ### 다음 경로 생성을 위한 위치 업데이트 ###
        self.update_start_pos(cmd_data_list_0)  
        print("test")        
        for i in range(num_UnitWork): 
            print("평탄화 단위 작업 :",i,"번 시작")

            ### exc_phase-1 ###
            print(f"exc_phase {i}-1 : 평탄화(법면) 작업 시작을 위한 스윙 작업")
            # 스윙속도, 목표 스윙각도 설정
            swingvel, swinggoal = 25, desired_swingAngle_list[swing_idx] # deg/s, degree
            self.path_generator.swingInfo(swingvel, swinggoal)
            cmd_data_list_1, Trajectory_1, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-1 : 완료")
            
            ### 현재 위치 업데이트 ###
            self.update_start_pos(cmd_data_list_1)
            
            ### exc_phase-2 ###
            print(f"exc_phase {i}-2 : 평탄화(법면) 초기 위치 이동 작업")
            # 스윙속도, 목표 스윙각도 설정
            swingvel, swinggoal = 25, desired_swingAngle_list[swing_idx] # deg/s, degree
            self.path_generator.swingInfo(swingvel, swinggoal)
            cmd_data_list_1, Trajectory_1, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-2 : 완료")
            
            #...
            # 9번 작업
            # 10번 작업                        

            if prev_exc_phase != cur_exc_phase and cur_exc_phase == "0":
                self.state_log = "0-DampZone"
                self.exc_seq_MSG.set('Pass')
                self.cmd_data_list = [self.cmd_data_list[-1]]
                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "1":
                print("exc_phase 0 : 대기 자세 이동 작업")
                
                lineVel, camCapDist, camCapDept, camCapAOA = 1.1, 3.2, 2.2, radians(-162)
                self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]
                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                prev_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "2":
                self.state_log = "2-init swing"
                self.exc_seq_MSG.set('법면 작영역으로 스윙')
                swingvel, swinggoal = 25, swing_angle_list[(self.swing_idx-1)%swing_num] # deg/s, degree
                self.path_generator.swingInfo(swingvel, swinggoal)
                self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]

                prev_exc_phase = cur_exc_phase
            
            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "3":
                """
                법면 굴착 초기 위치 이동
                """
                self.exc_seq_MSG.set('법면 굴착 시작점 이동')
                self.state_log = "3-init exc position"

                ################################## 중요 파라미터 ####################################
                LineEndDist, LineEndDept = round(self.location_distFar_list[(self.swing_idx - 1) % swing_num], 2), \
                                        round(self.desired_depthFar_list[(self.swing_idx - 1) % swing_num], 2)

                IncExcDist, IncExcDept = round(self.location_distNear_list[(self.swing_idx - 1) % swing_num], 2), \
                                        round(self.desired_depthNear_list[(self.swing_idx - 1) % swing_num], 2)

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

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "4":
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

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "5":
                self.state_log = "5-Lifting"
                self.exc_seq_MSG.set('리프팅')
                lineVel, camCapDist, camCapDept, camCapAOA = 0.7, 3.2, 2.2, radians(-165)
                self.path_generator.lineBasicInfo(lineVel, camCapDist, camCapDept, camCapAOA)
                self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]
                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "6":
                # 스윙 to 상차지점
                self.exc_seq_MSG.set('상차단계 : 스윙 to 상차')
                self.state_log = "6-swing to loading"
                swingvel, swinggoal = 25, DumpSwingAngle  # deg/s, degree
                #swingvel, swinggoal = 25, 90
                self.path_generator.swingInfo(swingvel, swinggoal)
                self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]
                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "7":
                # 상차지점으로(직선경로)
                self.exc_seq_MSG.set('상차단계 : 상차 지점 이동')
                self.state_log = "7-loading position"

                loadVel, loadDist, loadDept, loadAOA = 0.5, DumpDist, 0.5, radians(-162)

                self.path_generator.lineBasicInfo(loadVel, loadDist, loadDept, loadAOA)
                self.cmd_data_list, Trajectory, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]
                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "8":
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
                                            f'Dump swing angle : {round(DumpSwingAngle, 2)}\n')

                cur_exc_phase = cur_exc_phase

            elif prev_exc_phase != cur_exc_phase and cur_exc_phase == "9":
                # 스윙 to 상차지점
                self.exc_seq_MSG.set('원위치')
                self.state_log = "9-swing before moving"
                swingvel, swinggoal = 25, 0  # deg/s, degree
                self.path_generator.swingInfo(swingvel, swinggoal)
                self.cmd_data_list, Trajectory, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                curswing = self.cmd_data_list[-1][0]
                curdist, curdept, curaoa = Trajectory[-1][0], Trajectory[-1][1], Trajectory[-1][2]

                cur_exc_phase = cur_exc_phase

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
                    print(f"현재 굴착 {cur_exc_phase} 단계완료")
                    excavation_idx = excavation_idx + 1
                    cur_exc_phase = exc_phase_list[excavation_idx]
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

        time.sleep(0.1)

    # 작업 명령을 작성해주세요
    def work_input(self):    
        """
        평탄화 경로 생성을 위한 input data 생성
        
        """
        ROS_cmd = virtual_cmd.work_cmd_SE()
        
        return ROS_cmd

    # 가상 굴착기 정보를 위한 스레드(경로 명령값 그래로 거동하는 것으로 가정)
    def th_VirtualCAN(self):
        
        # 수식 추가
        self.swingangle = 0
        self.boomangle = 0
        self.armangle = 0
        self.bktangle = 0

        self.cur_x, self.cur_z, self.cur_AA = forwardkinematics_real(radians(self.boomangle), radians(self.armangle), radians(self.bktangle))
          
    # 굴착 경로 데이터를 저장하고 상태를 업데이트 해주는 스레드 
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

    # 실행되는 순간의 굴착기 조인트 각도 저장 함수(내부함수)
    def save_current_pos(self):                 
        return [self.swingangle, self.boomangle, self.armangle, self.bktangle]
    
    # 경로 생성을 위한 초기 위치 업데이트
    def update_start_pos(self,cmd):
        ### desired path 마지막 값으로 업데이트 ###
        curswing = cmd[-1][0]
        curboom = cmd[-1][1]
        curarm = cmd[-1][2]
        curbkt = cmd[-1][3]
        
        return curswing, curboom, curarm, curbkt
        

if __name__ == '__main__':
    AE = LEVELING()
