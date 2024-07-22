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
        # 상차 유무
        self.isDump = True

        # 조인트/버켓 위치 및 aoa 경로 데이터 목록, csv 파일 저장될 시, 헤더 역할
        self.headers = ['time(s)','x(m)','y(m)','z(m)','SwingAng_cmd(Deg)','BoomAng_cmd(Deg)','ArmAng_cmd(Deg)','BktAng_cmd(Deg)'] 

        ### 데이터 로깅 관련 변수 ###
        #굴삭 기록 저장시 조인트/버켓 위치 및 aoa 경로 데이터 파일(csv) 저장되는 주소
        self.DataRoute = './Data/record_path_leveling.csv'
        self.logging_trg = False #데이터 로깅 트리거/ 데이터 저장
        self.log_deque = deque() #데이터 로깅 큐
        
        # 가상 굴착기 정보를 위한 스레드 오픈 
        # virtual_CAN_th = threading.Thread(target=self.th_VirtualCAN,args=())          
        # virtual_CAN_th.start()
        
        # 데이터 로깅 스레드 : 100ms 마다 데이터 로깅
        # record_th = threading.Thread(target=self.th_recordData,args=())     
        # record_th.start()
        
        self.path_generation_leveling()

    def path_generation_leveling(self):
        
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
    
        # 8단계 : 다음 평탄화 작업을 위한 위치로 스윙             (다음 unit work 또는 종료를 위한 스윙 작업)
        
        # 9단계 : 종료 시 초기 대기 자세로 버켓 이동               (초기 대기 자세 이동 작업 : 종료 시에만 시행)
        
        ### 아래 자세에 대한 정의가 필요함 ###
        # 1. 대기 자세 : 대기 자세로 이동하는 자세
        # 2. 스윙(상차) 자세 : 상차를 위한 스윙 전, 버켓 자세(토사를 가지고 있을 때)
        # 3. 스윙(복귀) 자세 : 복귀를 위한 스윙 전, 버켓 자세
        # 4. 마지막 작업 후 스윙은 0도로 설정
        
        ### 작업 예시 ###
        # 1. 상차를 포함하고 2개의 unit work가 있는 경우
        #  (0단계-1단계) - (2단계-3단계-4단계-5단계-6단계-7단계-8단계) - (2단계-3단계-4단계-5단계-6단계-7단계-8단계) - 9단계
        # 2. 상차가 없고 3개의 unit work가 있는 경우
        #  (0단계-1단계) - (2단계-3단계-4단계-8단계) - (2단계-3단계-4단계-8단계) - (2단계-3단계-4단계-8단계) - 9단계
        ######################################################################################
        
        print("PATH generator 시작!")

        cur_exc_phase = ""
        cur_exc_phase = "0"
        exc_phase_idx = 0
        swing_idx = 0           
        
        # 전체 작업에 대한 flag (0 : 단위 작업중, 1 : 전체 작업 완료)
        LocalWork = 0
        

        ######################### 굴착 작업 정보 획득 #########################
        # 작업명령 저장(이전 TCP/IP 통신으로 받은 프로토콜)
        EXC_CMD = self.work_input() 
        print(EXC_CMD)
        # 모든 unit(sector)에 대한 작업정보 저장 cf)트렌칭의 경우 무조건 1개
        location = EXC_CMD["location"]
        # unit(sector) work 개수 저장
        num_UnitWork = len(location)
        print("unit work 개수 : ",num_UnitWork)

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
        # 덤프 작업이 있는지 없는지에 따라 조건문 추가해야함
        DumpDist, DumpDepth, DumpSwingAngle = cmd_util.dump_location(EXC_CMD)
        
        ## 경로생성 전 굴착기 현재 조인트 각도 저장
        cur_joint_angle = self.save_current_pos()   
        curswing, curboom, curarm, curbkt = cur_joint_angle[0], cur_joint_angle[1], cur_joint_angle[2], cur_joint_angle[3]
        # 현재 조인트 각도에 따른 굴착기 x,z,AOA 위치
        curdist, curdept, curaoa = forwardkinematics_real(radians(curboom), radians(curarm), radians(curbkt))
        
        l1 = 6.24565
        l2 = 3.11364
        l3 = 1.910051
        
        ### exc_phase-0 ###
        print("exc_phase 0 : 평탄화(법면) 작업 준비 경로 생성")
        # 대기 위치 및 이동 속도 설정(설정 필요함!!)
        lineVel, InitDist, InitDept, initAOA = 1.1, 3.2, 2.2, radians(-162)
        # 현재 위치에서 대기 위치로 이동하는 경로 생성
        self.path_generator.lineBasicInfo(lineVel, InitDist, InitDept, initAOA)
        cmd_data_list_0, Trajectory_0, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)      
        print(cmd_data_list_0)
        print(Trajectory_0)
        print("exc_phase 0 : 완료")
        
        # 다음 경로 생성을 위한 위치 업데이트
        curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_0,Trajectory_0)
        
        ### exc_phase-1 ###
        print(f"exc_phase 1 : 평탄화(법면) 작업 시작을 위한 스윙 작업")
        # 스윙속도, 목표 스윙각도 설정
        swingvel, swinggoal = 25, desired_swingAngle_list[swing_idx] # deg/s, degree
        self.path_generator.swingInfo(swingvel, swinggoal)
        cmd_data_list_1, Trajectory_1, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
        print(f"exc_phase 1 : 완료")
        
        # 다음 경로 생성을 위한 위치 업데이트
        curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_1,Trajectory_1)
    
        for i in range(num_UnitWork): 
            print("평탄화 단위 작업 :",i,"번 시작")
            
            ### exc_phase-2 ###
            print(f"exc_phase {i}-2 : 평탄화(법면) 초기 위치 이동 작업")
            # 평탄화(법면) 시작 위치
            LineEndDist = round(location_distFar_list[(swing_idx)], 2)
            LineEndDept = round(desired_depthFar_list[(swing_idx)], 2)
            # 평탄화(법면) 종료 위치
            IncExcDist  = round(location_distNear_list[(swing_idx)], 2)
            IncExcDept  = round(desired_depthNear_list[(swing_idx)], 2)

            ## 평탄화(법면) 작업 시작,종료 위치에서의 AOA를 설정하기 위한 알고리즘
            ## 평탄화 시작점에서 암과 버켓 링크가 일직선이 되도록 가정
            ## 위 조건을 만족할 때 붐, 암 링크의 workspace 확인
            ## 벗어나면 시작점 조정 
            
            # 시작점 조정 전 초기 평탄화 시작 위치 저장
            LineEndDist_orig, LineEndDept_orig = LineEndDist, LineEndDept
            # 시작점 조정 알고리즘(worspace 확인)
            for a in range(1,1001):
                l2_new = l2 + l3
                l_ground = sqrt(LineEndDist ** 2 + LineEndDept ** 2)

                Ang_Arm = -(pi - acos(np.clip((l1 ** 2 + l2_new ** 2 - l_ground ** 2) / (2 * l1 * l2_new), -1, 1)))
                Ang_Boom = acos(np.clip((l1 ** 2 + l_ground ** 2 - l2_new ** 2) / (2 * l1 * l_ground), -1, 1)) + atan2(LineEndDept, LineEndDist)

                Ang_Arm, Ang_Boom = degrees(Ang_Arm), degrees(Ang_Boom)

                if Ang_Boom <= 59.6 and Ang_Boom >= 0 and Ang_Arm <= -34.25 and Ang_Arm >= -159.1:
                    break
                else:
                    print(f"시작점 조정 : {a}")
                    LineEndDist = ((1000-a)*LineEndDist_orig + a*IncExcDist)/1000
                    LineEndDept = ((1000-a)*LineEndDept_orig + a*IncExcDept)/1000

            lambda_theta = np.clip((LineEndDist ** 2 + LineEndDept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(LineEndDist ** 2 + LineEndDept ** 2) * (l2 + l3)), -1, 1)
            # 평탄화 시작점에서의 AOA 결정
            LineEndAOA = atan2(LineEndDept, LineEndDist) - acos(lambda_theta)   
            # 초기위치 이동속도
            linevel = 1.1  # m/s
            self.path_generator.lineBasicInfo(linevel, LineEndDist, LineEndDept, LineEndAOA)  # 초기위치
            cmd_data_list_2, Trajectory_2, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-2 : 완료")
            
            ## 다음 경로 생성을 위한 위치 업데이트
            curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_2,Trajectory_2)
            
            ### exc_phase-3 ###
            print(f"exc_phase {i}-3 : 평탄화(법면) 작업")
            # 평탄화(법면) 속도 설정
            IncExcVel = 0.5
            # 평탄화 작업 최종 AOA
            IncExcAOA = radians(-140)
            self.path_generator.lineBasicInfo(IncExcVel, IncExcDist, IncExcDept, IncExcAOA)
            cmd_data_list_3, Trajectory_3, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-3 : 완료")
            
            ## 다음 경로 생성을 위한 위치 업데이트
            curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_3,Trajectory_3)
            
            ### exc_phase-4 ###
            print(f"exc_phase {i}-4 : 스윙(상차) 자세 이동 작업")
            # 스윙(상차) 자세 및 이동 속도 설정(설정 필요함!!)
            lineVel, DumpInitDist, DumpInitDept, DumpInitAOA = 0.7, 3.2, 2.2, radians(-165)
            self.path_generator.lineBasicInfo(lineVel, DumpInitDist, DumpInitDept, DumpInitAOA)
            cmd_data_list_4, Trajectory_4, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-4 : 완료")
       
            ## 다음 경로 생성을 위한 위치 업데이트
            curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_4,Trajectory_4)
            
            if self.isDump:
                ### exc_phase-5 ###
                print(f"exc_phase {i}-5 : 상차를 위한 스윙 작업")
                # 스윙속도, 목표 스윙각도 설정
                swingvel, swinggoal = 25, DumpSwingAngle  # deg/s, degree
                self.path_generator.swingInfo(swingvel, swinggoal)
                cmd_data_list_5, Trajectory_5, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
                print(f"exc_phase {i}-5 : 완료")
                
                # 다음 경로 생성을 위한 위치 업데이트
                curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_5,Trajectory_5)
                
                ### exc_phase-6 ###
                print(f"exc_phase {i}-6 : 상차 위치 이동 작업")
                # 스윙속도, 목표 스윙각도 설정(설정 필요함!!)
                loadVel, loadDist, loadDept, loadAOA = 0.5, DumpDist, 0.5, radians(-162)
                self.path_generator.lineBasicInfo(loadVel, loadDist, loadDept, loadAOA)
                cmd_data_list_6, Trajectory_6, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)
                print(f"exc_phase {i}-6 : 완료")
                
                # 다음 경로 생성을 위한 위치 업데이트
                curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_6,Trajectory_6)
                
                ### exc_phase-7 ###
                print(f"exc_phase {i}-7 : 상차 작업")
                # 상차 관련 상수 설정(설정 필요함!!)
                load_init_dist, load_init_dept = curdist, curdept
                load_rad, load_vel = 2.2, 0.7
                load_final_dist = 4 + load_rad + 0.2

                lambda_theta = np.clip((load_final_dist ** 2 + load_init_dept ** 2 + (l2 + l3) ** 2 - l1 ** 2) / (2 * sqrt(load_final_dist ** 2 + load_init_dept ** 2) * (l2 + l3)), -1, 1)
                load_final_aoa = atan2(load_init_dept, load_final_dist) - acos(lambda_theta) + radians(40)

                self.path_generator.loading_Track_info(load_init_dist, load_init_dept, load_final_dist, load_rad, load_vel, load_final_aoa)
                cmd_data_list_7, Trajectory_7, _ = self.path_generator.loading_Track(curswing, curdist, curdept, curaoa)
                print(f"exc_phase {i}-7 : 완료")
                
                # 다음 경로 생성을 위한 위치 업데이트
                curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_7,Trajectory_7)
                
            exc_phase_idx += 1
            swing_idx += 1
            
            ### 마지막 unit work 인지 확인 ###
            
            if exc_phase_idx < num_UnitWork: # 마지막 unit work가 아닌 경우
                ### exc_phase-8 ###
                print(f"exc_phase {i}-8 : 다음 unit work 위한 스윙 작업")
                # 스윙속도, 목표 스윙각도 설정
                swingvel, swinggoal = 25, desired_swingAngle_list[swing_idx] # deg/s, degree
            else: # 마지막 unit work 인 경우
                print(f"exc_phase {i}-8 : 다음 작업 종료를 위한 스윙 작업")
                # 스윙속도, 초기 스윙각도(=0) 설정(설정 필요함!!)
                swingvel, swinggoal = 25, 0 # deg/s, degree
            self.path_generator.swingInfo(swingvel, swinggoal)
            cmd_data_list_8, Trajectory_8, _ = self.path_generator.swingPath(curswing, curdist, curdept, curaoa)
            print(f"exc_phase {i}-8 : 완료")
                    
            # 다음 경로 생성을 위한 위치 업데이트
            curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_8,Trajectory_8)

        ### exc_phase-9 ###
        print("exc_phase 9 : 초기 대기 자세 이동 작업")
        # 대기 위치 및 이동 속도 설정(설정 필요함!!)
        lineVel, InitDist, InitDept, initAOA = 1.1, 3.2, 2.2, radians(-162)
        # 현재 위치에서 대기 위치로 이동하는 경로 생성
        self.path_generator.lineBasicInfo(lineVel, InitDist, InitDept, initAOA)
        cmd_data_list_9, Trajectory_9, _ = self.path_generator.linepath(curswing, curdist, curdept, curaoa)      
        print("exc_phase 9 : 완료")
        
        # 다음 경로 생성을 위한 위치 업데이트
        curswing, curdist, curdept, curaoa = self.update_start_pos(cmd_data_list_9,Trajectory_9)

        print("PATH generator 종료!")
    
    # 작업 명령을 작성해주세요
    def work_input(self):    
        """
        평탄화 경로 생성을 위한 input data 생성
        
        """
        ROS_cmd = virtual_cmd.work_cmd_leveling()
        
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
    def th_recordData(self):
        while True:
            
            if len(self.log_deque) > 0 and self.logging_trg:
                pathdata = self.log_deque.pop()
                recordInfo = pd.DataFrame(pathdata)
                recordInfo.T.to_csv(self.DataRoute, mode='a', index=False, header=self.headers)
                self.headers = False
            #빈 큐에서 뭐를 뽑는 것을 방지하기 위해 
            time.sleep(0.01)

    # 실행되는 순간의 굴착기 조인트 각도 저장 함수(내부함수)
    def save_current_pos(self):
        # 추 후 코드에 따라 수정되거나 삭제될 수 있음                 
        # return [self.swingangle, self.boomangle, self.armangle, self.bktangle]
        return [10, 50, -90, -50]
    
    # 경로 생성을 위한 초기 위치 업데이트
    def update_start_pos(self,cmd,traj):
        ### desired path 마지막 값으로 업데이트 ###
        curswing = cmd[-1][0]
        # curboom = cmd[-1][1]
        # curarm = cmd[-1][2]
        # curbkt = cmd[-1][3]

        curdist, curdept, curaoa = traj[-1][0], traj[-1][1], traj[-1][2]
        
        return curswing, curdist, curdept, curaoa
        

if __name__ == '__main__':
    AE = LEVELING()
