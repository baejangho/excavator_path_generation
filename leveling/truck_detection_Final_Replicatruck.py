import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
import sys
import cv2
import math
import csv
import matplotlib
import copy
from sklearn.cluster import DBSCAN
matplotlib.use('TkAgg')

# pip install scikit-learn

def truck_pointcloud(pcdfile):
    pcd = o3d.io.read_point_cloud(pcdfile)

    # 지표면이랑 평행하게 만드는 transform(가장 높은 점 찾기 + 나머지 두 점 이동 찾을 때 편하게 하려고)
    theta = np.deg2rad(-30)
    cam_x_rotation=np.array([[1, 0, 0, 0], [0, np.cos(theta), -np.sin(theta), 0], [0, np.sin(theta), np.cos(theta), 0], [0, 0, 0, 1]])

    pcd.transform(cam_x_rotation)

    pcd_np=np.asarray(pcd.points)

    # 트럭 앞쪽 대충 없애고, 땅 대략적으로 없앰
    ground_removed_np1 = pcd_np[(pcd_np[:, 0] > -0.8) & (pcd_np[:, 0] < 2.5) & (pcd_np[:, 1] > -2.5) & (pcd_np[:, 1] < 1) & (pcd_np[:, 2] > -8)]

    # DBSCAN 클러스터링 실행
    dbscan = DBSCAN(eps=0.1, min_samples=30)  # 매개변수는 데이터에 맞게 조정 필요
    labels = dbscan.fit_predict(ground_removed_np1)

    # 노이즈(레이블 == -1)가 아닌 포인트만 필터링
    ground_removed_np = ground_removed_np1[labels != -1]

    # 땅 제거 된 부분을 보여줌
    ground_removed_pcd = o3d.geometry.PointCloud()
    ground_removed_pcd.points = o3d.utility.Vector3dVector(ground_removed_np)

    o3d.io.write_point_cloud("ground_removed.pcd",ground_removed_pcd)

    #o3d.visualization.draw_geometries([ground_removed_pcd])

    # 회전 행렬 정의
    theta_rot = np.deg2rad(-30)
    cam_x_rotation_30 = np.array(
        [[1, 0, 0, 0], 
         [0, np.cos(theta_rot), -np.sin(theta_rot), 0], 
         [0, np.sin(theta_rot), np.cos(theta_rot), 0],
         [0, 0, 0, 1]])
    cam_z_rotation_30 = np.array(
        [[np.cos(theta_rot), -np.sin(theta_rot), 0, 0], 
         [np.sin(theta_rot), np.cos(theta_rot), 0, 0], 
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    cam_z_rotation_minus_30 = np.array(
        [[np.cos(-theta_rot), -np.sin(-theta_rot), 0, 0],
         [np.sin(-theta_rot), np.cos(-theta_rot), 0, 0], 
         [0, 0, 1, 0],
         [0, 0, 0, 1]])

    # 첫 번째 회전 상태에서 y값이 가장 큰 점 찾기
    pcd_rotated_1 = copy.deepcopy(ground_removed_pcd)
    pcd_rotated_1.transform(cam_x_rotation_30)
    pcd_rotated_1.transform(cam_z_rotation_30)
    pcd_rotated_1_np = np.asarray(pcd_rotated_1.points)
    index_max_y_1 = np.argmax(pcd_rotated_1_np[:, 1])
    point_max_y_1 = ground_removed_np[index_max_y_1]

    # 두 번째 회전 상태에서 y값이 가장 큰 점 찾기
    pcd_rotated_2 = copy.deepcopy(ground_removed_pcd)
    pcd_rotated_2.transform(cam_x_rotation_30)
    pcd_rotated_2.transform(cam_z_rotation_minus_30)
    pcd_rotated_2_np = np.asarray(pcd_rotated_2.points)
    index_max_y_2 = np.argmax(pcd_rotated_2_np[:, 1])
    point_max_y_2 = ground_removed_np[index_max_y_2]


    LeftRear_bef=point_max_y_1
    RightRear_bef=point_max_y_2
    #print('point1',point_max_y_1)
    #print('point2',point_max_y_2)

   
    w, index = ground_removed_pcd.segment_plane( 0.02, 3, 1000)
    inlier_cloud = ground_removed_pcd.select_by_index(index)
    inlier_cloud.paint_uniform_color([0, 0, 0])
    outlier_cloud = ground_removed_pcd.select_by_index(index, invert=True)
    obb=inlier_cloud.get_oriented_bounding_box()
    obb.color=(0,1,0)


    dir=np.array([-w[0],0,-w[2]])

    #print('real_dir:' ,dir)

    # 벡터의 크기를 계산합니다.
    norm = np.linalg.norm(dir)

    # 벡터를 크기로 나누어 단위 벡터를 얻습니다.
    unit_vector = dir / norm

    #print('unit vector:' ,unit_vector)

    # 트럭 y 축 기준으로 기울어진 정도 (cam 바라보는 방향이랑 트럭 방향 기울어진 정도)
    rot_angle=math.atan2(-(dir[2]),dir[0])*(-1)
    # 트럭 z 축 기준으로 기울어진 정도 (트럭이 땅에 얼마나 왼쪽 오른쪽이 기울어져 있는지)
    rot_angle2=math.atan2(dir[1],dir[0])
    #print('angle:',rot_angle)

    # unit vector에 5를 곱해서 트럭앞부분 두 점을 구합니다
    LeftFront_bef = LeftRear_bef + unit_vector * 5
    RightFront_bef = RightRear_bef + unit_vector * 5

    # 선택한 점들에 대한 구(spheres) 생성
    sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere1.translate(LeftFront_bef)
    sphere1.paint_uniform_color([0.1, 0.1, 0.1]) 

    sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere2.translate(RightFront_bef)
    sphere2.paint_uniform_color([0.1, 0.1, 0.1])  

    sphere3 = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere3.translate(LeftRear_bef)
    sphere3.paint_uniform_color([0.1, 0.1, 0.1])  

    sphere4 = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere4.translate(RightRear_bef)
    sphere4.paint_uniform_color([0.1, 0.1, 0.1])

    # 포인트 클라우드와 선택한 점을 함께 시각화 - 뒷편 평면, 경첩 두 점 포함
    #o3d.visualization.draw_geometries([ground_removed_pcd, sphere1, sphere2, sphere3, sphere4, obb])

    # 다시 카메라 30도 만큼 되돌려놓음 (카메라 좌표계로 원위치)
    theta1 = np.deg2rad(30)
    cam_x_rotation_2=np.array([[1, 0, 0], [0, np.cos(theta1), -np.sin(theta1)], [0, np.sin(theta1), np.cos(theta1)]])

    LeftFront=cam_x_rotation_2@LeftFront_bef
    RightFront=cam_x_rotation_2@RightFront_bef
    LeftRear=cam_x_rotation_2@LeftRear_bef
    RightRear=cam_x_rotation_2@RightRear_bef

    # boom 좌표계로 이동
    LeftFront_t=coordinate_transfer(LeftFront[0],LeftFront[1],LeftFront[2])
    RightFront_t=coordinate_transfer(RightFront[0],RightFront[1],RightFront[2])
    LeftRear_t=coordinate_transfer(LeftRear[0],LeftRear[1],LeftRear[2])
    RightRear_t=coordinate_transfer(RightRear[0],RightRear[1],RightRear[2])

    # Truck_point_1 은 지평에 평행한 4점 (30도 위로 돌렸을 때 기준임. y 값이 비슷해야됨)
    Truck_Point_1=np.vstack((LeftFront_bef,RightFront_bef,LeftRear_bef,RightRear_bef))
    # Truck_point_2 는 카메라 좌표계로 돌렸을때 네 점 (Pointcloud 찍으면 이렇게 나옴)
    Truck_Point_2=np.vstack((LeftFront,RightFront,LeftRear,RightRear))
    # Truck_point_3 은 붐 좌표계로 돌린 거
    Truck_Point_3=np.vstack((LeftFront_t,RightFront_t,LeftRear_t,RightRear_t))

    #print('Truckpoint : LF RF LR RR - before \n',Truck_Point_1)
    #print('Truckpoint : LF RF LR RR - after \n',Truck_Point_2)
    #print('Truckpoint : LF RF LR RR - transfered \n',Truck_Point_3)

    Truck_Point_List_auto=Truck_Point_3.tolist()

    return Truck_Point_List_auto


def coordinate_transfer(x,y,z):
    theta_t=np.deg2rad(30)
    x1=x
    z1=-z*np.cos(theta_t) + y*np.sin(theta_t) + 1.190
    y1=z*np.sin(theta_t) + y*np.cos(theta_t)+1.184

    coord_t=np.array([x1,y1,z1])
    return coord_t


# 함수 정의만 해서는 바로 안 나오니 실행
#pcdfile='pre-data/tester/test7.pcd'
#truck_pointcloud(pcdfile)
