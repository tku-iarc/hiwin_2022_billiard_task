######import######

from calendar import c
from curses.ascii import VT
from glob import glob
from operator import mod
import numpy as np
from ctypes import *
import time
import rospy
import cv2
import math
import os
import enum
import pyrealsense2 as rs
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
import configparser

import YOLO_Detect

######end######

######setting parameter######

DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0

error_x = -0.2
error_y = -0.3
pic_height_1 = 49.3
pic_height_2 = 18.3
pic_pos_x = 0.0
pic_pos_y = 36.8
pic_pos_z = pic_height_1

######end######

######global 變數######

global BMX, BMY, BMZ, CAMZ
global pic_OB
global saveImage_key, my_data

global H1, H2, H3, H4, H5, H6  # 洞口

global gHole
gHole = [0, 0]  # 初始化 gHole

global M_On_Table  # 母球是否在桌上的狀態
M_On_Table = 0

global loading_key
loading_key = 0

global billiard_radius
billiard_radius = 1.75  # 球半徑 = 1.75cm

global M
M = [0, 0]  # 母球 M=(Mx,My)

global C
C = [0, 0]  # 子球

global min_dif  # 角度最小差值 
min_dif = 0

global num_hole  # 目標洞口 洞口編號 1:左下 2:左上 3:中上 4:右上 5:右下 6:中下
num_hole = 0     

# global Hole_info    # 建立一個字典 洞號 對應 洞口座標
# Hole_info = {1: H1, 2: H2, 3: H3, 4: H4, 5: H5, 6: H6 }

global is_move  # 手臂是否有動作
is_move = False

global V, V1, V2
V = [0, 0]
V1 = [0, 0]
V2 = [0, 0]

global target_angle # ver9中的VT

global all_ball # 存所有球的陣列
all_ball = []

global button, shoot
shoot = 255
button = 0

######end######

######arm state######

Arm_state_flag = 0
Strategy_flag = 0
Sent_data_flag = 1

class Arm_status(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6

######end######


######Realsense#######

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)

######end######


######手臂移動函式######

def PTP_Move(X, Y, Z, pitch, roll, yaw, speed=50, acceleration=20):
    print("PTP Move ...")
    PTP_XYZ   = [X, Y, Z, pitch, roll, yaw]                 # XYZABC
    C_PTP_XYZ = (c_double * len(PTP_XYZ))(*PTP_XYZ)         # C Array
    modbus.PTP(1, speed, acceleration, 1, 0, C_PTP_XYZ)
    modbus.Arm_State_REGISTERS()
    time.sleep(0.2)
    modbus.Arm_State_REGISTERS()
    modbus.Arm_State_REGISTERS()

    while 1:
        frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
    print("END PTP Move")

######end######


######計算球洞######

def rqt_callback():
    #rospy.loginfo("""Reconfigure Request: {BMX}, {BMY}, {BMZ}, {BMX_1}, {BMY_1}""".format(**config))
    global BMX
    #BMX = config.BMX
    BMX = -33.9
    global BMY
    #BMY = config.BMY
    BMY = 28.28
    global BMZ
    #BMZ = config.BMZ
    BMZ = -22.4
    global BMX_1
    #BMX_1 = config.BMX_1
    BMX_1 = 26.05
    global BMY_1
    #BMY_1 = config.BMY_1
    BMY_1 = 36.59   
    print('BMXBMYBMZBMX1BMY1: ',BMX, ' ',BMY, ' ',BMZ, ' ',BMX_1, ' ',BMY_1)  # 因應 BM 找出球洞 相對位置

    TCC = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                [1,  0]])
    TC = np.array([[0, 1],    #+時針旋轉的旋轉矩陣
                [-1,  0]])

    B2B= np.array([BMX_1-BMX,BMY_1-BMY])    #從BM指向BM_1的向量                       
    B2BL = np.linalg.norm(B2B)              #B2B向量的長度
    Uv = B2B / B2BL                         #B2B向量的單位向量  
    Uu = TCC.dot(Uv)                      #B2B向量逆時針90度旋轉的單位向量

    global H1
    H1 = [BMX + 4.1*Uv[0] + 4.1*Uu[0], BMY + 4.1*Uv[1] + 4.1*Uu[1]]    #洞H1 左下
    global H2
    H2 = [BMX + 4.1*Uv[0] + 33.27*Uu[0], BMY + 4.1*Uv[1] + 33.27*Uu[1]]   #洞H2 左上
    global H3
    H3 = [((BMX + BMX_1)/2) + 33.27*Uu[0], ((BMY + BMY_1)/2) + 33.27*Uu[1]]   #洞H3 中上
    global H4
    H4 = [BMX_1 - 4.1*Uv[0] + 33.27*Uu[0], BMY_1 - 4.1*Uv[1] + 33.27*Uu[1]]   #洞H4 右上
    global H5
    H5 = [BMX_1 - 4.1*Uv[0] + 4.1*Uu[0], BMY_1 - 4.1*Uv[1] + 4.1*Uu[1]]   #洞H5 右下
    global H6
    H6 = [(BMX + BMX_1)/2 + 4.1*Uu[0], (BMY + BMY_1)/2 + 4.1*Uu[1]]    #洞H6 中下  
    print('HHHHHHHHHHHHHHHHHHH', H1,' ',H2,' ',H3,' ',H4,' ',H5,' ',H6)

######end######


######根據球數選擇策略######
def get_str_mid(data):
    global loading_key, my_data
    #print(data.bounding_boxes)
    if loading_key == 1:
        my_data = data
        ball_quantite = curve()
        global M  #zero ball
        global M_On_Table
        #print('MMM: ', M)
        if (M == [0.0, 0.0]) :
            M_On_Table = 0
            print("shit")
            print(M[0],M[1])
        else :
            print(M[0],M[1])
            loading_key = 0
            open=0
            if ball_quantite >= 10:
                print('early')
                early_stage()
            elif 3 < ball_quantite <= 9:
                print('middle')
                middle_stage()
            else:
                print('final')
                final_stage()
            M_On_Table = 1 

######end######


######find cueball######
def find_cueball(data):
    global  A, pic_pos_x, pic_pos_y, pic_pos_z
 #=====================================================================
    c_x = 643.47548083
    c_y = 363.67742746
    f_x = 906.60886808
    f_y = 909.34831447
    k_1 = 0.16962942
    k_2 = -0.5560001
    p_1 = 0.00116353
    p_2 = -0.00122694
    k_3 = 0.52491878
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    # dist_coef = np.array([0, 0, 0, 0, 0])
    z_world = pic_pos_z
    for item in data:
        if item[0] == 'M':
            mx = float(item[2] + item[4])/2
            my = float(item[3] + item[5])/2
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

            CAM_X = (img_pos[0] - c_x) * z_world / f_x
            CAM_Y = (img_pos[1] - c_y) * z_world / f_y

            A[0:3, 0:3] = A_mat
                        
            CAM_XY = np.mat([[CAM_X],[CAM_Y],[0],[1]])
            END_XY = A * CAM_XY
            END_X = END_XY[0, 0]
            END_Y = END_XY[1, 0]
            BASE_X = END_Y + pic_pos_x
            BASE_Y = END_X + pic_pos_y

            M = [BASE_X , BASE_Y]
            break

    return M
        
######end######


######將平面座標轉換成手臂面座標 + 計算球數######

def curve():
    global all_ball, my_data, M, A, pic_pos_x, pic_pos_y, pic_pos_z
 #=====================================================================
    c_x = 643.47548083
    c_y = 363.67742746
    f_x = 906.60886808
    f_y = 909.34831447
    k_1 = 0.16962942
    k_2 = -0.5560001
    p_1 = 0.00116353
    p_2 = -0.00122694
    k_3 = 0.52491878
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    # dist_coef = np.array([0, 0, 0, 0, 0])
    z_world = pic_pos_z
 #---------------------------------------------------------------------
    counter = 0
    dis_BaH = np.zeros(6) 
    for item in my_data:
        # print(item)
        if float(item[1]) >= 0.7: #信心值大於預期目標 才做分析 取得該球中心座標
            exclusion_ball = 0
            # -----取中心座標-----
            mx = float(item[2] + item[4])/2
            my = float(item[3] + item[5])/2
 #=====================================================================
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

            CAM_X = (img_pos[0] - c_x) * z_world / f_x
            CAM_Y = (img_pos[1] - c_y) * z_world / f_y

            A[0:3, 0:3] = A_mat
                        
            CAM_XY = np.mat([[CAM_X],[CAM_Y],[0],[1]])
            END_XY = A * CAM_XY
            END_X = END_XY[0, 0]
            END_Y = END_XY[1, 0]
            BASE_X = END_Y + pic_pos_x
            BASE_Y = END_X + pic_pos_y

            global H1,H2,H3,H4,H5,H6
            global billiard_radius
            H = [H1,H2,H3,H4,H5,H6]

            if(BASE_X<-31 and BASE_Y>57):       #左上洞口       #去除洞口誤判成球
                exclusion_ball = 1
            elif(BASE_X<-31 and BASE_Y<29.5):     #左下洞口
                exclusion_ball = 1
            elif(BASE_X>32 and BASE_Y>57):     #右上洞口
                exclusion_ball = 1
            elif(BASE_X>32 and BASE_Y<30):     #右下洞口
                exclusion_ball = 1
            elif(BASE_X>-1 and BASE_X<1 and BASE_Y>58):     #中上洞口
                exclusion_ball = 1
            elif(BASE_X>-1 and BASE_X<1 and BASE_Y<30):     #中下洞口
                exclusion_ball = 1
            else:
                exclusion_ball = 0

            for j in range(6):
                 dis_BaH[j] = (((H[j][0] - BASE_X)**2 + (H[j][1] - BASE_Y)**2)**0.5)
                 if dis_BaH[j] <= 1*billiard_radius :
                    print('i see a ball in hole~~~~~~~~~~~~~~~~~~~~~')
                
                    exclusion_ball = 1
            result = np.where(dis_BaH == np.amin(dis_BaH))          #result代表dis_BaH的最小值的位置
            if exclusion_ball != 1:
                all_ball.append([str(item[0]), float(item[1]), BASE_X, BASE_Y, dis_BaH[result[0][0]]])
                counter += 1
    #print(' s', counter,'t')
    return counter

######end######

######開球######
###可能有用先留著###

#def open_pool():                                        
    position =  [M[0]-2, M[1]-2, -0.8, 180, 0, 180]
    robot_ctr.Step_AbsPTPCmd(position)
    shoot==255

######end######

######球數大於10的策略######
###all_ball([str(item.Class), float(item.probability), BASE_X, BASE_Y, dis_BaH[result[0][0]]])###
def early_stage():
    global C, M, pic_OB, all_ball
    global ball_sorted, kissball
    global shoot

    all_ball.sort(key = lambda x: x[4])      #依與洞口的距離排序 #lambda x:x[4]代表x=x[4]，x用all_ball[]帶入 #這邊就是依據all_ball[4]從小排列
    ball_sorted = []
    for items in all_ball:
        if items[4] != 0.0 and (((M[0]-billiard_radius) > items[2] or items[2] > (M[0]+billiard_radius)) or ((M[1]-billiard_radius) > items[3] or items[3] > (M[1]+billiard_radius))):
            ball_sorted.append(items)          #排除把母球辨識成其他球之母球資訊及空資訊
    # print(ball_sorted)
    target_contener = [[],[],[],[]]             #分類子球[[好球], [大角度球], [干擾], [大角度干擾]]
    for value in ball_sorted:
        if( value[0] != 'M'):
            C = [value[2], value[3]]            #預設C(子球)
            dis_CM = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)         #母球與子球的距離
            OB1_ball = check_front(0)          #判斷母球和子球間是否有干擾球
            OB2_ball = check_back()            #判斷洞口和子球間是否有干擾球
            # angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB1_ball == 0 and OB2_ball == 0:
                kissball = 0; angle = MBallCheck(0)
                if angle == 0:                 #[好球]
                    # print('good target~~~~~~')
                    target_contener[0].append([value[0], value[2], value[3], dis_CM])
                # elif angle == 1:                 #[大角度球]
                #     print('not good angle 1')
                #     target_contener[1].append([value[0], value[2], value[3]])
                elif angle == -1:
                    z=0
                    # print('angle over 90 degree 1')
            elif OB1_ball == 1 and OB2_ball == 0:              #-------------obstacle should be 1!now----------
                kissball = 1; angle = MBallCheck(0)
                if angle == 0:                 #[干擾]
                    # print('looking for other ball-----------')
                    target_contener[2].append([value[0], value[2], value[3], dis_CM, pic_OB[0][2]])
                # elif angle == 1:                # [大角度干擾]]
                #     print('not good angle 2')
                #     target_contener[3].append([value[0], value[2], value[3], pic_OB[0][2]])
                elif angle == -1:
                    z=0
                    # print('angle over 90 degree 2')
            else:
                z= 0
                # print('too many obstacle!!!!!!!')
    if len(target_contener[0]) >= 1:
        target_contener[0].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------', target_contener[0][0][0])
        print('no obstacle!!')
        # print(target_contener[0])
        C = [target_contener[0][0][1], target_contener[0][0][2]]
        shoot = 150
        kissball = 0
    elif len(target_contener[0]) < 1 and len(target_contener[2]) >= 1:
        target_contener[2].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------but', target_contener[2][0][0])
        print('there is an obstacle----------',target_contener[2][0][4])
        C = [target_contener[2][0][1], target_contener[2][0][2]]
        shoot = 255
        kissball = 1
    else:
        luckeyball()
        shoot = 255
        kissball = 3

######end######

######球數介於9~4顆的策略######

def middle_stage():
    global C, M, pic_OB, all_ball
    global ball_sorted, kissball
    global shoot

    all_ball.sort(key = lambda x: x[4])      #依與洞口的距離排序
    ball_sorted = []
    for items in all_ball:
        if items[4] != 0.0 and (((M[0]-billiard_radius) > items[2] or items[2] > (M[0]+billiard_radius)) or ((M[1]-billiard_radius) > items[3] or items[3] > (M[1]+billiard_radius))):
            ball_sorted.append(items)          #排除把母球辨識成其他球之母球資訊及空資訊
    #print(ball_sorted)
    target_contener = [[],[],[],[]]             #分類子球[[好球], [大角度球], [干擾], [大角度干擾]]
    for value in ball_sorted:
        if( value[0] != '0'):
            C = [value[2], value[3]]            #預設C(子球)
            dis_CM = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)         #母球與子球的距離
            OB1_ball = check_front(0)          #判斷母球和子球間是否有干擾球
            OB2_ball = check_back()             #判斷洞口和子球間是否有干擾球
            # angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB1_ball == 0 and OB2_ball == 0:
                kissball = 0; angle = MBallCheck(0)
                if angle == 0:                 #[好球]
                    # print('good target~~~~~~')
                    target_contener[0].append([value[0], value[2], value[3], dis_CM])
                elif angle == 1:                 #[大角度球]
                    # print('not good angle 1')
                    target_contener[1].append([value[0], value[2], value[3]])
                elif angle == -1:
                    z=0
                    # print('angle over 90 degree 1')
            elif OB1_ball == 1 and OB2_ball == 0:              #-------------obstacle should be 1!now----------
                kissball = 1; angle = MBallCheck(0)
                if angle == 0:                 #[干擾]
                    # print('looking for other ball-----------')
                    target_contener[2].append([value[0], value[2], value[3], dis_CM, pic_OB[0][2]])
                elif angle == 1:                # [大角度干擾]]
                    # print('not good angle 2')
                    target_contener[3].append([value[0], value[2], value[3], pic_OB[0][2]])
                elif angle == -1:
                    z=0
                    # print('angle over 90 degree 2')
            else:
                z= 0
                # print('too many obstacle!!!!!!!')
    if len(target_contener[0]) >= 1:                 #[好球]
        target_contener[0].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------', target_contener[0][0][0])
        print('no obstacle!!')
        C = [target_contener[0][0][1], target_contener[0][0][2]]          #確定C(子球)
        shoot = 255
        kissball = 0
    elif len(target_contener[0]) < 1 and len(target_contener[2]) >= 1:                 #[干擾]
        target_contener[2].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------but', target_contener[2][0][0])
        print('there is an obstacle----------',target_contener[2][0][4])
        C = [target_contener[2][0][1], target_contener[2][0][2]]
        shoot = 255
        kissball = 1
    elif len(target_contener[0]) < 1 and len(target_contener[2]) < 1 and len(target_contener[3]) >= 1:                 # [大角度干擾]]
        target_contener[3].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------not good angle', target_contener[3][0][0])
        #print('there is an obstacle----------',target_contener[3][0][4])
        C = [target_contener[3][0][1], target_contener[3][0][2]]
        shoot = 255
        kissball = 1
    elif len(target_contener[0]) < 1 and len(target_contener[2]) < 1 and len(target_contener[3]) < 1 and len(target_contener[1]) >= 1:                 #[大角度球]
        #target_contener[1].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------and', target_contener[1][0][0])
        print('not good angle & no obstacle!!----------')
        C = [target_contener[1][0][1], target_contener[1][0][2]]
        shoot = 150
        kissball = 0
    else:
        print('000000')
        C = [ball_sorted[0][2], ball_sorted[0][3]]        #往離洞口最近的球之方向打
        shoot = 255
        kissball = 0

######end######

######球數小於4顆的策略######

def final_stage():
    global C, M, pic_OB, all_ball
    global ball_sorted, kissball
    global shoot

    all_ball.sort(key = lambda x: x[4])      #依與洞口的距離排序
    ball_sorted = []
    for items in all_ball:
        if items[4] != 0.0 and (((M[0]-0.7*billiard_radius) > items[2] or items[2] > (M[0]+0.7*billiard_radius)) or ((M[1]-0.7*billiard_radius) > items[3] or items[3] > (M[1]+0.7*billiard_radius))):
            ball_sorted.append(items)          #排除把母球辨識成其他球之母球資訊及空資訊
            #print(ball_sorted)
    target_contener = [[],[],[],[]]             #分類子球[[好球], [大角度球], [干擾], [大角度干擾]]
    for value in ball_sorted:
        if( value[0] != '0'):
            C = [value[2], value[3]]            #預設C(子球)
            dis_CM = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)         #母球與子球的距離
            OB1_ball = check_front(0)          #判斷母球和子球間是否有干擾球
            OB2_ball = check_back()             #判斷洞口和子球間是否有干擾球
            #angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB1_ball == 0 and OB2_ball == 0:
                kissball = 0; 
                angle = MBallCheck(2)
                if angle == 0:                 #[好球]
                    print('good target~~~~~~')
                    target_contener[0].append([value[0], value[2], value[3], dis_CM])
                elif angle == 1:                 #[大角度球]
                    print('not good angle 1')
                    target_contener[1].append([value[0], value[2], value[3]])
                elif angle == -1:
                    z=0
                    print('angle over 90 degree 1')
            elif OB1_ball == 1 and OB2_ball == 0:              #-------------obstacle should be 1!now----------
                kissball = 1; 
                angle = MBallCheck(2)
                if angle == 0:                 #[干擾]
                    print('looking for other ball-----------')
                    target_contener[2].append([value[0], value[2], value[3], dis_CM, pic_OB[0][2]])
                elif angle == 1:                # [大角度干擾]]
                    print('not good angle 2')
                    target_contener[3].append([value[0], value[2], value[3], pic_OB[0][2]])
                elif angle == -1:
                    z=0
                    print('angle over 90 degree 2')
            else:
                z= 0
                print('too many obstacle!!!!!!!')
                shoot = 150
    if len(target_contener[0]) >= 1:                 #[好球]
        target_contener[0].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------', target_contener[0][0][0])
        print('no obstacle!!')
        C = [target_contener[0][0][1], target_contener[0][0][2]]          #確定C(子球)
        shoot = 150
        kissball = 0
    elif len(target_contener[0]) < 1 and len(target_contener[2]) >= 1:                 #[干擾]
        target_contener[2].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------but', target_contener[2][0][0])
        print('there is an obstacle----------',target_contener[2][0][4])
        C = [target_contener[2][0][1], target_contener[2][0][2]]
        shoot = 255
        kissball = 1
    elif len(target_contener[0]) < 1 and len(target_contener[2]) < 1 and len(target_contener[3]) >= 1:                 # [75度干擾]]
        target_contener[3].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------not good angle', target_contener[3][0][0])
        print('there is an obstacle----------',target_contener[3][0][4])
        C = [target_contener[3][0][1], target_contener[3][0][2]]
        shoot = 255
        kissball = 1
    elif len(target_contener[0]) < 1 and len(target_contener[2]) < 1 and len(target_contener[3]) < 1 and len(target_contener[1]) >= 1:                 #[75度球]
        #target_contener[1].sort(key = lambda x: x[3])        #依角度和母球與子球之距離 從小排序
        print('success to set terget! and target is---------and', target_contener[1][0][0])
        print('not good angle & no obstacle!!----------')
        C = [target_contener[1][0][1], target_contener[1][0][2]]
        shoot = 150
        kissball = 0
    else:
        # boundsball()
        gwun()
        shoot = 150

######end######

######判斷母球和子球間是否有干擾球######

def check_front(A):
    global M, C , OB_1, V, pic_OB
    global ball_sorted

    pic_OB = [[0,0,''] for i in range(2)]           #干擾球宣告用
    counter = 0
    for item in ball_sorted:             #--------- searching of corect obstcle-----------
        if ((item[0] != '0') and ([item[2], item[3]] != C) and (counter < 2)):
            OB_1 = [item[2], item[3]]           #子球'前方'的干擾球

            if A == 0:
                check_angle()           #找出角度最好的洞口
                VCtoH1= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
                chL1 = np.linalg.norm(VCtoH1)                       #CH向量的長度
                UVch1 = VCtoH1 / chL1                                #CH向量的單位向量  
                V = [ (C[0] - (2*(billiard_radius)) * UVch1[0]), (C[1] - (2*(billiard_radius)) * UVch1[1]) ]   
            
            VCtoH1V= np.array([(gHole[0]-C[0])*(-1),gHole[1]-C[1]])    #從C指向H的向量                       
            chL1V = np.linalg.norm(VCtoH1V)                       #CH向量的長度
            UVch1V = VCtoH1V / chL1V                                #CH向量的單位向量
            d = 2.3*(billiard_radius) * UVch1V       

            w = np.array([V[0] - M[0], V[1] - M[1]])           #母球與虛擬球之向量
            u = np.array([OB_1[0] - M[0], OB_1[1] - M[1]])           #母球與干擾球之向量
            L = abs(np.cross(w, u) / np.linalg.norm(w))           #用面積(平面向量外積)除於底(母球與虛擬球之向量)得到高 = 母球與虛擬球的連線到干擾球之距離
            if M[1] >= V[1] :           #洞口與虛擬球的分佈
                if M[0] >= V[0] :
                    if L <= (2.3 * billiard_radius) and OB_1[0] <= M[0]+d[0] and OB_1[0] >= V[0]-d[0] and OB_1[1] <= M[1]+d[1] and OB_1[1] >= V[1]-d[1]:
                        pic_OB[counter] =  [item[2], item[3], item[0]]
                        counter += 1
                if M[0] < V[0] :
                    if L <= (2.3 * billiard_radius) and OB_1[0] >= M[0]-d[0] and OB_1[0] <= V[0]+d[0] and OB_1[1] <= M[1]+d[1] and OB_1[1] >= V[1]-d[1]:
                        pic_OB[counter] =  [item[2], item[3], item[0]]
                        counter += 1
            elif M[1] < V[1] :
                if M[0] >= V[0] :
                    if L <= (2.3 * billiard_radius) and OB_1[0] <= M[0]+d[0] and OB_1[0] >= V[0]-d[0] and OB_1[1] >= M[1]-d[1] and OB_1[1] <= V[1]+d[1]:
                        pic_OB[counter] =  [item[2], item[3], item[0]]
                        counter += 1
                if M[0] < V[0] :
                    if L <= (2.3 * billiard_radius) and OB_1[0] >= M[0]-d[0] and OB_1[0] <= V[0]+d[0] and OB_1[1] >= M[1]-d[1] and OB_1[1] <= V[1]+d[1]:
                        pic_OB[counter] =  [item[2], item[3], item[0]]
                        counter += 1
        elif (counter >= 2):           #有兩顆以上干擾球
            break
    return counter

######end######

######判斷洞口和子球間是否有干擾球######

def check_back():
    global gHole
    global C, OB_2
    global ball_sorted

    VCtoH1V= np.array([(gHole[0]-C[0])*(-1),gHole[1]-C[1]])    #從C指向H的向量                       
    chL1V = np.linalg.norm(VCtoH1V)                       #CH向量的長度
    UVch1V = VCtoH1V / chL1V                                #CH向量的單位向量
    d = 2.3*(billiard_radius) * UVch1V
    counter = 0
    for item in ball_sorted:             #--------- searching of corect obstcle-----------
        if ((item[0] != '0') and ([item[2], item[3]] != C) and (counter < 2)):
            OB_2 = [item[2], item[3]]             #子球'後方'的干擾球
            # check_angle()           #找出角度最好的洞口

            w = np.array([gHole[0] - C[0], gHole[1] - C[1]])       #子球與洞口之向量
            u = np.array([OB_2[0] - C[0], OB_2[1] - C[1]])        #子球與干擾球之向量
            L = abs(np.cross(w, u) / np.linalg.norm(w))           #子球與洞口的連線到干擾球之距離
            if gHole[1] >= C[1] :
                if gHole[0] >= C[0] :
                    if L <= (2.3 * billiard_radius) and OB_2[0] <= gHole[0]+d[0] and OB_2[0] >= C[0]-d[0] and OB_2[1] <= gHole[1]+d[1] and OB_2[1] >= C[1]-d[1]:
                        counter += 1
                if gHole[0] < C[0] :
                    if L <= (2.3 * billiard_radius) and OB_2[0] >= gHole[0]-d[0] and OB_2[0] <= C[0]+d[0] and OB_2[1] <= gHole[1]+d[1] and OB_2[1] >= C[1]-d[1]:
                        counter += 1
            elif gHole[1] < C[1] :
                if gHole[0] >= C[0] :
                    if L <= (2.3 * billiard_radius) and OB_2[0] <= gHole[0]+d[0] and OB_2[0] >= C[0]-d[0] and OB_2[1] >= gHole[1]-d[1] and OB_2[1] <= C[1]+d[1]:
                        counter += 1
                if gHole[0] < C[0] :
                    if L <= (2.3 * billiard_radius) and OB_2[0] >= gHole[0]-d[0] and OB_2[0] <= C[0]+d[0] and OB_2[1] >= gHole[1]-d[1] and OB_2[1] <= C[1]+d[1]:
                        counter += 1
        elif (counter >= 2):           #有兩顆以上干擾球
            break
    return counter

######end######

######找與子球最小角度之洞口######

def check_angle():
    global H1,H2,H3,H4,H5,H6
    global gHole
    
    check_hole = [] #local variable 存每個洞口 跟 子球 母球的 角度差值

    global Hole_info
    Hole_info = {
        1: H1,    
        2: H2,    
        3: H3,    
        4: H4,    
        5: H5,
        6: H6 
        }
    
    global min_dif, num_hole
    global M, C                           
    VMtoC= np.array([C[0]-M[0],C[1]-M[1]]).reshape(2, 1)        #母到子的向量
    Lmc = np.linalg.norm(VMtoC)                                 #MC向量長

    Vmh1 = np.array([H1[0]-M[0],H1[1]-M[1]]).reshape(2, 1)      #母到洞的向量
    Tmh1 = Vmh1.transpose()                                     #轉置
    h1dot = np.dot(Tmh1, VMtoC)                                 #內積
    Lmh1 = np.linalg.norm(Vmh1)                                 #MH向量長
    Dmh1 = np.rad2deg(np.arccos(h1dot/(Lmc*Lmh1)))              #兩向量夾角
    
    Vmh2 = np.array([H2[0]-M[0],H2[1]-M[1]]).reshape(2, 1)
    Tmh2 = Vmh2.transpose()
    h2dot = np.dot(Tmh2, VMtoC)
    Lmh2 = np.linalg.norm(Vmh2)
    Dmh2 = np.rad2deg(np.arccos(h2dot/(Lmc*Lmh2)))             

    Vmh3 = np.array([H3[0]-M[0],H3[1]-M[1]]).reshape(2, 1)
    Tmh3 = Vmh3.transpose()
    h3dot = np.dot(Tmh3, VMtoC)
    Lmh3 = np.linalg.norm(Vmh3)
    Dmh3 = np.rad2deg(np.arccos(h3dot/(Lmc*Lmh3)))     

    Vmh4 = np.array([H4[0]-M[0],H4[1]-M[1]]).reshape(2, 1)
    Tmh4 = Vmh4.transpose()
    h4dot = np.dot(Tmh4, VMtoC)
    Lmh4 = np.linalg.norm(Vmh4)
    Dmh4 = np.rad2deg(np.arccos(h4dot/(Lmc*Lmh4)))

    Vmh5 = np.array([H5[0]-M[0],H5[1]-M[1]]).reshape(2, 1)
    Tmh5 = Vmh5.transpose()
    h5dot = np.dot(Tmh5, VMtoC)
    Lmh5 = np.linalg.norm(Vmh5)
    Dmh5 = np.rad2deg(np.arccos(h5dot/(Lmc*Lmh5)))                      

    Vmh6 = np.array([H6[0]-M[0],H6[1]-M[1]]).reshape(2, 1)
    Tmh6 = Vmh6.transpose()
    h6dot = np.dot(Tmh6, VMtoC)
    Lmh6 = np.linalg.norm(Vmh6)
    Dmh6 = np.rad2deg(np.arccos(h6dot/(Lmc*Lmh6)))    
    
    check_hole = ['0', 
                Dmh1,
                Dmh2, 
                Dmh3,
                Dmh4,
                Dmh5,
                Dmh6 ]

    min_dif = min(check_hole[1:7])       #確認最小差值
    num_hole = check_hole.index(min_dif)   
    gHole = Hole_info.get(num_hole)

######end######

######判斷子球與洞口的角度是否太大######
###還不懂###

def MBallCheck(A):
    global kissball
    global M, C , OB_1, gHole
    global billiard_radius, H1, H5
 #============method1:check by area==============================================      
    if A == 0:
        Degrees = 45
        Radians = math.radians(Degrees)

        VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        chL = np.linalg.norm(VCtoH)                       #CH向量的長度
        UVch = VCtoH / chL                                #CH向量的單位向量  
        V = [ (C[0]-2*billiard_radius*UVch[0]), (C[1]-2*billiard_radius*UVch[1]) ]    #虛擬球V的座標

        VE = [ (V[0]-UVch[0]), (V[1]-UVch[1]) ]           #虛擬球延伸的座標點

        TCC = np.array([[math.cos(Radians), -math.sin(Radians)],    #逆時針旋轉的旋轉矩陣
                    [math.sin(Radians),  math.cos(Radians)]])

        TC = np.array([[math.cos(-Radians), -math.sin(-Radians)],    #逆時針旋轉的旋轉矩陣
                        [math.sin(-Radians),  math.cos(-Radians)]])

        VtoVE = np.array([(VE[0]-V[0]),                   #V指向VE的向量
                        (VE[1]-V[1])])

        TCC45 = TCC.dot(VtoVE)                            #兩者內積後得到逆時針選轉45度的deltaX跟deltaY
        TC45 = TC.dot(VtoVE)   
        Hdown = np.array([0, -1])        #往下指的方向向量
        uT = Hdown.transpose()    #轉置矩陣
        HdotV = np.dot(uT, VtoVE)                      #向下的向量和V到VE的向量的內積
        VVEL = np.linalg.norm(VtoVE)                       #V到VE向量的長度
        Angle = float(np.rad2deg(np.arccos(HdotV/VVEL)))   #求夾角

        MS = TCC45[1]/TCC45[0]                            #deltaX分之deltaY得到直線斜率
        NMS = -1/MS                                         #乘上富號得到與其垂直也就是順時旋轉的直線斜率

        TCC18 = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                        [1,  0]])
        TCC180 = TCC18.dot(VtoVE)
        CHS = TCC180[1]/TCC180[0]

        LTCC = MS  * (M[0]-V[0]) - (M[1]-V[1])            #將M帶入直線方程式判斷
        LTC  = NMS * (M[0]-V[0]) - (M[1]-V[1])
        L180 = CHS * (M[0]-V[0]) - (M[1]-V[1])

        if MS < 0:
            LTCC = LTCC * -1
        if NMS < 0:
            LTC = LTC * -1
        if CHS < 0:
            L180 = L180 * -1

        if Angle < 45 and LTCC < 0 and LTC > 0:
            return 0
        elif V[0] > gHole[0] :
            if 135 > Angle > 45 and LTCC > 0 and LTC > 0:
                return 0
            elif L180 > 0 :
                return 1
        elif V[0] < gHole[0] :
            if 135 > Angle > 45 and LTCC < 0 and LTC < 0:
                return 0
            elif L180 < 0 :
                return 1
        elif Angle > 135 and LTCC > 0 and LTC < 0:
            return 0
        else:
            return -1
    
    elif A == 2:
        Degrees = 45
        Radians = math.radians(Degrees)

        VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        chL = np.linalg.norm(VCtoH)                       #CH向量的長度
        UVch = VCtoH / chL                                #CH向量的單位向量  
        V = [ (C[0]-2*billiard_radius*UVch[0]), (C[1]-2*billiard_radius*UVch[1]) ]    #虛擬球V的座標

        VE = [ (V[0]-UVch[0]), (V[1]-UVch[1]) ]           #虛擬球延伸的座標點
        
        #===================TURN45===================================================================
        TCC = np.array([[math.cos(Radians), -math.sin(Radians)],    #逆時針旋轉的旋轉矩陣
                    [math.sin(Radians),  math.cos(Radians)]])

        TC = np.array([[math.cos(-Radians), -math.sin(-Radians)],    #逆時針旋轉的旋轉矩陣
                        [math.sin(-Radians),  math.cos(-Radians)]])

        VtoVE = np.array([(VE[0]-V[0]),                   #V指向VE的向量
                        (VE[1]-V[1])])

        TCC45 = TCC.dot(VtoVE)                            #兩者內積後得到逆時針選轉45度的deltaX跟deltaY
        TC45 = TC.dot(VtoVE)   
        Hdown = np.array([0, -1])        #往下指的方向向量
        uT = Hdown.transpose()    #轉置矩陣
        HdotV = np.dot(uT, VtoVE)                      #向下的向量和V到VE的向量的內積
        VVEL = np.linalg.norm(VtoVE)                       #V到VE向量的長度
        Angle = float(np.rad2deg(np.arccos(HdotV/VVEL)))   #求夾角

        MS = TCC45[1]/TCC45[0]                            #deltaX分之deltaY得到直線斜率
        NMS = -1/MS                                         #乘上富號得到與其垂直也就是順時旋轉的直線斜率

        #===================TURN75===================================================================
        TCC75 = np.array([[math.cos(math.radians(75)), -math.sin(math.radians(75))],    #逆時針旋轉的旋轉矩陣
                    [math.sin(math.radians(75)),  math.cos(math.radians(75))]])

        TC75 = np.array([[math.cos(-math.radians(75)), -math.sin(-math.radians(75))],    #逆時針旋轉的旋轉矩陣
                        [math.sin(-math.radians(75)),  math.cos(-math.radians(75))]])
        MTCC75 = TCC75.dot(VtoVE)                            #兩者內積後得到逆時針選轉45度的deltaX跟deltaY
        MTC75 = TC75.dot(VtoVE)   

        MS75 = MTCC75[1]/MTCC75[0]                            #deltaX分之deltaY得到直線斜率
        NMS75 = MTC75[1]/MTC75[0]                                         #乘上富號得到與其垂直也就是順時旋轉的直線斜率


        LTCC   = MS  * (M[0]-V[0]) - (M[1]-V[1])            #將M帶入直線方程式判斷
        LTC    = NMS * (M[0]-V[0]) - (M[1]-V[1])
        LTCC75 = MS75  * (M[0]-V[0]) - (M[1]-V[1])            #將M帶入直線方程式判斷
        LTC75  = NMS75 * (M[0]-V[0]) - (M[1]-V[1])

        if MS < 0:
            LTCC = LTCC * -1
        if NMS < 0:
            LTC = LTC * -1
        if MS75 < 0:
            LTCC75 = LTCC75 * -1
        if NMS75 < 0:
            LTC75 = LTC75 * -1


        if Angle < 45 and LTCC < 0 and LTC > 0:
            return 0
        if Angle < 25 and LTCC75 < 0 and LTC75 > 0:
            return 1
        elif V[0] > gHole[0] :
            if 135 > Angle > 45 and LTCC > 0 and LTC > 0:
                return 0
            elif 155 > Angle > 25 and LTCC75 > 0 and LTC75 > 0:
                return 1
        elif V[0] < gHole[0] :
            if 135 > Angle > 45 and LTCC < 0 and LTC < 0:
                return 0
            elif 155 > Angle > 25 and LTCC75 < 0 and LTC75 < 0:
                return 1
        elif Angle > 135 and LTCC > 0 and LTC < 0:
            return 0
        elif Angle > 155 and LTCC75 > 0 and LTC75 < 0:
            return 0
        else:
            return -1
    
    elif A == 1:
 #============method2:check by angle==============================================
        VV2H= np.array([C[0]-M[0],C[1]-M[1]])                           
        vhL = np.linalg.norm(VV2H)        

        VV2M= np.array([H5[0]-H1[0],H5[1]-H1[1]])                          
        mvL = np.linalg.norm(VV2M)   

        TM = VV2M.transpose()    #轉置矩陣
        HdotM = np.dot(VV2H, TM)                    #V到VE向量的長度
        Angle = float(np.rad2deg(np.arccos(HdotM/(vhL*mvL))))   #求夾角
        return Angle

######end######

######luckey ball######

def luckeyball():
    global C, M, target_angle
    global ball_sorted
    global billiard_radius, gHole
    global H1,H2,H3,H4,H5,H6
    global kissball 

    CM = np.zeros(int(len(ball_sorted)))  #各子球與其他子球的距離和（判斷中心球）
    i = 0; ball_remoMM=[]
    for item in ball_sorted:
        if item[0] != '0':
            ball_remoMM.append(item)
    for value in ball_remoMM:
        C = [value[2], value[3]]     #預設C(子球)
        N = 0; dis_CM = 0
        for items in ball_remoMM:
            if ([items[2], items[3]] != C):
                K = [items[2], items[3]]                                      #抓的球
                dis_CM = (((C[0] - K[0])**2 + (C[1] - K[1])**2)**0.5)         #球的距離
                N += dis_CM
        CM[i] = N
        i += 1
    u = 0
    for ikey in CM:
        if ikey != 0:
            u += 1
    # print('u',u)
    w = 0
    CM_resu = np.zeros(u)
    for item in CM:
        # print('CM',CM)
        if item != 0:
            CM_resu[w] = item
            w += 1          #排除把母球辨識成其他球之母球資訊及空資訊
    result = np.where(CM_resu == np.amin(CM_resu))
    C =  [ball_remoMM[result[0][0]][2],ball_remoMM[result[0][0]][3]]
    # print(C)
    VVHSE= np.array([C[0]-M[0],C[1]-M[1]]) 
    VVup = np.array([0, 1]).reshape(2, 1)
    VuT = VVup.transpose()
    Vn = np.dot(VuT, VVHSE)
    VHSEL = np.linalg.norm(VVHSE)
    target_angle = float(np.rad2deg(np.arccos(Vn/VHSEL)))
    if(C[0]>M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
        target_angle = target_angle * -1
    print('come on jackpod$$$$$$$$$$')

######end######

######不懂######

def gwun():
    global C, M, V, target_angle, CVT
    global ball_sorted
    global billiard_radius, gHole
    global kissball, shoot
    global H1, H2, H3, H4, H5, H6
    H = [H1, H2, H3, H4, H5, H6]
    k = 0            
    for item in ball_sorted:
        if item[0] != '0':
            C = [item[2], item[3]]
            angle = MBallCheck(1) 
            if angle >= 135:                     #短邊
                if (H2[1]+H1[1])/2 >= C[1]:           #洞口與虛擬球的分佈
                    V = [ (C[0] + (32**0.5)/3* (billiard_radius)), (C[1] + 2/3 * (billiard_radius)) ] 
                    k = 1
                    break
                elif (H2[1]+H1[1])/2 < C[1] :
                    V = [ (C[0] + (32**0.5)/3* (billiard_radius)), (C[1] - 2/3 * (billiard_radius)) ] 
                    k = 1
                    break
            elif angle <= 45:                     #短邊
                if (H2[1]+H1[1])/2 >= C[1]:           #洞口與虛擬球的分佈
                    V = [ (C[0] - (32**0.5)/3* (billiard_radius)), (C[1] + 2/3 * (billiard_radius)) ] 
                    k = 1
                    break
                elif (H2[1]+H1[1])/2 < C[1] :
                    V = [ (C[0] - (32**0.5)/3* (billiard_radius)), (C[1] - 2/3 * (billiard_radius)) ] 
                    k = 1
                    break
            elif 45 < angle < 135 and M[1] <= C[1]:                     #長邊
                a = np.array([abs(H1[0]-C[0]), abs(H5[0]-C[0]), abs(H6[0]-C[0])])
                result = np.where(a == np.amin(a))
                gHole =  H[result[0][0]]
                if (H1 == gHole and C[0] <= H6[0]) or (H6 == gHole and C[0] > H6[0]):                               #CH向量的單位向量  
                    V = [ (C[0] + 2/3* (billiard_radius)), (C[1] - (32**0.5)/3 * (billiard_radius)) ] 
                    k = 1
                    break
                elif (H5 == gHole and C[0] >= H6[0]) or (H6 == gHole and C[0] < H6[0]):
                    V = [ (C[0] - 2/3* (billiard_radius)), (C[1] - (32**0.5)/3 * (billiard_radius)) ] 
                    k = 1
                    break
            elif 45 < angle < 135 and M[1] > C[1] :                     #長邊
                a = np.array([abs(H2[0]-C[0]), abs(H3[0]-C[0]), abs(H4[0]-C[0])])
                result = np.where(a == np.amin(a))
                gHole =  H[result[0][0]]
                if (H2 == gHole and C[0] <= H3[0]) or (H3 == gHole and C[0] > H3[0]):                               #CH向量的單位向量  
                    V = [ (C[0] + 2/3* (billiard_radius)), (C[1] + (32**0.5)/3 * (billiard_radius)) ] 
                    k = 1
                    break
                elif (H4 == gHole and C[0] >= H3[0]) or (H3 == gHole and C[0] < H3[0]):
                    V = [ (C[0] - 2/3* (billiard_radius)), (C[1] + (32**0.5)/3 * (billiard_radius)) ] 
                    k = 1
                    break
    OB1_ball = check_front(1)          #判斷母球和子球間是否有干擾球 
    if OB1_ball == 0 and k == 1:
        VHSE= np.array([V[0]-M[0],V[1]-M[1]]) 
        Vup = np.array([0, 1]).reshape(2, 1)
        uT = Vup.transpose()
        n = np.dot(uT, VHSE)
        HSEL = np.linalg.norm(VHSE)
        target_angle = float(np.rad2deg(np.arccos(n/HSEL)))
        if(V[0]>M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
            target_angle = target_angle * -1
        kissball = 2
    else:
        print('nothing')
        kissball = 0

######end######

######根據組合球球數計算擊球角度######

def GetStartEnd():
    global num_hole, gHole
    global kissball
    global V, M, C
    global target_angle,shoot

    if kissball == 3:
        print("target hole site is: ", gHole)
        print('lets see my shoot!!!!!!------->', target_angle)
    elif kissball == 2:
        print("target hole site is: ", gHole)
        print('try kickshot------->', target_angle)
        shoot=255
    elif kissball == 1:
        check_angle()
        print("target hole is 1: number ",num_hole,"hole")
        print("target hole site is: ", gHole)
        MOB_1C()
    elif kissball == 0:
        check_angle()
        print("target hole is 0: number",num_hole,"hole")
        print("target hole site is: ", gHole)
        #---------------計算子球後的虛擬球球心位置------------

        VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        chL = np.linalg.norm(VCtoH)                       #CH向量的長度
        UVch = VCtoH / chL                                #CH向量的單位向量  
        V = [ (C[0]-2 * (billiard_radius) * UVch[0]), (C[1]-2 * (billiard_radius) * UVch[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

        VHSE= np.array([V[0]-M[0],V[1]-M[1]])             #M指向V的向量
        Vup = np.array([0, 1]).reshape(2, 1)
        uT = Vup.transpose()
        n = np.dot(uT, VHSE)
        HSEL = np.linalg.norm(VHSE)
        target_angle = float(np.rad2deg(np.arccos(n/HSEL)))
        print ('target_angle:',target_angle)
        if(V[0]>M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
            target_angle = target_angle * -1       
        print ('normal hitball')      
        print ('normal hitball DEGREE :',target_angle)

######end######

######計算組合球的擊球角度######

def MOB_1C():
    global gHole
    global V1, V2
    global M, C, OB_1, target_angle

    #------------從洞口(gHole)推算子球(C)的虛擬球1(V1)-------------------
    print ('(OB_1)hit C')
    VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])                  #從C指向H的向量                       
    chL = np.linalg.norm(VCtoH)                                     #CH向量的長度
    UVch = VCtoH / chL                                              #CH向量的單位向量  
    V1 = [ (C[0]-2 * (billiard_radius) * UVch[0]), (C[1]-2 * (billiard_radius) * UVch[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 

    #------------從虛擬球1(V1)推算干擾球(OB_1)的虛擬球2(V2)-------------------
    VOtoV= np.array([V1[0]-OB_1[0],V1[1]-OB_1[1]])                        #從指向的向量                       
    ovL = np.linalg.norm(VOtoV)                                     #向量的長度
    UVov = VOtoV / ovL                                              #向量的單位向量 
    V2 = [ (C[0]-2 * (billiard_radius) * UVov[0]), (C[1]-2 * (billiard_radius) * UVov[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 

    VHSE= np.array([V2[0]-M[0],V2[1]-M[1]])
    Vup = np.array([0, 1]).reshape(2, 1)
    uT = Vup.transpose()
    n = np.dot(uT, VHSE)
    HSEL = np.linalg.norm(VHSE)
    target_angle = float(np.rad2deg(np.arccos(n/HSEL)))
    if(V2[0]>=M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
        target_angle = target_angle * -1
    print ('OB_1擊球DEGREE :',target_angle)

######end######

######取得影像資料######

def yolo_pic():
    time.sleep(2.5)
    global M, C, OB_1, OB_2, all_ball
    global kissball, loading_key
    
    #take picture
    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())
    ball_imformation = YOLO_Detect.detect_ALL(img)

    M = find_cueball(ball_imformation)
    get_str_mid(ball_imformation)
    
    C =[0,0]
    OB_1 = [0,0]
    OB_2 = [0,0]
    all_ball = []
    kissball = 0
    loading_key = 1

######end######

######完整策略執行######

def Mission_Trigger():
    global M, C, pic_pos_x, pic_pos_y, pic_pos_z, pic_height_1, pic_height_2
    global target_angle, is_move
    global BMZ
    global saveImage_key, loading_key
    global yaw
    global ItemNo,position

    if is_move == True and Arm_state_flag == Arm_status.Isbusy :
         is_move = False
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1 and is_move == False:
        if Arm_state == 1: #傳送指令給socket選擇手臂動作
            if ItemNo==0:                   #--------------------待機位置--------------------------------------
                #position =  [36.8, 0 , 11.35, 180, 0, 90]
            #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
            #ArmTask.Arm_Mode(2,1,0,100,2)#action,ra,grip,vel(速度),both
                PTP_Move(36.8, 0 , 11.35, 180, 0, 90, 10, 1)
            # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            # input()
            #if  B == "open":
                #ItemNo = 1
                 
            #elif B =="non":
                #ItemNo = 0
                if dig_input[1]: #從0開始算(藍色接線)
                    print ("start!")            
                    ItemNo = 1
                else:
                    ItemNo = 0

                is_move = True


            elif ItemNo==1:            #---------------------------拍照位置---------------------------
                #position =  [ 4.24, 33.6, 39, 179.9, -2.7, 88.7]
                #position =  [3.33,33.6, 39, 179.9, -2.7, 88.7]
            #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
            #ArmTask.Arm_Mode(2,1,0,80,2)#action,ra,grip,vel,both
                PTP_Move(4.24, 33.6, 39, 179.9, -2.7, 88.7, 10, 1)
            # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                ItemNo = 2
                saveImage_key = 1
                is_move = True
                
            elif ItemNo==2:            #------------------------選定目標--------------------------
                global M_On_Table 
                M_On_Table = 0
                time.sleep(3)
                yolo_pic()
                print ("target")
                time.sleep(1)
                ItemNo = 3
            
                    
            elif ItemNo==3:            #------------------------確認母球在球桌--------------------------
            
                if loading_key == 1:
                    ItemNo = 3
                    print('check!')
                        
                elif M[0] >= H1[0]-6 and M[0] <= H4[0]+20 and M[1] >= H1[1]-20 and M[1] <= H4[1]+10 :
                    print('-----------------ok!')
                    time.sleep(3)
                    ItemNo = 4
                        
                else:
                    print('no----------')
                    M_On_Table = 0
                    loading_key = 0
                    ItemNo = 0
                        

            elif (ItemNo==4 and M_On_Table!=0):             #------------------------決定擊球方式--------------------------
                print('case(4)')    
                if(M[0]==0 and M[1]==0):
                    ItemNo = 0
                GetStartEnd()
                print(ball_sorted)
                
                #position =  [0, 36.8, BMZ+40.08, 180, 0, 90]
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,80,2)#action,ra,grip,vel,both
                #robot_ctr.Step_AbsPTPCmd(position)
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                    
                if (M[0]==0 and M[1]==0):
                    ItemNo = 0
                else:
                    ItemNo = 5
                is_move = True
                    
            elif ItemNo==5:             #------------------------二次確認位置--------------------------
                print('case(5)')
                #position =  [M[0] - A[1, 3], M[1] - A[0, 3], 25, 180, 0, 90]
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,80,2)#action,ra,grip,vel,both
                PTP_Move(M[0] - A[1, 3], M[1] - A[0, 3], 25, 180, 0, 90, 10, 1)
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                # input()
                ItemNo = 6
                is_move = True
                    
            elif ItemNo==8:               #--------------------確定母球位置-----------------------
                time.sleep(1.3)
                print('case(8)')
                ###
                # position =  [M[0], M[1], -1, 180, 0, target_angle+90]
                # position = [M[0]+2.7-(34-M[0])/68*6.7, M[1]-3.9+(59-M[1])/36*6.9, -1, 180, 0, target_angle+90]
                # print('yeet')
                ###
                if(M[0]>16 and M[1]>33.5 and M[1]<55):
                    PTP_Move(M[0]-4, M[1]-1, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('right right') 
                elif(M[0]>6 and M[1]>33.5 and M[1]<55):
                    PTP_Move(M[0]-1, M[1], -0.8, 180, 0, target_angle+90, 10, 1)
                    print('left right') 
                elif(M[0]<-29 and M[1]>33.5 and M[1]<55):                                                        
                    PTP_Move(M[0]+3.4, M[1], -0.5, 180, 0, target_angle+90, 10, 1)
                    print('left')
                elif(M[0]<6  and M[0]>-2 and M[1]>33.5 and M[1]<55):
                    PTP_Move(M[0]-1, M[1]+0.5, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('middle') 
                elif(M[0]<6 and M[1]<33.5):
                    PTP_Move(M[0]+1, M[1]+3.5, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('left button') 
                elif(M[0]>16 and M[1]<33.5):
                    PTP_Move(M[0]-3.5, M[1]+2, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('right right button')
                elif(M[0]>6 and M[0]<16 and M[1]<33.5):
                    PTP_Move(M[0]-2.5, M[1]+2, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('right left button')
                elif(M[0]>6 and M[0]<16 and M[1]>55):
                    PTP_Move(M[0]-2.5, M[1]-2, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('right left top')  
                elif(M[0]>16 and M[1]>55):
                    PTP_Move(M[0]-3.5, M[1]-2, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('right right top')  
                elif(M[0]<6 and M[1]>55):
                    PTP_Move(M[0]+2.4, M[1]-1.8, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('left top')  
                else:                                                        
                    PTP_Move(M[0]+1.25, M[1]+0.5, -0.8, 180, 0, target_angle+90, 10, 1)
                    print('normal')
                                                                               
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,40,2)#action,ra,grip,vel,both
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                ItemNo = 11
                is_move = True
                   
            elif ItemNo==9:                #--------------------擊球起始點上方-----------------------
                print('case(9)')
                #position =  [M[0], M[1], -5, 180, 0, target_angle] 
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,20,2)#action,ra,grip,vel,both
                PTP_Move(M[0], M[1], -5, 180, 0, target_angle, 10, 1)
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                ItemNo = 10
                is_move = True
                                
            elif ItemNo==10:             #--------------------擊球起始點-----------------------    
                print('case(10)')
                if(M[0]>20):
                    PTP_Move(M[0]-5, M[1], -10, 180, 0, target_angle, 10, 1)
                    print('max') 
                else:                                                         
                    PTP_Move(M[0], M[1], -10, 180, 0, target_angle, 10, 1)
                    print('min')
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                # input() #---------------------------------------------------------------------------------------------------
                ItemNo = 11
                is_move = True
                
            elif ItemNo==11:             #------------------------擊球------------------------------
                #draw()   
                if(shoot==150):
                    modbus.DO(IO_Port,True)
                    time.sleep(0.5)
                    modbus.DO(IO_Port,False)
                    time.sleep(3)
                    print('hit ball weak!')
                elif(shoot==255):
                    modbus.DO(IO_Port,True)
                    time.sleep(0.5)
                    modbus.DO(IO_Port,False)
                    time.sleep(3)
                    print('hit ball strong!')
                ItemNo = 12
                    
            elif ItemNo==12:                #------------------------擊球起始點上方(避免線路纏繞)-------------------------------
                print('back home')
                if(0<target_angle+90<90):
                    yaw = 45
                    print('45')
                elif(90<=target_angle+90<180):
                    yaw = 135
                    print('135')
                elif(-179<=target_angle+90<-90):
                    yaw =180 
                    print('180')
                else:
                    yaw =45
                    print('45')
                #position =  [M[0], M[1], 8, 180, 0, yaw]
                time.sleep(2)
                #ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both
                PTP_Move(M[0], M[1], 8, 180, 0, yaw, 10, 1)
                # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                # draw()
                                            #-----------回合結束------------------------------------
                ItemNo = 0
                is_move = True
                    
            else: # default, could also just omit condition or 'if True'
                rospy.on_shutdown(myhook)
                #ArmTask.rospy.on_shutdown(myhook)           


    #action: ptp line
    #ra : abs rel
    #grip 夾爪
    #vel speed
    #both : Ctrl_Mode

def myhook():
    print ("shutdown time!")

######end######





'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''

so_file = "./Hiwin_API.so"
modbus = CDLL(so_file)


if __name__ == "__main__":
    Arm_state = 0
    IO_Port = 301 # D0
    
    
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off

    
    rqt_callback()

    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        Mission_Trigger()
        if(modbus.Arm_State_REGISTERS() == 1):
            break

    modbus.Modbus_Close()
    print("Modbus Close")  

    

