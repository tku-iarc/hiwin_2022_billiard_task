from calendar import c
from operator import mod
import numpy as np
from ctypes import *
import time
import rospy

import pyrealsense2 as rs
import cv2
import time
import copy
import math
import os
import configparser
from matplotlib import pyplot as plt 

import YOLO_Detect

is_busy = True
open_ball = 0
shoot = 255

start_point = [-367.708, 12.711 ,293.500, 180.000, 0.000, 90.605]
pic_Point = [6.624, 399.776, 410.976, 180.000, 0, 91.663]
higher_pic_point = [-2.457, 417.614, 410.971, 180.000, 0, 90.605]
open_point = [156.912, 450.010, 9.072, 180.000, 0, 180]
above_open_point = [156.912, 450.010, 21.072, 180.000, 0, 90]

yaw_zero = [-999,-999,-999,-999,-999,-999]
yaw_180 = [-999,-999,-999,-999,-999,-999]
go_home_zero = [-999,-999,-999,-999,-999,-999]
go_home_180 = [-999,-999,-999,-999,-999,-999]

closelook_point = [000.000, 000.000, 000.000, 000.000, 000.000]
cueball_point = [000.000, 000.000, 000.000, 000.000, 000.000]
sucker2camera = [0, -26.25]                     #[-47.0, -32.5] orig
M = [-999,-999,-999,-999,-999,-999]
target = [-999,-999,-999,-999,-999,-999]
moveup_point = [-999,-999,-999,-999,-999,-999]

H1 = [-999,-999]
H2 = [-999,-999]
H3 = [-999,-999]
H4 = [-999,-999]
H5 = [-999,-999]
H6 = [-999,-999]


# Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)
''''''


'''
    PTP移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def PTP_Move(Point, speed=50, acceleration=20):
    print(bcolors.Move + "    PTP Move ... " + bcolors.RESET ,end='')
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

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
        # time.sleep(0.01)
    print(bcolors.EndMove + "END PTP Move!" + bcolors.RESET)

# 輸出(print)使用顏色
class bcolors:
    # 大項目
    Task = '\033[90m'    # 深藍
    Warning = '\033[31m'

    # 資料計算
    Count = '\033[36m'   #
    OK = '\033[32m'     #綠色加底
    Data = '\033[47m'   # 資料輸出

    # 手臂移動
    Move = '\033[93m' #黃色加底
    EndMove = '\033[32m' #綠色加底

    # 重設
    RESET = '\033[0m' #RESET COLOR



def GetStartEnd():
    global num_hole, gHole
    global kissball
    global V, M, C
    global target_angle,shoot
    global billiard_radius

    if kissball == 3:
        print("target hole site is: ", gHole)
        print('lets see my shoot!!!!!!------->', target_angle)
        print("============================================================================")
    elif kissball == 2:
        print("target hole site is: ", gHole)
        print('try kickshot------->', target_angle)
        print("============================================================================")
        shoot=255
    elif kissball == 1:
        check_angle()
        print("target hole is 1: number ",num_hole,"hole")
        print("target hole site is: ", gHole)
        MOB_1C()
        print("============================================================================")
    elif kissball == 0:
        check_angle()
        print("target hole is 0: number",num_hole,"hole")
        print("target hole site is: ", gHole)
        #---------------計算子球後的虛擬球球心位置------------

        VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        chL = np.linalg.norm(VCtoH)                       #CH向量的長度
        UVch = VCtoH / chL                                #CH向量的單位向量  
        V = [ (C[0] - 2*(billiard_radius)*UVch[0]), (C[1] - 2*(billiard_radius) * UVch[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

        VHSE= np.array([V[0]-M[0],V[1]-M[1]])             #M指向V的向量
        Vup = np.array([0, 1]).reshape(2, 1)
        uT = Vup.transpose()
        n = np.dot(uT, VHSE)
        HSEL = np.linalg.norm(VHSE)
        target_angle = float(np.rad2deg(np.arccos(n/HSEL)))
        print ('target_angle:',target_angle)
        if(V[0]<M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
            target_angle = target_angle * -1       
        print ('normal hitball')      
        print ('normal hitball DEGREE :',target_angle)
        print("============================================================================")

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


######球數大於10的策略######
###all_ball([str(item.Class), float(item.probability), BASE_X, BASE_Y, dis_BaH[result[0][0]]])###
def early_stage():
    global C, M, pic_OB, all_ball
    global ball_sorted, kissball
    global shoot

    all_ball.sort(key = lambda x: x[4])      #依與洞口的距離排序 #lambda x:x[4]代表x=x[4]，x用all_ball[]帶入 #這邊就是依據all_ball[4]從小排列
    ball_sorted = []
    for items in all_ball:
        if items[4] != 0.0 and (( ((M[0]-billiard_radius) > items[2]) or (items[2] > (M[0]+billiard_radius)) ) or ( ((M[1]-billiard_radius) > items[3]) or (items[3] > (M[1]+billiard_radius)) )):
            ball_sorted.append(items)          #排除把母球辨識成其他球之母球資訊及空資訊
    # print(ball_sorted)
    target_contener = [[],[],[],[]]             #分類子球[[好球], [大角度球], [干擾], [大角度干擾]]  [[0 for i in range(4)] for j in]
    for value in ball_sorted:
        if( value[0] != 'M'):
            C = [value[2], value[3]]            #預設C(子球)
            dis_CM = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)         #母球與子球的距離
            OB_MC = check_front(0)          #判斷母球和子球間是否有干擾球
            OB_HC = check_back()            #判斷洞口和子球間是否有干擾球
            print("C:[{:4.0f},{:4.0f}], OB1_ball:{}, OB2_ball:{}".format(value[2],value[3],OB_MC,OB_HC))
            # angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB_MC == 0 and OB_HC == 0:
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
            elif OB_MC == 1 and OB_HC == 0:              #-------------obstacle should be 1!now----------
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
    print("============================================================================")

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
            OB_MC = check_front(0)          #判斷母球和子球間是否有干擾球
            OB_HC = check_back()             #判斷洞口和子球間是否有干擾球
            print("C:[{:4.0f},{:4.0f}], OB1_ball:{}, OB2_ball:{}".format(value[2],value[3],OB_MC,OB_HC))
            # angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB_MC == 0 and OB_HC == 0:
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
            elif OB_MC == 1 and OB_HC == 0:              #-------------obstacle should be 1!now----------
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
    print("============================================================================")

######end######

######球數小於4顆的策略######

def final_stage():
    global C, M, pic_OB, all_ball
    global ball_sorted, kissball
    global shoot

    all_ball = sorted(all_ball,key = lambda x: x[4])  #依與洞口的距離排序
    ball_sorted = []
    for items in all_ball:
        if items[4] != 0.0 and (((M[0]-0.7*billiard_radius) > items[2] or items[2] > (M[0]+0.7*billiard_radius)) or ((M[1]-0.7*billiard_radius) > items[3] or items[3] > (M[1]+0.7*billiard_radius))):
            ball_sorted.append(items)          #排除把母球辨識成其他球之母球資訊及空資訊
            print(ball_sorted)
    target_contener = [[],[],[],[]]             #分類子球[[好球], [大角度球], [干擾], [大角度干擾]]
    for value in ball_sorted:
        if( value[0] != '0'):
            C = [value[2], value[3]]            #預設C(子球)
            dis_CM = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)         #母球與子球的距離
            OB_MC = check_front(0)          #判斷母球和子球間是否有干擾球
            OB_HC = check_back()             #判斷洞口和子球間是否有干擾球
            print("C:[{:4.0f},{:4.0f}], OB1_ball:{}, OB2_ball:{}".format(value[2],value[3],OB_MC,OB_HC))
            #angle = MBallCheck()           #判斷子球與洞口的角度是否太大
            if OB_MC == 0 and OB_HC == 0:
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
            elif OB_MC == 1 and OB_HC == 0:              #-------------obstacle should be 1!now----------
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
    print("============================================================================")

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

    check_angle() 
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
        if item[0] != 'M':
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



def find_holes():
    global H1, H2, H3, H4, H5, H6
    BMX = -343.851
    BMY = 262.251      
    BMX_1 = 340.392
    BMY_1 = 262.361

    TCC = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])

    B2B= np.array([BMX_1-BMX,BMY_1-BMY])    #從BM指向BM_1的向量                       
    B2BL = np.linalg.norm(B2B)              #B2B向量的長度
    Uv = B2B / B2BL                         #B2B向量的單位向量  
    Uu = TCC.dot(Uv)                      #B2B向量逆時針90度旋轉的單位向量

    H1 = [BMX + 41*Uv[0] + 41*Uu[0], BMY + 41*Uv[1] + 41*Uu[1]]    #洞H1 左下

    H2 = [BMX + 41*Uv[0] + 332.7*Uu[0], BMY + 41*Uv[1] + 332.7*Uu[1]]   #洞H2 左上

    H3 = [((BMX + BMX_1)/2) + 332.7*Uu[0], ((BMY + BMY_1)/2) + 332.7*Uu[1]]   #洞H3 中上

    H4 = [BMX_1 - 41*Uv[0] + 332.7*Uu[0], BMY_1 - 41*Uv[1] + 332.7*Uu[1]]   #洞H4 右上

    H5 = [BMX_1 - 41*Uv[0] + 41*Uu[0], BMY_1 - 41*Uv[1] + 41*Uu[1]]   #洞H5 右下

    H6 = [(BMX + BMX_1)/2 + 41*Uu[0], (BMY + BMY_1)/2 + 41*Uu[1]]    #洞H6 中下  
    
    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)
    print("============================================================================")



def curve():
    global all_ball, my_data, M
    global H1, H2, H3, H4, H5, H6
    global billiard_radius

    hole_x = [H1[0], H2[0], H3[0], H4[0], H5[0], H6[0]]
    hole_y = [H1[1], H2[1], H3[1], H4[1], H5[1], H6[1]]
 #=====================================================================
    c_x = 642.69902197
    c_y = 370.27915175
    f_x = 918.73516739
    f_y = 919.13339111
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([1.04827825e-01, 1.62037134e-01, -9.84883489e-04 , 1.89111101e-03, -1.35428691e+00])
 #---------------------------------------------------------------------
    all_ball = []
    counter = 0
    dis_BaH = np.zeros(6) 
    for item in my_data:
        if (item[0] != 'M') & (float(item[1]) > 70): 
            exclusion_ball = 0
            # -----取中心座標-----
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]
            center_x = round((img_pos[0]-(1280/2))*0.5, 3)
            center_y = round((img_pos[1]-(720/2))*0.5, 3)

            if center_x>229.33:
                new_center_x = center_x*0.95
                print("1")

            elif (center_x<-302.85) and (center_y<350.399):
                new_center_x = center_x*0.91
                print("2")

            elif (center_x<-302.85) and (center_y>=350.399) and (center_y<394.709):
                new_center_x = center_x*0.3
                print("3")

            elif center_x<-302.85:
                new_center_x = center_x*0.87
                print("4")

            elif (center_x<-302.85) and (center_y>=503.58) and (center_y<540.631):
                new_center_x = center_x*0.5
                print("5")

            elif (center_x<-302.85) and (center_y>540.631):
                new_center_x = center_x*0.91
                print("6")

            else:
                new_center_x = center_x

#350.399->394.709->503.580->540.631
            BASE_X = pic_Point[0] + new_center_x -2
            BASE_Y = pic_Point[1] - center_y - sucker2camera[1] + 6.531

           
            if(BASE_X<-306.402 and BASE_Y>596.253):       #左上洞口       #去除洞口誤判成球
                exclusion_ball = 1
            elif(BASE_X<-306.402 and BASE_Y<304.554):     #左下洞口
                exclusion_ball = 1
            elif(BASE_X>299.18 and BASE_Y>596.254):     #右上洞口
                exclusion_ball = 1
            elif(BASE_X>299.18 and BASE_Y<304.554):     #右下洞口
                exclusion_ball = 1
            elif(BASE_X>-6 and BASE_X<0 and BASE_Y>596.253):     #中上洞口
                exclusion_ball = 1
            elif(BASE_X>-6 and BASE_X<0 and BASE_Y<304.554):     #中下洞口
                exclusion_ball = 1
            else:
                exclusion_ball = 0

            for j in range(6):
                 dis_BaH[j] = (((hole_x[j] - BASE_X)**2 + (hole_y[j] - BASE_Y)**2)**0.5)
                 if dis_BaH[j] <= 1*billiard_radius :
                    print('i see a ball in hole~~~~~~~~~~~~~~~~~~~~~')
                
                    exclusion_ball = 1
            result = np.where(dis_BaH == np.amin(dis_BaH))          #result代表dis_BaH的最小值的位置
            if exclusion_ball != 1:
                all_ball.append([str(item[0]), float(item[1]), BASE_X, BASE_Y, dis_BaH[result[0][0]]])
                counter += 1

    print(all_ball)
    print("============================================================================")
    #print(' s', counter,'t')
    return counter


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

def find_cueball(data, mode):
 #=====================================================================
    c_x = 642.69902197
    c_y = 370.27915175
    f_x = 918.73516739
    f_y = 919.13339111
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    #dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    dist_coef = np.array([1.04827825e-01, 1.62037134e-01, -9.84883489e-04 , 1.89111101e-03, -1.35428691e+00])
    for item in data:
        if item[0] == 'M':
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]
            center_x = round((img_pos[0]-(1280/2))*0.5, 3)
            center_y = round((img_pos[1]-(720/2))*0.5, 3)

            if center_x>229.33:
                new_center_x = center_x*0.95
                print("1")

            elif (center_x<-302.85) and (center_y<350.399):
                new_center_x = center_x*0.91
                print("2")

            elif (center_x<-302.85) and (center_y>=350.399) and (center_y<394.709):
                new_center_x = center_x*0.3
                print("3")

            elif center_x<-302.85:
                new_center_x = center_x*0.87
                print("4")

            elif (center_x<-302.85) and (center_y>=503.58) and (center_y<540.631):
                new_center_x = center_x*0.5
                print("5")

            elif (center_x<-302.85) and (center_y>540.631):
                new_center_x = center_x*0.91
                print("6")

            else:
                new_center_x = center_x


            if mode == 'close_TakePic':
                BASE_X = pic_Point[0] + new_center_x - 2
                BASE_Y = pic_Point[1] - center_y - sucker2camera[1] + 6.531
                M = [BASE_X , BASE_Y , 100.171 , 180 , 0 , -179.929]
            if mode == 'doublecheck':
                BASE_X = pic_Point[0] + round((img_pos[0]-(1280/2))*0.5, 3) + sucker2camera[0]
                BASE_Y = pic_Point[1] - round((img_pos[1]-(720/2))*0.5, 3) - sucker2camera[1]
                M = [BASE_X , BASE_Y , -66 , 180 , 0 , -179.929]

    return M

def yolo_pic():
    global M
    global loading_key

    loading_key =1

    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())
    ball_information = YOLO_Detect.detect_ALL(img,0.5)

    print(ball_information)

    time.sleep(0.1)

    M = find_cueball(ball_information, "close_TakePic")
    get_str_mid(ball_information)

def plot_table():
    global H1, H2, H3, H4, H5, H6, M, C
    global all_ball
    hole_x = [H1[0], H2[0], H3[0], H4[0], H5[0], H6[0], H1[0]]
    hole_y = [H1[1], H2[1], H3[1], H4[1], H5[1], H6[1], H1[1]]
    ball_sorted = []
    ball_x = []
    ball_y = []

    for item in all_ball:
        if item[0] != -999:
            ball_sorted.append(item)
    print(ball_sorted)

    for balls in ball_sorted:
        ball_x.append(balls[2])
        ball_y.append(balls[3])
    print(ball_x)
    print(ball_y)

    fig = plt.figure() 
    axes = fig.add_subplot(111)
    axes.set_facecolor('g')
    
    plt.plot(hole_x,hole_y)

    for i in range(6):
        hole_circle = plt.Circle((hole_x[i], hole_y[i]), 15.9, fill=False)
        axes.set_aspect(1)
        axes.add_artist(hole_circle)

    ball_center = plt.Circle((M[0], M[1]), 12.45, color = 'w')
    axes.set_aspect(1)
    axes.add_artist(ball_center) 

    V_center = plt.Circle((V[0], V[1]), 12.45, color = 'y')
    axes.set_aspect(1)
    axes.add_artist(V_center) 

    for j in range(len(ball_sorted)):            
        if (ball_x[j] == C[0]) & (ball_y[j] == C[1]):
            ball_center = plt.Circle((ball_x[j], ball_y[j]), 12.45, color = 'r')
            axes.set_aspect(1)
            axes.add_artist(ball_center) 
        else:    
            ball_center = plt.Circle((ball_x[j], ball_y[j]), 12.45)
            axes.set_aspect(1)
            axes.add_artist(ball_center) 
    print("============================================================================")

    plt.title("Table")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.xlim(-400,400)
    plt.ylim(0,850)
    # plt.ion()
    plt.show()
    plt.pause(1.5)
    plt.close()
 
def Mission_trigger():
    global is_busy
    global M
    global open_ball, shoot

    step = 0
    IO_Port = 301 # D0
    IO_Port_4 = 303 # D4 未降壓
    IO_Port_5 = 304 # D5 降壓

    if is_busy == True:
        is_busy = False
    if is_busy == False:
        if modbus.DI(IO_Port_in,0) == 1:
            is_busy = True
            PTP_Move(start_point,50,20)
            step = 1

        if step == 1:
            if open_ball == 1:
                is_busy = True
                PTP_Move(above_open_point,50,20)
                PTP_Move(open_point,50,20)
                
                modbus.DO(IO_Port_4,1)
                time.sleep(0.1)
                modbus.DO(IO_Port_4,0)
                time.sleep(0.1)
                PTP_Move(above_open_point,50,20)
                open_ball = 0               
                step = 6

            elif open_ball == 0:
                is_busy = True
                PTP_Move(pic_Point,50,20)
                time.sleep(0.1)
                find_holes()
                yolo_pic()
                step = 2

        if step == 2:
            GetStartEnd()
            plot_table()
            step = 3

        if step == 3:
            is_busy = True
            hit_angle = 90-target_angle
            print(hit_angle)
            print("============================================================================")
            target = [M[0], M[1], 23.803, 180.000, 0.000, hit_angle]
            if (hit_angle <= 0) and (hit_angle >= -90):
                yaw_zero = [M[0], M[1], 41.072, 180.000, 0.000, 0]
                PTP_Move(yaw_zero,50,20)
                time.sleep(0.5)
                PTP_Move(target,50,20)
            elif (target_angle < -90) and (target_angle >= -180):
                yaw_180 = [M[0], M[1], 41.072, 180.000, 0.000, 180]
                PTP_Move(yaw_180,50,20)
                time.sleep(0.5)
                PTP_Move(target,50,20)
            else:
                PTP_Move(target,50,20)
            step = 4

        if step == 4:
            is_busy = True
            if shoot == 255:
                modbus.DO(IO_Port_4,1)
                time.sleep(0.1)
                modbus.DO(IO_Port_4,0)
            if shoot == 150:
                modbus.DO(IO_Port_5,1)
                time.sleep(0.1)
                modbus.DO(IO_Port_5,0)
            step = 5
        
        if step == 5:
            is_busy = True
            if (hit_angle <= 0) and (hit_angle >= -90):
                yaw_zero = [M[0], M[1], 41.072, 180.000, 0.000, 0]
                PTP_Move(yaw_zero,50,20)
                time.sleep(0.5)
                PTP_Move(start_point,50,20)
            elif (target_angle < -90) and (target_angle >= -180):
                yaw_180 = [M[0], M[1], 41.072, 180.000, 0.000, 180]
                PTP_Move(yaw_180,50,20)
                time.sleep(0.5)
                go_home_180 = [M[0], M[1], 41.072, 180.000, 0.000, 90]
                PTP_Move(go_home_180,50,20)
                time.sleep(0.5)
                PTP_Move(start_point,50,20)
            else:
                go_home_zero = [M[0], M[1], 41.072, 180.000, 0.000, 90]
                PTP_Move(go_home_zero,50,20)
                time.sleep(0.5)
                PTP_Move(start_point,50,20) 
            step = 0

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



if __name__ == '__main__':

    billiard_radius = 15
    Arm_state = 0
    IO_Port_in = 300  #開始按鈕
    IO_Port_4 = 303 # D4 未降壓
    IO_Port_5 = 304 # D5 降壓
    
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
    find_holes()

    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        PTP_Move(start_point,50,20)
        PTP_Move(start_point,50,20)

        Mission_trigger()

        # if(modbus.Arm_State_REGISTERS() == 1):
        #     break

    modbus.Modbus_Close()
    print("Modbus Close")  
