from calendar import c
from concurrent.futures import process
from operator import mod
from re import I
from tracemalloc import start
from turtle import hideturtle
from xmlrpc.server import CGIXMLRPCRequestHandler
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

from logging import root, shutdown
import tkinter as tk
from _tkinter import *
import os, sys
from unicodedata import name

import YOLO_Detect
import hiwin_motor

# sudo chmod 777 /dev/ttyUSB0

is_busy = True
shoot = 255
billiard_radius = 12.45
acl = 80
speed = 80
testing = 0

IO_Port_in = 300 # DI0 開始按鈕
IO_Port_4 = 303  # DO4 10V
IO_Port_5 = 304  # DO5 12V
IO_Port_6 = 305  # DO5 24V

BMX = -345.378    
BMY = 242.258   
BMX_1 = BMX + 692.95
BMY_1 = BMY 

height = 150

                ######################
                    # 開球設定
open_ball = 0      # open_ball = 1 -> 要開球, open_ball = 0 -> 不開球
open_R = 0          # open_R = 1 -> 開球位置在手臂右邊, open_L要設為0
open_L = 0          # open_L = 1 -> 開球位置在手臂左邊, open_R要設為0
                ######################

start_point = [-367.708, 12.711 ,445.693, 180.000, 0.000, 90.00]
pic_Point = [-2.367, 343.359, 530.359, 180.000, 0, 90]           # z = 530.359

open_point_left = [-172.626, 428.117, 117.063, 180, 0, 0]         # z = -27.62
open_point_left_above = [-172.626, 428.117, 155, 180, 0, 90]     # z = -10
open_point_right = [164.999, 422.354, 117.063, 180, 0, 180]
open_point_right_above = [164.999, 422.354, 155, 180, 0, 90]

left_corner_point = [-330.729, 246.414, 197.925, 180, 0, 90]
right_corner_point = [331.065, 249.509, 197.925, 180, 0, 90]
#############################################################################

testing_point = [-20.475, 444.575, 311.425, 180, 0, 90]

yaw_zero = [-999,-999,-999,-999,-999,-999]
yaw_180 = [-999,-999,-999,-999,-999,-999]
go_home_zero = [-999,-999,-999,-999,-999,-999]
go_home_180 = [-999,-999,-999,-999,-999,-999]

closelook_point = [000.000, 000.000, 000.000, 000.000, 000.000]
cueball_point = [000.000, 000.000, 000.000, 000.000, 000.000]
Hit2camera = [5.252, 85.255]                    #[-47.0, -32.5] orig  更之前的 [0, -26.25]
M = [-999,-999,-999,-999,-999,-999]
target = [-999,-999,-999,-999,-999,-999]
moveup_point = [-999,-999,-999,-999,-999,-999]

pic_x_float = 0.753
pic_y_float = 390.671
pic_z_float = 380.513
pic_yaw_float = 90.975
left_x_float = -345.671
left_y_float = 239.309
right_x_float = 346.408
right_y_float = 241.056

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

    modbus.PTP(1, speed, acceleration, 0, 5, C_PTP_XYZ)
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

def find_holes():
    global H1, H2, H3, H4, H5, H6
    global BMX, BMY, BMX_1, BMY_1
    global testing
    global left_x_float, left_y_float
    global right_x_float, right_y_float
    global hit_power_check, smallest_dis_check, bigger_middle_y
    
    if testing == 0:
        BMX = left_x_float
        BMY = left_y_float
        BMX_1 = right_x_float
        BMY_1 = right_y_float

    TCC = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])

    B2B= np.array([BMX_1-BMX,BMY_1-BMY])    #從BM指向BM_1的向量                       
    B2BL = np.linalg.norm(B2B)              #B2B向量的長度
    Uv = B2B / B2BL                         #B2B向量的單位向量  
    Uu = TCC.dot(Uv)                      #B2B向量逆時針90度旋轉的單位向量

    H1 = [BMX + 47.2*Uv[0] + 47.2*Uu[0], BMY + 47.2*Uv[1] + 47.2*Uu[1]]    #洞H1 左下

    H2 = [BMX + 47.2*Uv[0] + 360*Uu[0], BMY + 47.2*Uv[1] + 360*Uu[1]]   #洞H2 左上

    H3 = [((BMX + BMX_1)/2) + 360*Uu[0], ((BMY + BMY_1)/2) + 360*Uu[1]]   #洞H3 中上

    H4 = [BMX_1 - 47.2*Uv[0] + 360*Uu[0], BMY_1 - 47.2*Uv[1] + 360*Uu[1]]   #洞H4 右上

    H5 = [BMX_1 - 47.2*Uv[0] + 47.2*Uu[0], BMY_1 - 47.2*Uv[1] + 47.2*Uu[1]]   #洞H5 右下

    H6 = [(BMX + BMX_1)/2 + 47.2*Uu[0], (BMY + BMY_1)/2 + 47.2*Uu[1]]    #洞H6 中下  

    hit_power_check = (abs(H1[0] - H6[0])*(2/3))
    smallest_dis_check = (abs(H1[0] - H6[0])/4)
    bigger_middle_y = (abs(H1[1] + H2[1])/2)
    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)
    print("============================================================================")

def FinalAngle():
    global M, C
    global kissball,shoot,luckyshoot
    global num_hole, gHole
    global billiard_radius
    global back_OB_storted
    global target_angle
    global V

    if kissball == 0:
        find_TargetHole()
        print("target hole is : {}".format(num_hole))
        print("target hole site is : {}".format(gHole))
        #---------------計算子球後的虛擬球球心位置------------

        good_angle(mode = 'good')
        C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        L_C2H = np.linalg.norm(C2H)                       #CH向量的長度
        Unit_C2H = C2H / L_C2H                                #CH向量的單位向量  
        V = [ (C[0] - 2.3*(billiard_radius)*Unit_C2H[0]), (C[1] - 2.3*(billiard_radius) * Unit_C2H[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

        M2V= np.array([V[0]-M[0],V[1]-M[1]])             #M指向V的向量
        front_vector = np.array([0, 1]).reshape(2, 1)
        T_front_vector = front_vector.transpose()
        n = np.dot(T_front_vector, M2V)
        L_M2V = np.linalg.norm(M2V)
        target_angle = float(np.rad2deg(np.arccos(n/L_M2V)))
        print ('target_angle:',target_angle)
        if(V[0]<M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
            target_angle = target_angle * -1       
        print ('normal hitball')      
        print ('normal hitball DEGREE :',target_angle)
        print("============================================================================")
    elif kissball == 1:
        OB_V()
    elif luckyshoot == 1:    
        luckyball()

def OB_V():
    global M, C
    global gHole
    global target_angle
    global back_OB_storted
    global billiard_radius
    global V1, V2

    #------------從洞口(gHole)推算子球(C)的虛擬球1(V1)-------------------
    print("C[2] :::::: ",C[2])
    find_TargetHole()
    good_angle(mode = 'OB_plot')
    C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])                  #從C指向H的向量                       
    L_C2H = np.linalg.norm(C2H)                                     #CH向量的長度
    Unit_C2H = C2H / L_C2H                                              #CH向量的單位向量  
    V1 = [ (C[0] - 2*(billiard_radius) * Unit_C2H[0]), (C[1] - 2*(billiard_radius) * Unit_C2H[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

    #------------從虛擬球1(V1)推算干擾球(OB_1)的虛擬球2(V2)-------------------
    OB2V1 = np.array([V1[0]-C[2][0], V1[1]-C[2][1]])
    L_OB2V1 = np.linalg.norm(OB2V1)
    Unit_OB2V1 = OB2V1/L_OB2V1
    V2 = [(C[2][0] - 2*(billiard_radius) * Unit_OB2V1[0]), (C[2][1] - 2*(billiard_radius) * Unit_OB2V1[1])]

    M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
    front_vector = np.array([0, 1]).reshape(2, 1)
    T_front_vector = front_vector.transpose()
    n = np.dot(T_front_vector, M2V2)
    L_M2V2 = np.linalg.norm(M2V2)
    target_angle = float(np.rad2deg(np.arccos(n/L_M2V2)))
    print ('target_angle:',target_angle)
    if(V2[0]<M[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
        target_angle = target_angle * -1       
    print ('OB hitball')      
    print ('OB hitball DEGREE :',target_angle)
    print("============================================================================")


def first_stage():
    global M, C
    global all_ball
    global billiard_radius, hit_power_check, smallest_dis_check
    global back_OB_storted, gHole
    global ball_storted
    global kissball,shoot,luckyshoot

    luckyshoot = 0
    kissball = 0
    bad_target = 0

    all_ball.sort(key = lambda x:x[4])
    ball_storted = []
    for ball in all_ball:
        if (ball[4] > billiard_radius) and (( ((M[0]-billiard_radius) > ball[2]) or (ball[2] > (M[0]+billiard_radius)) ) or ( ((M[1]-billiard_radius) > ball[3]) or (ball[3] > (M[1]+billiard_radius)) )):
            ball_storted.append(ball)

    good_target = []
    BigAngle_Good_target = []
    OB_target = []

    for items in ball_storted:
        if (items[0] != 'M'):
            C = [items[2], items[3]]
            find_TargetHole()
            dis_CH = (((C[0] - gHole[0])**2 + (C[1] - gHole[1])**2)**0.5)   # 子球和目標洞距離,用來決定擊球力度
            dis_MC = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)           # 母球和子球距離,用來決定擊球力度
            front_OB = check_front()        # front代表目標球與球洞之間
            back_OB = check_back()          # back代表母球與目標球之間
            print("目前子球 C :[{:4.0f},{:4.0f}], 目標球與球洞之間 front_OB:{},母球與目標球之間 back_OB:{}".format(items[2],items[3],front_OB,back_OB))

            if (front_OB == 0) and (back_OB == 0):
                angle = good_angle(mode = 'good')
                if angle == 1:
                    good_target.append([items[0], C[0], C[1], dis_MC, dis_CH])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , good target !!".format(C[0], C[1]))
                elif angle == 0:
                    BigAngle_Good_target.append([items[0], C[0], C[1], dis_MC, dis_CH])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , good but big angle target !!".format(C[0], C[1]))
            elif (front_OB == 0) and (back_OB == 1):
                angle = good_angle(mode = 'OB')
                if angle == 1:
                    OB_target.append([items[0], C[0], C[1], back_OB_storted[0], dis_MC])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , kissball !!".format(C[0], C[1]))
                elif angle == 0:
                    bad_target += 1
                    print("目前子球 C :[{:4.0f},{:4.0f}] , big angle kissball !!".format(C[0], C[1]))
            else:
                bad_target += 1
                print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))

    print("//////////////////////////////////////////////////////////////////////")
    
    if len(good_target) >= 1:
        good_target.sort(key = lambda x: x[3])
        # good_target.sort(key = lambda x: x[4])
        print("Successfully set a good target C : [{}, {:4.0f}, {:4.0f}]".format(good_target[0][0], good_target[0][1], good_target[0][2]))
        C = [good_target[0][1], good_target[0][2]]
        if ((good_target[0][3] >= hit_power_check) and (good_target[0][3] < hit_power_check*2)) or ((good_target[0][4] >= hit_power_check) and (good_target[0][4] < hit_power_check*2)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (good_target[0][3] >= hit_power_check*2) or (good_target[0][4] >= hit_power_check*2):
            shoot = 300
            kissball = 0
            print("super power !!")
        elif ((good_target[0][3] < hit_power_check) and (good_target[0][3] > smallest_dis_check)) and ((good_target[0][4] < hit_power_check*2) and (good_target[0][4] > smallest_dis_check)):
            shoot = 150
            kissball = 0
            print("minimum power !!")
        elif (good_target[0][4] < 50):
            shoot = 150
            kissball = 0
            print("close hole !!")  
        else:
            shoot = 150
            kissball = 0
            print("不在判斷內，小力")

    elif (len(good_target) < 1) and (len(OB_target) >= 1):
        OB_target.sort(key = lambda x: x[4])
        print("Successfully set a kissball target C : [{}, {:4.0f}, {:4.0f}]".format(OB_target[0][0], OB_target[0][1], OB_target[0][2]))
        C = [OB_target[0][1], OB_target[0][2], OB_target[0][3]]
        print("back_OB_storted : ",C[2])
        shoot = 255
        kissball = 1
        print("maximum power !!")

    elif (len(good_target) < 1) and (len(OB_target) < 1) and (bad_target >= len(BigAngle_Good_target)):
        shoot = 300
        luckyshoot = 1
        print("luckyball !!")

    elif (len(good_target) < 1) and (len(OB_target) < 1) and (bad_target < len(BigAngle_Good_target)) and (len(BigAngle_Good_target) >= 1):
        # BigAngle_Good_target.sort(key = lambda x: x[3])
        BigAngle_Good_target.sort(key = lambda x: x[4])
        print("Successfully set a good but big angle target C : [{}, {:4.0f}, {:4.0f}]".format(BigAngle_Good_target[0][0], BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]))
        C = [BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]]
        if ((BigAngle_Good_target[0][3] >= hit_power_check) and (BigAngle_Good_target[0][4] >= hit_power_check)) or \
           ((BigAngle_Good_target[0][3] >= hit_power_check) and (BigAngle_Good_target[0][4] < hit_power_check)) or \
           ((BigAngle_Good_target[0][3] < hit_power_check) and (BigAngle_Good_target[0][4] >= hit_power_check)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif ((BigAngle_Good_target[0][3] < hit_power_check) and (BigAngle_Good_target[0][3] > smallest_dis_check)) and ((BigAngle_Good_target[0][4] < hit_power_check) and (BigAngle_Good_target[0][4] > smallest_dis_check)):
            shoot = 150
            kissball = 0
            print("minimum power !!")
        elif (BigAngle_Good_target[0][4] < 60):
            shoot = 150
            kissball = 0
            print("close hole !!")  

    else:
        shoot = 300
        luckyshoot = 1
        print("luckyball !!")
    
    print("============================================================================")

def second_stage():
    global M, C
    global all_ball
    global billiard_radius, hit_power_check, smallest_dis_check
    global back_OB_storted, gHole
    global ball_storted
    global kissball,shoot,luckyshoot

    luckyshoot = 0
    kissball = 0
    bad_target = 0

    all_ball.sort(key = lambda x:x[4])
    ball_storted = []
    for ball in all_ball:
        if (ball[4] > billiard_radius) and (( ((M[0]-billiard_radius) > ball[2]) or (ball[2] > (M[0]+billiard_radius)) ) or ( ((M[1]-billiard_radius) > ball[3]) or (ball[3] > (M[1]+billiard_radius)) )):
            ball_storted.append(ball)

    good_target = []
    BigAngle_Good_target = []
    OB_target = []

    for items in ball_storted:
        if (items[0] != 'M'):
            C = [items[2], items[3]]
            find_TargetHole()
            dis_CH = (((C[0] - gHole[0])**2 + (C[1] - gHole[1])**2)**0.5)   # 子球和目標洞距離,用來決定擊球力度
            dis_MC = (((C[0] - M[0])**2 + (C[1] - M[1])**2)**0.5)           # 母球和子球距離,用來決定擊球力度
            front_OB = check_front()        # front代表目標球與球洞之間
            back_OB = check_back()          # back代表母球與目標球之間
            print("目前子球 C :[{:4.0f},{:4.0f}], 目標球與球洞之間 front_OB:{},母球與目標球之間 back_OB:{}".format(items[2],items[3],front_OB,back_OB))

            if (front_OB == 0) and (back_OB == 0):
                angle = good_angle(mode = 'good')
                if angle == 1:
                    good_target.append([items[0], C[0], C[1], dis_MC, dis_CH])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , good target !!".format(C[0], C[1]))
                elif angle == 0:
                    BigAngle_Good_target.append([items[0], C[0], C[1], dis_MC, dis_CH])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , good but big angle target !!".format(C[0], C[1]))
            elif (front_OB == 0) and (back_OB == 1):
                angle = good_angle(mode = 'OB')
                if angle == 1:
                    OB_target.append([items[0], C[0], C[1], back_OB_storted[0], dis_MC])
                    print("目前子球 C :[{:4.0f},{:4.0f}] , kissball !!".format(C[0], C[1]))
                elif angle == 0:
                    bad_target += 1
                    print("目前子球 C :[{:4.0f},{:4.0f}] , big angle kissball !!".format(C[0], C[1]))
            else:
                bad_target += 1
                print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))

    print("//////////////////////////////////////////////////////////////////////")
    
    if len(good_target) >= 1:
        good_target.sort(key = lambda x: x[3])
        # good_target.sort(key = lambda x: x[4])
        print("Successfully set a good target C : [{}, {:4.0f}, {:4.0f}]".format(good_target[0][0], good_target[0][1], good_target[0][2]))
        C = [good_target[0][1], good_target[0][2]]
        if ((good_target[0][3] >= hit_power_check) and (good_target[0][3] < hit_power_check*2)) or ((good_target[0][4] >= hit_power_check*2) and (good_target[0][4] < hit_power_check*2)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (good_target[0][3] >= hit_power_check*2) or (good_target[0][4] >= hit_power_check*2):
            shoot = 300
            kissball = 0
            print("super power !!")
        elif ((good_target[0][3] < hit_power_check) and (good_target[0][3] > smallest_dis_check)) and ((good_target[0][4] < hit_power_check) and (good_target[0][4] > smallest_dis_check)):
            shoot = 150
            kissball = 0
            print("minimum power !!")
        elif (good_target[0][4] < 60):
            shoot = 150
            kissball = 0
            print("close hole !!")  
        else:
            shoot = 150
            kissball = 0
            print("不在判斷內，小力")

    elif (len(good_target) < 1)  and (len(OB_target) >= 1):
        OB_target.sort(key = lambda x: x[4])
        print("Successfully set a kissball target C : [{}, {:4.0f}, {:4.0f}]".format(OB_target[0][0], OB_target[0][1], OB_target[0][2]))
        C = [OB_target[0][1], OB_target[0][2], OB_target[0][3]]
        print("back_OB_storted : ",C[2])
        shoot = 255
        kissball = 1
        print("minimum power !!")

    elif (len(good_target) < 1) and (len(OB_target) < 1) and (bad_target >= len(BigAngle_Good_target)):
        shoot = 300
        luckyshoot = 1
        print("luckyball !!")

    elif (len(good_target) < 1) and (len(OB_target) < 1) and (bad_target < len(BigAngle_Good_target)) and (len(BigAngle_Good_target) >= 1):
        # BigAngle_Good_target.sort(key = lambda x: x[3])
        BigAngle_Good_target.sort(key = lambda x: x[4])
        print("Successfully set a good but big angle target C : [{}, {:4.0f}, {:4.0f}]".format(BigAngle_Good_target[0][0], BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]))
        C = [BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]]
        if ((BigAngle_Good_target[0][3] >= hit_power_check) and (BigAngle_Good_target[0][4] >= hit_power_check)) and (BigAngle_Good_target[0][4] >= 60):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (BigAngle_Good_target[0][4] < 60):
            shoot = 150
            kissball = 0
            print("close hole !!")  
        else:
            shoot = 150
            kissball = 0
            print("minimum power !!")

    else:
        shoot = 300
        luckyshoot = 1
        print("luckyball !!")
    
    print("============================================================================")


def luckyball():
    global C, M, target_angle
    global ball_storted
    global billiard_radius, gHole
    global H1,H2,H3,H4,H5,H6
    global kissball 

    CM = np.zeros(int(len(ball_storted)))  #各子球與其他子球的距離和（判斷中心球）
    i = 0; ball_remoMM=[]
    for item in ball_storted:
        if item[0] != 'M':
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
    print('come on jackpod !!!!')


def check_front():
    global M
    global gHole,billiard_radius
    global ball_storted
    global OB_front

    print("===front===")
    dis_limit = 2.3*billiard_radius

    counter = 0

    TCCW = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])
    TCW = np.array([[0, 1],      #+時針旋轉的旋轉矩陣
                   [-1,  0]])

    C2H = np.array([gHole[0]-C[0], gHole[1]-C[1]])
    L_C2H = np.linalg.norm(C2H)
    Unit_C2H = C2H/L_C2H
    TCCW_Unit_C2H = TCCW.dot(Unit_C2H)
    TCW_Unit_C2H = TCW.dot(Unit_C2H)   
    TCCW_lim_C2H = dis_limit*TCCW_Unit_C2H
    TCW_lim_C2H = dis_limit*TCW_Unit_C2H

    TCCW_C2H = [TCCW_lim_C2H[0] + C[0],TCCW_lim_C2H[1] + C[1]]
    TCW_C2H = [TCW_lim_C2H[0] + C[0],TCW_lim_C2H[1] + C[1]]

    H2C = np.array([C[0]-gHole[0], C[1]-gHole[1]])
    L_H2C = np.linalg.norm(H2C)
    Unit_H2C = H2C/L_H2C
    TCCW_Unit_H2C = TCCW.dot(Unit_H2C)
    TCW_Unit_H2C = TCW.dot(Unit_H2C)
    TCCW_lim_H2C = dis_limit*TCCW_Unit_H2C
    TCW_lim_H2C = dis_limit*TCW_Unit_H2C

    TCCW_H2C = [TCCW_lim_H2C[0] + gHole[0],TCCW_lim_H2C[1] + gHole[1]]
    TCW_H2C = [TCW_lim_H2C[0] + gHole[0],TCW_lim_H2C[1] + gHole[1]]

    TCCW_C2HtoTCW_C2H = [TCW_C2H[0]-TCCW_C2H[0], TCW_C2H[1]-TCCW_C2H[1]]
    TCW_C2HtoTCCW_H2C = [TCCW_H2C[0]-TCW_C2H[0], TCCW_H2C[1]-TCW_C2H[1]]
    TCCW_H2CtoTCW_H2C = [TCW_H2C[0]-TCCW_H2C[0], TCW_H2C[1]-TCCW_H2C[1]]
    TCW_H2CtoTCCW_C2H = [TCCW_C2H[0]-TCW_H2C[0], TCCW_C2H[1]-TCW_H2C[1]]

    for items in ball_storted:
        if (items[0] != 'M') and ([items[2],items[3]] != C) and (counter < 2):
            OB_front = [items[2], items[3]]

            TCCW_C2HtoOB_front = [OB_front[0]-TCCW_C2H[0], OB_front[1]-TCCW_C2H[1]]
            TCW_C2HtoOB_front = [OB_front[0]-TCW_C2H[0], OB_front[1]-TCW_C2H[1]]
            TCCW_H2CtoOB_front = [OB_front[0]-TCCW_H2C[0], OB_front[1]-TCCW_H2C[1]]
            TCW_H2CtoOB_front = [OB_front[0]-TCW_H2C[0], OB_front[1]-TCW_H2C[1]]

            C2OB_front = np.array([OB_front[0] - C[0], OB_front[1] - C[1]])            #子球與干擾球之向量
            L = abs(np.cross(C2H, C2OB_front) / np.linalg.norm(C2H))                   #用面積(平面向量外積)除於底(母球與虛擬球之向量)得到高 = 母球與虛擬球的連線到干擾球之距離

            if gHole[1] >= C[1]:
                if gHole[0] >= C[0]:
                    pos_C2H_normal = np.cross(TCCW_C2HtoOB_front, TCCW_C2HtoTCW_C2H)
                    pos_parallel_R = np.cross(TCW_C2HtoTCCW_H2C, TCW_C2HtoOB_front)
                    pos_H2C_normal = np.cross(TCCW_H2CtoTCW_H2C, TCCW_H2CtoOB_front)
                    pos_parallel_L = np.cross(TCW_H2CtoOB_front, TCW_H2CtoTCCW_C2H)
                    if (L < dis_limit) and (pos_C2H_normal < 0) and (pos_parallel_R > 0) and (pos_H2C_normal > 0) and (pos_parallel_L < 0):
                        counter = counter + 1
                elif gHole[0] < C[0]:
                    pos_C2H_normal = np.cross(TCCW_C2HtoTCW_C2H, TCCW_C2HtoOB_front)
                    pos_parallel_R = np.cross(TCW_C2HtoTCCW_H2C, TCW_C2HtoOB_front)
                    pos_H2C_normal = np.cross(TCCW_H2CtoOB_front, TCCW_H2CtoTCW_H2C)
                    pos_parallel_L = np.cross(TCW_H2CtoOB_front, TCW_H2CtoTCCW_C2H)
                    if (L < dis_limit) and (pos_C2H_normal > 0) and (pos_parallel_R > 0) and (pos_H2C_normal < 0) and (pos_parallel_L < 0):
                        counter = counter + 1
            elif gHole[1] < C[1]:
                if gHole[0] >= C[0]:
                    pos_C2H_normal = np.cross(TCCW_C2HtoOB_front, TCCW_C2HtoTCW_C2H)
                    pos_parallel_R = np.cross(TCW_C2HtoOB_front, TCW_C2HtoTCCW_H2C)
                    pos_H2C_normal = np.cross(TCCW_H2CtoTCW_H2C, TCCW_H2CtoOB_front)
                    pos_parallel_L = np.cross(TCW_H2CtoTCCW_C2H, TCW_H2CtoOB_front)
                    if (L < dis_limit) and (pos_C2H_normal < 0) and (pos_parallel_R < 0) and (pos_H2C_normal > 0) and (pos_parallel_L > 0):
                        counter = counter + 1
                elif gHole[0] < C[0]:
                    pos_C2H_normal = np.cross(TCCW_C2HtoTCW_C2H, TCCW_C2HtoOB_front)
                    pos_parallel_R = np.cross(TCW_C2HtoOB_front, TCW_C2HtoTCCW_H2C)
                    pos_H2C_normal = np.cross(TCCW_H2CtoOB_front, TCCW_H2CtoTCW_H2C)
                    pos_parallel_L = np.cross(TCW_H2CtoTCCW_C2H, TCW_H2CtoOB_front)
                    if (L < dis_limit) and (pos_C2H_normal > 0) and (pos_parallel_R < 0) and (pos_H2C_normal < 0) and (pos_parallel_L > 0):
                        counter = counter + 1
        elif counter >= 2:
            break
    print("front counter:",counter)
    return counter 


def check_back():
    global M,C
    global ball_storted
    global gHole,billiard_radius
    global back_OB_storted

    print("===back===")
    back_OB_storted = [[]for i in range(3)]           #干擾球宣告用
    counter = 0
    dis_limit = 2.3*billiard_radius

    TCCW = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])
    TCW = np.array([[0, 1],      #+時針旋轉的旋轉矩陣
                   [-1,  0]])

    M2C = np.array([C[0]-M[0], C[1]-M[1]])
    L_M2V = np.linalg.norm(M2C)
    Unit_M2C = M2C/L_M2V
    TCCW_Unit_M2C = TCCW.dot(Unit_M2C)
    TCW_Unit_M2C = TCW.dot(Unit_M2C)   
    TCCW_lim_M2C = dis_limit*TCCW_Unit_M2C
    TCW_lim_M2C = dis_limit*TCW_Unit_M2C

    TCCW_M2C = [TCCW_lim_M2C[0] + M[0],TCCW_lim_M2C[1] + M[1]]
    TCW_M2C = [TCW_lim_M2C[0] + M[0],TCW_lim_M2C[1] + M[1]]

    C2M = np.array([M[0]-C[0], M[1]-C[1]])
    L_C2M = np.linalg.norm(C2M)
    Unit_C2M = C2M/L_C2M
    TCCW_Unit_C2M = TCCW.dot(Unit_C2M)
    TCW_Unit_C2M = TCW.dot(Unit_C2M)
    TCCW_lim_C2M = dis_limit*TCCW_Unit_C2M
    TCW_lim_C2M = dis_limit*TCW_Unit_C2M

    TCCW_C2M = [TCCW_lim_C2M[0] + C[0],TCCW_lim_C2M[1] + C[1]]
    TCW_C2M = [TCW_lim_C2M[0] + C[0],TCW_lim_C2M[1] + C[1]]

    TCCW_M2CtoTCW_M2C = [TCW_M2C[0]-TCCW_M2C[0], TCW_M2C[1]-TCCW_M2C[1]]
    TCW_M2CtoTCCW_C2M = [TCCW_C2M[0]-TCW_M2C[0], TCCW_C2M[1]-TCW_M2C[1]]
    TCCW_C2MtoTCW_C2M = [TCW_C2M[0]-TCCW_C2M[0], TCW_C2M[1]-TCCW_C2M[1]]
    TCW_C2MtoTCCW_M2C = [TCCW_M2C[0]-TCW_C2M[0], TCCW_M2C[1]-TCW_C2M[1]]

    for items in ball_storted:
        if (items[0] != 'M') and ([items[2],items[3]] != C) and (counter < 2):
            OB_back = [items[2], items[3]]

            TCCW_M2VtoOB_back = [OB_back[0]-TCCW_M2C[0], OB_back[1]-TCCW_M2C[1]]
            TCW_M2VtoOB_back = [OB_back[0]-TCW_M2C[0], OB_back[1]-TCW_M2C[1]]
            TCCW_V2MtoOB_back = [OB_back[0]-TCCW_C2M[0], OB_back[1]-TCCW_C2M[1]]
            TCW_V2MtoOB_back = [OB_back[0]-TCW_C2M[0], OB_back[1]-TCW_C2M[1]]

            C2OB_back = np.array([OB_back[0] - M[0], OB_back[1] - M[1]])              #母球與干擾球之向量
            L = abs(np.cross(M2C, C2OB_back) / np.linalg.norm(M2C))                   #用面積(平面向量外積)除於底(母球與虛擬球之向量)得到高 = 母球與虛擬球的連線到干擾球之距離

            if C[1] >= M[1]:
                if C[0] >= M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoOB_back, TCCW_M2CtoTCW_M2C)
                    pos_parallel_R = np.cross(TCW_M2CtoTCCW_C2M, TCW_M2VtoOB_back)
                    pos_V2M_normal = np.cross(TCCW_C2MtoTCW_C2M, TCCW_V2MtoOB_back)
                    pos_parallel_L = np.cross(TCW_V2MtoOB_back, TCW_C2MtoTCCW_M2C)
                    if (L < dis_limit) and (pos_M2V_normal < 0) and (pos_parallel_R > 0) and (pos_V2M_normal > 0) and (pos_parallel_L < 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
                elif C[0] < M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2CtoTCW_M2C, TCCW_M2VtoOB_back)
                    pos_parallel_R = np.cross(TCW_M2CtoTCCW_C2M, TCW_M2VtoOB_back)
                    pos_V2M_normal = np.cross(TCCW_V2MtoOB_back, TCCW_C2MtoTCW_C2M)
                    pos_parallel_L = np.cross(TCW_V2MtoOB_back, TCW_C2MtoTCCW_M2C)
                    if (L < dis_limit) and (pos_M2V_normal > 0) and (pos_parallel_R > 0) and (pos_V2M_normal < 0) and (pos_parallel_L < 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
            elif C[1] < M[1]:
                if C[0] >= M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoOB_back, TCCW_M2CtoTCW_M2C)
                    pos_parallel_R = np.cross(TCW_M2VtoOB_back, TCW_M2CtoTCCW_C2M)
                    pos_V2M_normal = np.cross(TCCW_C2MtoTCW_C2M, TCCW_V2MtoOB_back)
                    pos_parallel_L = np.cross(TCW_C2MtoTCCW_M2C, TCW_V2MtoOB_back)
                    if (L < dis_limit) and (pos_M2V_normal < 0) and (pos_parallel_R < 0) and (pos_V2M_normal > 0) and (pos_parallel_L > 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
                elif C[0] < M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2CtoTCW_M2C, TCCW_M2VtoOB_back)
                    pos_parallel_R = np.cross(TCW_M2VtoOB_back, TCW_M2CtoTCCW_C2M)
                    pos_V2M_normal = np.cross(TCCW_V2MtoOB_back, TCCW_C2MtoTCW_C2M)
                    pos_parallel_L = np.cross(TCW_C2MtoTCCW_M2C, TCW_V2MtoOB_back)
                    if (L < dis_limit) and (pos_M2V_normal > 0) and (pos_parallel_R < 0) and (pos_V2M_normal < 0) and (pos_parallel_L > 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
        elif counter >= 2:
            break
    print("back counter:",counter)
    return counter 

def good_angle(mode):
    global M, C, gHole
    global billiard_radius
    global extend_V, extend_V1, extend_V2, point_TCCW45, point_TCW45, point_CC45, point_C45
    global fin_point_TCCW45, fin_point_TCW45
    global back_OB_storted
    global V

    TCCW45 = np.array([[0.707, -1*0.707],
                       [0.707, 0.707]])

    TCW45 = np.array([[0.707, -1*(-0.707)],
                      [-0.707, 0.707]])

    if mode == 'good':
        C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        L_C2H = np.linalg.norm(C2H)                       #CH向量的長度
        Unit_C2H = C2H / L_C2H                                #CH向量的單位向量  
        V = [ (C[0] - 2*(billiard_radius)*Unit_C2H[0]), (C[1] - 2*(billiard_radius) * Unit_C2H[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 
        extend_V = [(C[0] - 10*(billiard_radius)*Unit_C2H[0]), (C[1] - 10*(billiard_radius)*Unit_C2H[1])]

        V2extend_V = [extend_V[0]-V[0], extend_V[1]-V[1]]
        TCCW45_V2extend_V = TCCW45.dot(V2extend_V)
        TCW45_V2extend_V = TCW45.dot(V2extend_V)
        point_TCCW45 = [TCCW45_V2extend_V[0]+V[0], TCCW45_V2extend_V[1]+V[1]]
        point_TCW45 = [TCW45_V2extend_V[0]+V[0], TCW45_V2extend_V[1]+V[1]]

        V2M = [M[0]-V[0], M[1]-V[1]]

        pos_TCCW45 = np.cross(TCCW45_V2extend_V, V2M)
        pos_TCW = np.cross(TCW45_V2extend_V, V2M)
        print("pos_TCCW45 : ",pos_TCCW45)
        print("pos_TCW : ",pos_TCW)

        if (pos_TCCW45 < 0) and (pos_TCW > 0):
            return 1
        else:
            return 0
    
    if mode == 'OB':
        C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])                                       
        L_C2H = np.linalg.norm(C2H)                                     
        Unit_C2H = C2H / L_C2H                                             
        V1 = [ (C[0] - 2*(billiard_radius) * Unit_C2H[0]), (C[1] - 2*(billiard_radius) * Unit_C2H[1]) ]    
        extend_V1 = [C[0] - 10*(billiard_radius)*Unit_C2H[0], C[1] - 10*(billiard_radius)*Unit_C2H[1]]

        V12extend_V1 = [extend_V1[0]-V1[0], extend_V1[0]-V1[0]]
        TCCW45_V12extend_V1 = TCCW45.dot(V12extend_V1)
        TCW45_V12extend_V1 = TCW45.dot(V12extend_V1)
        point_CC45 = [TCCW45_V12extend_V1[0]+V1[0], TCCW45_V12extend_V1[1]+V1[1]]
        point_C45 = [TCW45_V12extend_V1[0]+V1[0], TCW45_V12extend_V1[1]+V1[1]]

        V12OB = [back_OB_storted[0][0]-V1[0], back_OB_storted[0][1]-V1[1]]

        pos_CC45 = np.dot(TCCW45_V12extend_V1, V12OB)
        pos_C45 = np.dot(TCW45_V12extend_V1, V12OB)
        print("pos_CC45 : ",pos_CC45)
        print("pos_C45 : ", pos_C45)

        if (pos_CC45 < 0) and (pos_C45 > 0):
            print("子球好角度")
            OB2V1 = np.array([V1[0]-back_OB_storted[0][0], V1[1]-back_OB_storted[0][1]])
            L_OB2V1 = np.linalg.norm(OB2V1)
            Unit_OB2V1 = OB2V1/L_OB2V1
            V2 = [(back_OB_storted[0][0] - 2*(billiard_radius)*Unit_OB2V1[0]), (back_OB_storted[0][1] - 2*(billiard_radius)*Unit_OB2V1[1])]
            extend_V2 = [(back_OB_storted[0][0] - 10*(billiard_radius)*Unit_OB2V1[0]), (back_OB_storted[0][1] - 10*(billiard_radius)*Unit_OB2V1[1])]

            V22extend_v2 = [extend_V2[0]-V2[0], extend_V2[1]-V2[1]]
            TCCW45_V22extend_v2 = TCCW45.dot(V22extend_v2)
            TCW45_V22extend_v2 = TCW45.dot(V22extend_v2)
            point_TCCW45 = [TCCW45_V22extend_v2[0]+V2[0], TCCW45_V22extend_v2[1]+V2[1]]
            point_TCW45 = [TCW45_V22extend_v2[0]+V2[0], TCW45_V22extend_v2[1]+V2[1]]

            V22M = [M[0]-V2[0], M[1]-V2[1]]

            pos_TCCW45 = np.cross(TCCW45_V22extend_v2, V22M)
            pos_TCW = np.cross(TCW45_V22extend_v2, V22M)
            print("pos_TCCW45 : ",pos_TCCW45)
            print("pos_TCW : ", pos_TCW)

            if (pos_TCCW45 < 0) and (pos_TCW > 0):
                print("干擾球好角度")
                return 1
            else:
                print("干擾球大角度")
                return 0
        else:
            print("子球大角度")
            return 0

    if mode == 'OB_plot':
        C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])                                       
        L_C2H = np.linalg.norm(C2H)                                     
        Unit_C2H = C2H / L_C2H                                             
        V1 = [ (C[0] - 2*(billiard_radius) * Unit_C2H[0]), (C[1] - 2*(billiard_radius) * Unit_C2H[1]) ]    

        OB2V1 = np.array([V1[0]-C[2][0], V1[1]-C[2][1]])
        L_OB2V1 = np.linalg.norm(OB2V1)
        Unit_OB2V1 = OB2V1/L_OB2V1
        V2 = [(C[2][0] - 2*(billiard_radius)*Unit_OB2V1[0]), (C[2][1] - 2*(billiard_radius)*Unit_OB2V1[1])]
        extend_V2 = [(C[2][0] - 10*(billiard_radius)*Unit_OB2V1[0]), (C[2][1] - 10*(billiard_radius)*Unit_OB2V1[1])]

        V22extend_v2 = [extend_V2[0]-V2[0], extend_V2[1]-V2[1]]
        TCCW45_V22extend_v2 = TCCW45.dot(V22extend_v2)
        TCW45_V22extend_v2 = TCW45.dot(V22extend_v2)
        fin_point_TCCW45 = [TCCW45_V22extend_v2[0]+V2[0], TCCW45_V22extend_v2[1]+V2[1]]
        fin_point_TCW45 = [TCW45_V22extend_v2[0]+V2[0], TCW45_V22extend_v2[1]+V2[1]]

def good_M_back():
    global M, C, V, V1, V2
    global ball_storted, gHole
    global billiard_radius
    global TCCW_M2V, TCW_M2V, TCCW_back_ex2M, TCW_back_ex2M

    print("good back check !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    counter = 0
    dis_limit = 40.9

    TCCW = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])
    TCW = np.array([[0, 1],      #+時針旋轉的旋轉矩陣
                   [-1,  0]])
    
    M2V = np.array([V[0]-M[0], V[1]-M[1]])
    L_M2V = np.linalg.norm(M2V)
    Unit_M2V = M2V/L_M2V

    TCCW_Unit_M2V = TCCW.dot(Unit_M2V)
    TCW_Unit_M2V = TCW.dot(Unit_M2V)
    TCCW_lim_M2V = dis_limit*TCCW_Unit_M2V
    TCW_lim_M2V = dis_limit*TCW_Unit_M2V

    TCCW_M2V = [TCCW_lim_M2V[0]+M[0], TCCW_lim_M2V[1]+M[1]]
    TCW_M2V = [TCW_lim_M2V[0]+M[0], TCW_lim_M2V[1]+M[1]]

    back_extend = [M[0] - (95.8)*Unit_M2V[0], M[1] - (95.8)*Unit_M2V[1]]
    back_extend2M = [M[0]-back_extend[0], M[1]-back_extend[1]]
    L_back_extend2M = np.linalg.norm(back_extend2M)
    Unit_back_extend2M = back_extend2M/L_back_extend2M

    TCCW_Unit_back_extend2M = TCCW.dot(Unit_back_extend2M)
    TCW_Unit_back_extend2M = TCW.dot(Unit_back_extend2M)
    TCCW_lim_back_extend2M = dis_limit*TCCW_Unit_back_extend2M
    TCW_lim_back_extend2M = dis_limit*TCW_Unit_back_extend2M

    TCCW_back_ex2M = [TCCW_lim_back_extend2M[0]+back_extend[0], TCCW_lim_back_extend2M[1]+back_extend[1]]
    TCW_back_ex2M = [TCW_lim_back_extend2M[0]+back_extend[0], TCW_lim_back_extend2M[1]+back_extend[1]]

    TCCW_M2VtoTCW_M2V = [TCW_M2V[0]-TCCW_M2V[0], TCW_M2V[1]-TCCW_M2V[1]]
    TCW_M2VtoTCW_back_ex2M = [TCW_back_ex2M[0]-TCW_M2V[0], TCW_back_ex2M[1]-TCW_M2V[1]]
    TCW_back_ex2MtoTCCW_back_ex2M = [TCCW_back_ex2M[0]-TCW_back_ex2M[0], TCCW_back_ex2M[1]-TCW_back_ex2M[1]]
    TCCW_back_ex2MtotTCCW_M2V = [TCCW_M2V[0]-TCCW_back_ex2M[0], TCCW_M2V[1]-TCCW_back_ex2M[1]]

    print("TCCW_M2V : ",TCCW_M2V)
    print("TCW_M2V : ",TCW_M2V)
    print("TCCW_back_ex2M", TCCW_back_ex2M)
    print("TCW_back_ex2M", TCW_back_ex2M)

    for items in ball_storted:
        if (items[0] != 'M') and ([items[2],items[3]] != C) and (counter < 1):
            back_ball = [items[2], items[3]]
            print("目前障礙 : ",back_ball)

            TCCW_M2Vtoback_ball = [back_ball[0]-TCCW_M2V[0], back_ball[1]-TCCW_M2V[1]]
            TCW_M2Vtoback_ball = [back_ball[0]-TCW_M2V[0], back_ball[1]-TCW_M2V[1]]
            TCCW_back_ex2Mtoback_ball = [back_ball[0]-TCCW_back_ex2M[0], back_ball[1]-TCCW_back_ex2M[1]]
            TCW_back_ex2Mtoback_ball = [back_ball[0]-TCW_back_ex2M[0], back_ball[1]-TCW_back_ex2M[1]]

            if V[1] >= M[1]:
                if V[0] >= M[0]:
                    print("1111111")
                    pos_M2V_normal = np.cross(TCCW_M2Vtoback_ball, TCCW_M2VtoTCW_M2V)
                    pos_paralll_R = np.cross(TCW_M2Vtoback_ball, TCW_M2VtoTCW_back_ex2M)
                    pos_back_extend2M_normal = np.cross(TCW_back_ex2MtoTCCW_back_ex2M, TCW_back_ex2Mtoback_ball)
                    pos_parallel_L = np.cross(TCCW_back_ex2MtotTCCW_M2V, TCCW_back_ex2Mtoback_ball)

                    print("pos_M2V_normal : ",pos_M2V_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_back_extend2M_normal : ",pos_back_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V_normal > 0) and (pos_paralll_R > 0) and (pos_back_extend2M_normal < 0) and (pos_parallel_L < 0):
                        print("**")
                        print("A ball block my hit!!")
                        counter = counter + 1

                elif V[0] < M[0]:
                    print("2222222")
                    pos_M2V_normal = np.cross(TCCW_M2VtoTCW_M2V, TCCW_M2Vtoback_ball)
                    pos_paralll_R = np.cross(TCW_M2Vtoback_ball, TCW_M2VtoTCW_back_ex2M)
                    pos_back_extend2M_normal = np.cross(TCW_back_ex2Mtoback_ball, TCW_back_ex2MtoTCCW_back_ex2M)
                    pos_parallel_L = np.cross(TCCW_back_ex2MtotTCCW_M2V, TCCW_back_ex2Mtoback_ball)

                    print("pos_M2V_normal : ",pos_M2V_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_back_extend2M_normal : ",pos_back_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V_normal < 0) and (pos_paralll_R > 0) and (pos_back_extend2M_normal > 0) and (pos_parallel_L < 0):
                        print("/*")
                        print("A ball block my hit!!")
                        counter = counter + 1

            elif V[1] < M[1]:
                if V[0] >= M[0]:
                    print("3333333")
                    pos_M2V_normal = np.cross(TCCW_M2Vtoback_ball, TCCW_M2VtoTCW_M2V)
                    pos_paralll_R = np.cross(TCW_M2VtoTCW_back_ex2M, TCW_M2Vtoback_ball)
                    pos_back_extend2M_normal = np.cross(TCW_back_ex2MtoTCCW_back_ex2M, TCW_back_ex2Mtoback_ball)
                    pos_parallel_L = np.cross(TCCW_back_ex2Mtoback_ball, TCCW_back_ex2MtotTCCW_M2V)

                    print("pos_M2V_normal : ",pos_M2V_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_back_extend2M_normal : ",pos_back_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V_normal > 0) and (pos_paralll_R < 0) and (pos_back_extend2M_normal < 0) and (pos_parallel_L > 0):
                        print("*/")
                        print("A ball block my hit!!")
                        counter = counter + 1

                elif V[0] < M[0]:
                    print("4444444")
                    pos_M2V_normal = np.cross(TCCW_M2VtoTCW_M2V, TCCW_M2Vtoback_ball)
                    pos_paralll_R = np.cross(TCW_M2VtoTCW_back_ex2M, TCW_M2Vtoback_ball)
                    pos_back_extend2M_normal = np.cross(TCW_back_ex2Mtoback_ball, TCW_back_ex2MtoTCCW_back_ex2M)
                    pos_parallel_L = np.cross(TCCW_back_ex2Mtoback_ball, TCCW_back_ex2MtotTCCW_M2V)

                    print("pos_M2V_normal : ",pos_M2V_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_back_extend2M_normal : ",pos_back_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V_normal < 0) and (pos_paralll_R < 0) and (pos_back_extend2M_normal > 0) and (pos_parallel_L > 0):
                        print("//")
                        print("A ball block my hit!!")
                        counter = counter + 1
        elif counter >= 1:
            break

    return counter

def OB_M_back():
    global M, C, V, V1, V2
    global ball_storted, gHole
    global billiard_radius
    global TCCW_M2V2, TCW_M2V2, TCCW_OBback_ex2M, TCW_OBback_ex2M

    TCCW_M2V2 = [0, 0]
    TCW_M2V2 = [0, 0]
    TCCW_OBback_ex2M = [0, 0]
    TCW_OBback_ex2M = [0, 0]

    counter = 0
    dis_limit = 40.9
    print("OB back check !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    TCCW = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])
    TCW = np.array([[0, 1],      #+時針旋轉的旋轉矩陣
                   [-1,  0]])
    
    M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
    L_M2V2 = np.linalg.norm(M2V2)
    Unit_M2V2 = M2V2/L_M2V2

    TCCW_Unit_M2V2 = TCCW.dot(Unit_M2V2)
    TCW_Unit_M2V2 = TCW.dot(Unit_M2V2)
    TCCW_lim_M2V2 = dis_limit*TCCW_Unit_M2V2
    TCW_lim_M2V2 = dis_limit*TCW_Unit_M2V2

    TCCW_M2V2 = [TCCW_lim_M2V2[0]+M[0], TCCW_lim_M2V2[1]+M[1]]
    TCW_M2V2 = [TCW_lim_M2V2[0]+M[0], TCW_lim_M2V2[1]+M[1]]

    OBback_extend = [M[0] - (95.8)*Unit_M2V2[0], M[1] - (95.8)*Unit_M2V2[1]]
    OBback_extend2M = [M[0]-OBback_extend[0], M[1]-OBback_extend[1]]
    L_OBback_extend2M = np.linalg.norm(OBback_extend2M)
    Unit_OBback_extend2M = OBback_extend2M/L_OBback_extend2M

    TCCW_Unit_OBback_extend2M = TCCW.dot(Unit_OBback_extend2M)
    TCW_Unit_OBback_extend2M = TCW.dot(Unit_OBback_extend2M)
    TCCW_lim_OBback_extend2M = dis_limit*TCCW_Unit_OBback_extend2M
    TCW_lim_OBback_extend2M = dis_limit*TCW_Unit_OBback_extend2M

    TCCW_OBback_ex2M = [TCCW_lim_OBback_extend2M[0]+OBback_extend[0], TCCW_lim_OBback_extend2M[1]+OBback_extend[1]]
    TCW_OBback_ex2M = [TCW_lim_OBback_extend2M[0]+OBback_extend[0], TCW_lim_OBback_extend2M[1]+OBback_extend[1]]

    TCCW_M2V2toTCW_M2V2 = [TCW_M2V2[0]-TCCW_M2V2[0], TCW_M2V2[1]-TCCW_M2V2[1]]
    TCW_M2V2toTCW_OBback_ex2M = [TCW_OBback_ex2M[0]-TCW_M2V2[0], TCW_OBback_ex2M[1]-TCW_M2V2[1]]
    TCW_OBback_ex2MtoTCCW_OBback_ex2M = [TCCW_OBback_ex2M[0]-TCW_OBback_ex2M[0], TCCW_OBback_ex2M[1]-TCW_OBback_ex2M[1]]
    TCCW_OBback_ex2MtotTCCW_M2V2 = [TCCW_M2V2[0]-TCCW_OBback_ex2M[0], TCCW_M2V2[1]-TCCW_OBback_ex2M[1]]


    print("TCCW_M2V2 : ",TCCW_M2V2)
    print("TCW_M2V2 : ",TCW_M2V2)
    print("TCCW_OBback_ex2M", TCCW_OBback_ex2M)
    print("TCW_OBback_ex2M", TCW_OBback_ex2M)

    for items in ball_storted:
        if (items[0] != 'M') and ([items[2],items[3]] != C) and (counter < 1):
            back_ball = [items[2], items[3]]
            print("目前障礙 : ",back_ball)

            TCCW_M2V2toback_ball = [back_ball[0]-TCCW_M2V2[0], back_ball[1]-TCCW_M2V2[1]]
            TCW_M2V2toback_ball = [back_ball[0]-TCW_M2V2[0], back_ball[1]-TCW_M2V2[1]]
            TCCW_OBback_ex2Mtoback_ball = [back_ball[0]-TCCW_OBback_ex2M[0], back_ball[1]-TCCW_OBback_ex2M[1]]
            TCW_OBback_ex2Mtoback_ball = [back_ball[0]-TCW_OBback_ex2M[0], back_ball[1]-TCW_OBback_ex2M[1]]

            if V2[1] >= M[1]:
                if V2[0] >= M[0]:
                    pos_M2V2_normal = np.cross(TCCW_M2V2toback_ball, TCCW_M2V2toTCW_M2V2)
                    pos_paralll_R = np.cross(TCW_M2V2toback_ball, TCW_M2V2toTCW_OBback_ex2M)
                    pos_OBback_extend2M_normal = np.cross(TCW_OBback_ex2MtoTCCW_OBback_ex2M, TCW_OBback_ex2Mtoback_ball)
                    pos_parallel_L = np.cross(TCCW_OBback_ex2MtotTCCW_M2V2, TCCW_OBback_ex2Mtoback_ball)

                    print("pos_M2V2_normal : ",pos_M2V2_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_OBback_extend2M_normal : ",pos_OBback_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V2_normal > 0) and (pos_paralll_R > 0) and (pos_OBback_extend2M_normal < 0) and (pos_parallel_L < 0):
                        print("++")
                        print("A ball block my hit!!")
                        counter = counter + 1

                elif V2[0] < M[0]:
                    pos_M2V2_normal = np.cross(TCCW_M2V2toTCW_M2V2, TCCW_M2V2toback_ball)
                    pos_paralll_R = np.cross(TCW_M2V2toback_ball, TCW_M2V2toTCW_OBback_ex2M)
                    pos_OBback_extend2M_normal = np.cross(TCW_OBback_ex2Mtoback_ball, TCW_OBback_ex2MtoTCCW_OBback_ex2M)
                    pos_parallel_L = np.cross(TCCW_OBback_ex2MtotTCCW_M2V2, TCCW_OBback_ex2Mtoback_ball)

                    print("pos_M2V2_normal : ",pos_M2V2_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_OBback_extend2M_normal : ",pos_OBback_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V2_normal < 0) and (pos_paralll_R > 0) and (pos_OBback_extend2M_normal > 0) and (pos_parallel_L < 0):
                        print("-+")
                        print("A ball block my hit!!")
                        counter = counter + 1

            elif V2[1] < M[1]:
                if V2[0] >= M[0]:
                    pos_M2V2_normal = np.cross(TCCW_M2V2toback_ball, TCCW_M2V2toTCW_M2V2)
                    pos_paralll_R = np.cross(TCW_M2V2toTCW_OBback_ex2M, TCW_M2V2toback_ball)
                    pos_OBback_extend2M_normal = np.cross(TCW_OBback_ex2MtoTCCW_OBback_ex2M, TCW_OBback_ex2Mtoback_ball)
                    pos_parallel_L = np.cross(TCCW_OBback_ex2Mtoback_ball, TCCW_OBback_ex2MtotTCCW_M2V2)

                    print("pos_M2V2_normal : ",pos_M2V2_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_OBback_extend2M_normal : ",pos_OBback_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V2_normal > 0) and (pos_paralll_R < 0) and (pos_OBback_extend2M_normal < 0) and (pos_parallel_L > 0):
                        print("+-")
                        print("A ball block my hit!!")
                        counter = counter + 1

                elif V2[0] < M[0]:
                    pos_M2V2_normal = np.cross(TCCW_M2V2toTCW_M2V2, TCCW_M2V2toback_ball)
                    pos_paralll_R = np.cross(TCW_M2V2toTCW_OBback_ex2M, TCW_M2V2toback_ball)
                    pos_OBback_extend2M_normal = np.cross(TCW_OBback_ex2Mtoback_ball, TCW_OBback_ex2MtoTCCW_OBback_ex2M)
                    pos_parallel_L = np.cross(TCCW_OBback_ex2Mtoback_ball, TCCW_OBback_ex2MtotTCCW_M2V2)

                    print("pos_M2V2_normal : ",pos_M2V2_normal)
                    print("pos_paralll_R : ",pos_paralll_R)
                    print("pos_OBback_extend2M_normal : ",pos_OBback_extend2M_normal)
                    print("pos_parallel_L : ",pos_parallel_L)
                    print("/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*")
                    if (pos_M2V2_normal < 0) and (pos_paralll_R < 0) and (pos_OBback_extend2M_normal > 0) and (pos_parallel_L > 0):
                        print("--")
                        print("A ball block my hit!!")
                        counter = counter + 1
        elif counter >= 1:
            break

    return counter

def boundary_back(mode):
    global M, C, V, V2
    global H1, H2, H3, H4, H5, H6
    global back_extend, CC45back_extend, C45back_extend
    global OBback_extend, OBCC45back_extend, OBC45back_extend

    TCCW45 = np.array([[0.707, -1*0.707],
                       [0.707, 0.707]])

    TCW45 = np.array([[0.707, -1*(-0.707)],
                      [-0.707, 0.707]])

    L_boundary = [H2[0]-H1[0], H2[1]-H1[1]]
    U_boundary = [H4[0]-H2[0], H4[1]-H2[1]]
    R_boundary = [H5[0]-H4[0], H5[1]-H4[1]]
    D_boundary = [H1[0]-H5[0], H1[1]-H5[1]]

    if mode == 'OB':
        M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
        L_M2V2 = np.linalg.norm(M2V2)
        Unit_M2V2 = M2V2/L_M2V2
        OBback_extend = [M[0] - (billiard_radius*5.5)*Unit_M2V2[0], M[1] - (billiard_radius*5.5)*Unit_M2V2[1]]
        CC45_Unit_M2V2 = TCCW45.dot(Unit_M2V2)
        C45_Unit_M2V2 = TCW45.dot(Unit_M2V2)
        OBCC45back_extend = [M[0] - (billiard_radius*5.5)*CC45_Unit_M2V2[0], M[1] - (billiard_radius*5.5)*CC45_Unit_M2V2[1]]
        OBC45back_extend = [M[0] - (billiard_radius*5.5)*C45_Unit_M2V2[0], M[1] - (billiard_radius*5.5)*C45_Unit_M2V2[1]]

        H1toOBback = [OBback_extend[0]-H1[0], OBback_extend[1]-H1[1]]
        H2toOBback = [OBback_extend[0]-H2[0], OBback_extend[1]-H2[1]]
        H4toOBback = [OBback_extend[0]-H4[0], OBback_extend[1]-H4[1]]
        H5toOBback = [OBback_extend[0]-H5[0], OBback_extend[1]-H5[1]]

        H1toOBCC45back_extend = [OBCC45back_extend[0]-H1[0], OBCC45back_extend[1]-H1[1]]
        H2toOBCC45back_extend = [OBCC45back_extend[0]-H2[0], OBCC45back_extend[1]-H2[1]]
        H3toOBCC45back_extend = [OBCC45back_extend[0]-H4[0], OBCC45back_extend[1]-H4[1]]
        H4toOBCC45back_extend = [OBCC45back_extend[0]-H5[0], OBCC45back_extend[1]-H5[1]]

        H1toOBC45back_extend = [OBC45back_extend[0]-H1[0], OBC45back_extend[1]-H1[1]]
        H2toOBC45back_extend = [OBC45back_extend[0]-H2[0], OBC45back_extend[1]-H2[1]]
        H3toOBC45back_extend = [OBC45back_extend[0]-H4[0], OBC45back_extend[1]-H4[1]]
        H4toOBC45back_extend = [OBC45back_extend[0]-H5[0], OBC45back_extend[1]-H5[1]]

        if H1[1] < H5[1]:
            pos_L_boundary = np.cross(L_boundary, H1toOBback)
            pos_U_boundary = np.cross(U_boundary, H2toOBback)
            pos_R_boundary = np.cross(H4toOBback, R_boundary)
            pos_D_boundary = np.cross(H5toOBback, D_boundary)

            pos_CC45_L_boundary = np.cross(L_boundary, H1toOBCC45back_extend)
            pos_CC45_U_boundary = np.cross(U_boundary, H2toOBCC45back_extend)
            pos_CC45_R_boundary = np.cross(H3toOBCC45back_extend, R_boundary)
            pos_CC45_D_biundary = np.cross(H4toOBCC45back_extend, D_boundary)

            pos_C45_L_boundary = np.cross(L_boundary, H1toOBC45back_extend)
            pos_C45_U_boundary = np.cross(U_boundary, H2toOBC45back_extend)
            pos_C45_R_boundary = np.cross(H3toOBC45back_extend, R_boundary)
            pos_C45_D_boundary = np.cross(H4toOBC45back_extend, D_boundary)

            if ((pos_L_boundary > 0) or (pos_U_boundary > 0) or (pos_R_boundary < 0) or (pos_D_boundary < 0)) or \
               ((pos_CC45_L_boundary > 0) or (pos_CC45_U_boundary > 0) or (pos_CC45_R_boundary < 0) or (pos_CC45_D_biundary < 0)) or \
               ((pos_C45_L_boundary > 0) or (pos_C45_U_boundary > 0) or (pos_C45_R_boundary < 0) or (pos_C45_D_boundary < 0)):
                return 1
            else:
                return 0            
        elif H1[1] > H5[1]:
            pos_L_boundary = np.cross(L_boundary, H1toOBback)
            pos_U_boundary = np.cross(H2toOBback, U_boundary)
            pos_R_boundary = np.cross(H4toOBback, R_boundary)
            pos_D_boundary = np.cross(D_boundary, H5toOBback) 

            pos_CC45_L_boundary = np.cross(L_boundary, H1toOBCC45back_extend)
            pos_CC45_U_boundary = np.cross(H2toOBCC45back_extend, U_boundary)
            pos_CC45_R_boundary = np.cross(H3toOBCC45back_extend, R_boundary)
            pos_CC45_D_biundary = np.cross(D_boundary, H4toOBCC45back_extend)

            pos_C45_L_boundary = np.cross(L_boundary, H1toOBC45back_extend)
            pos_C45_U_boundary = np.cross(H2toOBC45back_extend, U_boundary)
            pos_C45_R_boundary = np.cross(H3toOBC45back_extend, R_boundary)
            pos_C45_D_boundary = np.cross(D_boundary, H4toOBC45back_extend)

            if ((pos_L_boundary > 0) or (pos_U_boundary < 0) or (pos_R_boundary < 0) or (pos_D_boundary > 0)) or \
               ((pos_CC45_L_boundary > 0) or (pos_CC45_U_boundary < 0) or (pos_CC45_R_boundary < 0) or (pos_CC45_D_biundary > 0)) or \
               ((pos_C45_L_boundary > 0) or (pos_C45_U_boundary < 0) or (pos_C45_R_boundary < 0) or (pos_C45_D_boundary > 0)):
                return 1
            else:
                return 0         

    elif mode == 'good':
        M2V = np.array([V[0]-M[0], V[1]-M[1]])
        L_M2V = np.linalg.norm(M2V)
        Unit_M2V = M2V/L_M2V
        back_extend = [M[0] - (billiard_radius*5.5)*Unit_M2V[0], M[1] - (billiard_radius*5.5)*Unit_M2V[1]]
        CC45_Unit_M2V = TCCW45.dot(Unit_M2V)
        C45_Unit_M2V = TCW45.dot(Unit_M2V)
        CC45back_extend = [M[0] - (billiard_radius*5.5)*CC45_Unit_M2V[0], M[1] - (billiard_radius*5.5)*CC45_Unit_M2V[1]]
        C45back_extend = [M[0] - (billiard_radius*5.5)*C45_Unit_M2V[0], M[1] - (billiard_radius*5.5)*C45_Unit_M2V[1]]

        H1toback = [back_extend[0]-H1[0], back_extend[1]-H1[1]]
        H2toback = [back_extend[0]-H2[0], back_extend[1]-H2[1]]
        H4toback = [back_extend[0]-H4[0], back_extend[1]-H4[1]]
        H5toback = [back_extend[0]-H5[0], back_extend[1]-H5[1]]

        H1toCC45back_extend = [CC45back_extend[0]-H1[0], CC45back_extend[1]-H1[1]]
        H2toCC45back_extend = [CC45back_extend[0]-H2[0], CC45back_extend[1]-H2[1]]
        H3toCC45back_extend = [CC45back_extend[0]-H4[0], CC45back_extend[1]-H4[1]]
        H4toCC45back_extend = [CC45back_extend[0]-H5[0], CC45back_extend[1]-H5[1]]

        H1toC45back_extend = [C45back_extend[0]-H1[0], C45back_extend[1]-H1[1]]
        H2toC45back_extend = [C45back_extend[0]-H2[0], C45back_extend[1]-H2[1]]
        H3toC45back_extend = [C45back_extend[0]-H4[0], C45back_extend[1]-H4[1]]
        H4toC45back_extend = [C45back_extend[0]-H5[0], C45back_extend[1]-H5[1]]

        if H1[1] < H5[1]:
            pos_L_boundary = np.cross(L_boundary, H1toback)
            pos_U_boundary = np.cross(U_boundary, H2toback)
            pos_R_boundary = np.cross(H4toback, R_boundary)
            pos_D_boundary = np.cross(H5toback, D_boundary)

            pos_CC45_L_boundary = np.cross(L_boundary, H1toCC45back_extend)
            pos_CC45_U_boundary = np.cross(U_boundary, H2toCC45back_extend)
            pos_CC45_R_boundary = np.cross(H3toCC45back_extend, R_boundary)
            pos_CC45_D_biundary = np.cross(H4toCC45back_extend, D_boundary)

            pos_C45_L_boundary = np.cross(L_boundary, H1toC45back_extend)
            pos_C45_U_boundary = np.cross(U_boundary, H2toC45back_extend)
            pos_C45_R_boundary = np.cross(H3toC45back_extend, R_boundary)
            pos_C45_D_boundary = np.cross(H4toC45back_extend, D_boundary)

            if ((pos_L_boundary > 0) or (pos_U_boundary > 0) or (pos_R_boundary < 0) or (pos_D_boundary < 0)) or \
               ((pos_CC45_L_boundary > 0) or (pos_CC45_U_boundary > 0) or (pos_CC45_R_boundary < 0) or (pos_CC45_D_biundary < 0)) or \
               ((pos_C45_L_boundary > 0) or (pos_C45_U_boundary > 0) or (pos_C45_R_boundary < 0) or (pos_C45_D_boundary < 0)):
                return 1
            else:
                return 0      
        elif H1[1] > H5[1]:
            pos_L_boundary = np.cross(L_boundary, H1toback)
            pos_U_boundary = np.cross(H2toback, U_boundary)
            pos_R_boundary = np.cross(H4toback, R_boundary)
            pos_D_boundary = np.cross(D_boundary, H5toback) 

            pos_CC45_L_boundary = np.cross(L_boundary, H1toCC45back_extend)
            pos_CC45_U_boundary = np.cross(H2toCC45back_extend, U_boundary)
            pos_CC45_R_boundary = np.cross(H3toCC45back_extend, R_boundary)
            pos_CC45_D_biundary = np.cross(D_boundary, H4toCC45back_extend)

            pos_C45_L_boundary = np.cross(L_boundary, H1toC45back_extend)
            pos_C45_U_boundary = np.cross(H2toC45back_extend, U_boundary)
            pos_C45_R_boundary = np.cross(H3toC45back_extend, R_boundary)
            pos_C45_D_boundary = np.cross(D_boundary, H4toC45back_extend)

            if ((pos_L_boundary > 0) or (pos_U_boundary < 0) or (pos_R_boundary < 0) or (pos_D_boundary > 0)) or \
               ((pos_CC45_L_boundary > 0) or (pos_CC45_U_boundary < 0) or (pos_CC45_R_boundary < 0) or (pos_CC45_D_biundary > 0)) or \
               ((pos_C45_L_boundary > 0) or (pos_C45_U_boundary < 0) or (pos_C45_R_boundary < 0) or (pos_C45_D_boundary > 0)):
                return 1
            else:
                return 0         

def find_TargetHole():
    global H1,H2,H3,H4,H5,H6
    global gHole
    global min_dif, num_hole
    global M, C  

    global Hole_info
    Hole_info = {
        1: H1,    
        2: H2,    
        3: H3,    
        4: H4,    
        5: H5,
        6: H6 
        }

    check_hole = [] #local variable 存每個洞口 跟 子球 母球的 角度差值

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

def target_Strategy(quantity):
    global M
    global loading_key, no_cueball
    no_cueball = 0
    if loading_key == 1:
        if (M == [-999,-999]):
            print(M[0],M[1])
            print("No cueball on table")
            no_cueball += 1
        else:
            print(M[0],M[1])
            loading_key = 0
            if quantity > 7:
                print("Enter first stage")
                first_stage()
            elif quantity <= 7:
                print("Enter second stage")
                second_stage()
            no_cueball = 0              
            

def curve(my_data,mode):
    global all_ball, M
    global H1, H2, H3, H4, H5, H6
    global billiard_radius
    global pic_Point, now_C

    hole_x = [H1[0], H2[0], H3[0], H4[0], H5[0], H6[0]]
    hole_y = [H1[1], H2[1], H3[1], H4[1], H5[1], H6[1]]
 #=====================================================================
    c_x = 637.36542424
    c_y = 359.93598824
    f_x = 907.42716381
    f_y = 908.47256525

    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([1.82612813e-01, -5.57847355e-01, 4.63194714e-04 , -5.94172999e-05, 5.18000478e-01])
 #---------------------------------------------------------------------
    all_ball = []
    counter = 0
    dis_BallandHole = np.zeros(6) # 球和洞中心距離,用來判斷球是否在洞中
    for item in my_data:
        if (item[0] != 'M') & (float(item[1]) > 50): 
            exclusion_ball = 0
            # -----取中心座標-----
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]
            if mode == 'pic_point':
                center_x = round((img_pos[0]-(1280/2))*0.482, 3)
                center_y = round((img_pos[1]-(720/2))*0.483, 3)

                BASE_X = pic_Point[0] + (center_x*1.04) + Hit2camera[0]
                BASE_Y = pic_Point[1] - (center_y) + Hit2camera[1] 
            elif mode == 'second look':
                if (mx >= 590) and (mx <= 690) and (my >= 310) and (my <= 410):
                    center_x = round((img_pos[0]-(1280/2))*0.245, 3)
                    center_y = round((img_pos[1]-(720/2))*0.245, 3)

                    BASE_X = now_C[0] + (center_x) + Hit2camera[0]
                    BASE_Y = now_C[1] - (center_y) + Hit2camera[1]

                    new_C = [BASE_X, BASE_Y]

                    return new_C
                else:
                    continue
           
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
                 dis_BallandHole[j] = (((hole_x[j] - BASE_X)**2 + (hole_y[j] - BASE_Y)**2)**0.5)
                 if dis_BallandHole[j] <= 2*billiard_radius :
                    print('i see a ball in hole~~~~~~~~~~~~~~~~~~~~~')
                
                    exclusion_ball = 1
            result = np.where(dis_BallandHole == np.amin(dis_BallandHole))          #result代表dis_BaH的最小值的位置
            if exclusion_ball != 1:
                all_ball.append([str(item[0]), float(item[1]), BASE_X, BASE_Y, dis_BallandHole[result[0][0]]])
                counter += 1

    print(all_ball)
    print("============================================================================")
    #print(' s', counter,'t')
    return counter

def find_cueball(data, mode):
    global multi_M
    global bigger_middle_y
    global pic_Point
    global target, close_open_look

 #=====================================================================
    c_x = 637.36542424
    c_y = 359.93598824
    f_x = 907.42716381
    f_y = 908.47256525
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([1.82612813e-01, -5.57847355e-01, 4.63194714e-04 , -5.94172999e-05, 5.18000478e-01])
    

    multi_M = 0

    for item in data:
        if item[0] == 'M':
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]
            if mode == 'pic_point':
                center_x = round((img_pos[0]-(1280/2))*0.482, 3)
                center_y = round((img_pos[1]-(720/2))*0.483, 3)

                BASE_X = pic_Point[0] + (center_x*1.04) + Hit2camera[0]
                BASE_Y = pic_Point[1] - (center_y) + Hit2camera[1]
                M = [BASE_X , BASE_Y]
                return M

            elif mode == 'second look':
                if (mx > 590) and (mx < 690) and (my > 310) and (my < 410):
                    print("正確母球")
                    center_x = round((img_pos[0]-(1280/2))*0.245, 3)
                    center_y = round((img_pos[1]-(720/2))*0.245, 3)

                    BASE_X = target[0] + (center_x) + Hit2camera[0]
                    BASE_Y = target[1] - (center_y) + Hit2camera[1]
                    M = [BASE_X , BASE_Y]
                    return M
                else:
                    continue
            
            elif mode == 'close open':
                center_x = round((img_pos[0]-(1280/2))*0.245, 3)
                center_y = round((img_pos[1]-(720/2))*0.245, 3)

                BASE_X = close_open_look[0] + (center_x) + Hit2camera[0]
                BASE_Y = close_open_look[1] - (center_y) + Hit2camera[1]
                M = [BASE_X , BASE_Y]
                return M

def open(mode):
    global H1, H2, H3, H4, H5, H6
    global close_open_look

    hiwin_motor.motor_contorl(4)
    PTP_Move(pic_Point,50,20)
    ball_information = yolo_pic()
    M = find_cueball(ball_information, mode='pic_point')
    close_open_look = [M[0]-Hit2camera[0], M[1]-Hit2camera[1], 373.081, 180.000, 0.000, 90]
    PTP_Move(close_open_look,acl,speed)
    ball_information = yolo_pic()
    M = find_cueball(ball_information, mode='close open')

    if mode == 'LEFT':
        print("左開球")
        openL_dir = np.array([H5[0]-H1[0], H5[1]-H1[1]])
        L_openL_dir = np.linalg.norm(openL_dir)
        Unit_openL_dir = openL_dir/L_openL_dir
        front_vector = np.array([0, 1]).reshape(2, 1)
        T_front_vector = front_vector.transpose()
        n = np.dot(T_front_vector, openL_dir)
        target_angle = float(np.rad2deg(np.arccos(n/L_openL_dir)))
        hit_angle = 90-target_angle
        open_M = [M[0] - (66.76)*Unit_openL_dir[0], M[1] - (66.76)*Unit_openL_dir[1]]

        open_point_left = [open_M[0], open_M[1], 117.063, 180.000, 0.000, hit_angle]
        open_point_above = [open_M[0], open_M[1], 172.925, 180, 0, 90]
        PTP_Move(open_point_above,acl,speed)
        time.sleep(3)
        PTP_Move(open_point_left,acl,speed) 

    elif mode == 'RIGHT':
        print("右開球")
        openR_dir = np.array([H2[0]-H4[0], H2[1]-H4[1]])
        L_openR_dir = np.linalg.norm(openR_dir)
        Unit_openR_dir = openR_dir/L_openR_dir
        front_vector = np.array([0, 1]).reshape(2, 1)
        T_front_vector = front_vector.transpose()
        n = np.dot(T_front_vector, openR_dir)
        target_angle = float(np.rad2deg(np.arccos(n/L_openR_dir)))
        hit_angle = 90+target_angle
        open_M = [M[0] - (66.76)*Unit_openR_dir[0], M[1] - (66.76)*Unit_openR_dir[1]]

        open_point_right = [open_M[0], open_M[1], 117.063, 180.000, 0.000, hit_angle]
        open_point_above = [open_M[0], open_M[1], 172.925, 180, 0, 90]
        PTP_Move(open_point_above,acl,speed)
        time.sleep(3)
        PTP_Move(open_point_right,acl,speed)

    modbus.DO(IO_Port_2,1)
    time.sleep(0.2)
    modbus.DO(IO_Port_1,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_1,0)
    time.sleep(0.1)
    PTP_Move(open_point_above,acl,speed)
    modbus.DO(IO_Port_6, 0)

def yolo_pic():
    global M
    global loading_key

    loading_key =1

    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())
    ball_information = YOLO_Detect.detect_ALL(img,0.5)

    print(ball_information)

    return ball_information

def plot_table():
    global H1, H2, H3, H4, H5, H6, M, C, V, V1, V2
    global all_ball, gHole
    global kissball, luckyshoot
    global OB_back_flag, good_back_flag
    global extend_V, point_TCCW45, point_TCW45
    global fin_point_TCCW45, fin_point_TCW45
    global TCCW_M2V, TCW_M2V, TCW_back_ex2M, TCCW_back_ex2M
    global TCCW_M2V2, TCW_M2V2, TCCW_OBback_ex2M, TCW_OBback_ex2M
    global back_extend, CC45back_extend, C45back_extend
    global OBback_extend, OBCC45back_extend, OBC45back_extend

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

    if kissball == 0:
        H2V_x = [gHole[0], V[0]]
        H2V_y = [gHole[1], V[1]]
        TCCW45_x = [V[0], point_TCCW45[0]]
        TCCW45_y = [V[1], point_TCCW45[1]]
        TCW45_x = [V[0], point_TCW45[0]]
        TCW45_y = [V[1], point_TCW45[1]]
        angle_V2M_x = [V[0], M[0]]
        angle_V2M_y = [V[1], M[1]]
        good_back_x = [TCCW_M2V[0], TCW_M2V[0], TCW_back_ex2M[0], TCCW_back_ex2M[0], TCCW_M2V[0]]
        good_back_y = [TCCW_M2V[1], TCW_M2V[1], TCW_back_ex2M[1], TCCW_back_ex2M[1], TCCW_M2V[1]]
        back_extend_x = [M[0], back_extend[0]]
        back_extend_y = [M[1], back_extend[1]]
        CC45back_extend_x = [M[0], CC45back_extend[0]]
        CC45back_extend_y = [M[1], CC45back_extend[1]]
        C45back_extend_x = [M[0], C45back_extend[0]]
        C45back_extend_y = [M[1], C45back_extend[1]]
        
        if good_back_flag == 1:
            plt.plot(good_back_x, good_back_y)
        plt.plot(back_extend_x, back_extend_y, 'c--o')
        plt.plot(CC45back_extend_x, CC45back_extend_y, 'c--o')
        plt.plot(C45back_extend_x, C45back_extend_y, 'c--o')
        plt.plot(H2V_x,H2V_y)
        plt.plot(TCCW45_x,TCCW45_y, 'w')
        plt.plot(TCW45_x,TCW45_y, 'm')
        plt.plot(angle_V2M_x,angle_V2M_y, 'r')
        V_center = plt.Circle((V[0], V[1]), 12.45, color = 'y')
        axes.set_aspect(1)
        axes.add_artist(V_center)
        
    elif kissball == 1:
        H2V1_x = [gHole[0], V1[0]]
        H2V1_y = [gHole[1], V1[1]]
        V12V2_x = [V1[0], V2[0]]
        V12V2_y = [V1[1], V2[1]]
        fin_TCCW45_x = [V2[0], fin_point_TCCW45[0]]
        fin_TCCE45_y = [V2[1], fin_point_TCCW45[1]]
        fin_TCW45_x = [V2[0], fin_point_TCW45[0]]
        fin_TCW45_y = [V2[1], fin_point_TCW45[1]]
        angle_V22M_x = [V2[0], M[0]]
        angle_V22M_y = [V2[1], M[1]]
        OB_back_x = [TCCW_M2V2[0], TCW_M2V2[0], TCW_OBback_ex2M[0], TCCW_OBback_ex2M[0], TCCW_M2V2[0]]
        OB_back_y = [TCCW_M2V2[1], TCW_M2V2[1], TCW_OBback_ex2M[1], TCCW_OBback_ex2M[1], TCCW_M2V2[1]]
        OBback_extend_X = [M[0], OBback_extend[0]]
        OBback_extend_Y = [M[1], OBback_extend[1]]
        OBCC45back_extend_x = [M[0], OBCC45back_extend[0]]
        OBCC45back_extend_y = [M[1], OBCC45back_extend[1]]
        OBC45back_extend_x = [M[0], OBC45back_extend[0]]
        OBC45back_extend_y = [M[1], OBC45back_extend[1]]

        
        if OB_back_flag == 1:
            plt.plot(OB_back_x, OB_back_y)
        plt.plot(OBback_extend_X, OBback_extend_Y, 'c--o')
        plt.plot(OBCC45back_extend_x, OBCC45back_extend_y, 'c--o')
        plt.plot(OBC45back_extend_x, OBC45back_extend_y, 'c--o')
        plt.plot(H2V1_x,H2V1_y)
        plt.plot(V12V2_x,V12V2_y)
        plt.plot(fin_TCCW45_x,fin_TCCE45_y, 'w')
        plt.plot(fin_TCW45_x,fin_TCW45_y, 'm')
        plt.plot(angle_V22M_x,angle_V22M_y, 'r')
        V1_center = plt.Circle((V1[0], V1[1]), 12.45, color = 'm')
        axes.set_aspect(1)
        axes.add_artist(V1_center)
        V2_center = plt.Circle((V2[0], V2[1]), 12.45, color = 'y')
        axes.set_aspect(1)
        axes.add_artist(V2_center)
    elif luckyshoot == 1:
        pass

    for j in range(len(ball_sorted)):            
        ball_center = plt.Circle((ball_x[j], ball_y[j]), 12.45)
        axes.set_aspect(1)
        axes.add_artist(ball_center) 
    print("============================================================================")

    plt.title("Table")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.xlim(-400,400)
    plt.ylim(0,850)
    # plt.show()
    plt.ion()
    plt.pause(1.5)
    plt.close()
 
 

'''
    Mission_trigger():
        執行步驟：
            step0 : 當DI1(開始按鈕)被按下時移至起始位置
            step1 : if要開球,則將手臂移至開球位置並以最大力擊球
                    if為後攻和開球結束之後,則移至拍照位置並計算球洞位置和拍照計算目標球
                        find_holes():透過靠近手臂最左下及右下桌角所形成向量計算球洞中心
                        yolo_pic():先進行拍照,後將照片套用yolo模組得出球影像座標,最後將座標轉換成手臂座標


'''

def Mission_trigger():
    global is_busy, kissball
    global M, now_C, C
    global shoot, open_ball, open_L, open_R
    global no_cueball
    global pic_x_float, pic_y_float, pic_z_float, pic_yaw_float
    global pic_Point
    global OB_back_flag, good_back_flag
    global target

    step = 0        # 執行步驟
    OB_back_flag = 0
    good_back_flag = 0
    OB_boundary_flag = 0
    good_boundary_flag = 0
    M_back_flag = 0
    boundary_flag = 0

    pic_Point = [pic_x_float, pic_y_float, 530.359, 180.000, 0, pic_yaw_float]

    find_holes()

    while 1:
        PTP_Move(start_point,50,20)
        if is_busy == True:
            is_busy = False
        if is_busy == False:
            if modbus.DI(IO_Port_in,0) == 1:
                is_busy = True
                OB_back_flag = 0
                good_back_flag = 0
                OB_back_flag = 0
                good_back_flag = 0
                OB_boundary_flag = 0
                good_boundary_flag = 0
                M_back_flag = 0
                boundary_flag = 0
                modbus.DO(IO_Port_6, 1)
                hiwin_motor.motor_contorl(2)
                step = 1

            if step == 1:
                if open_ball == 1:
                    modbus.DO(IO_Port_2,0)
                    is_busy = True
                    if open_R == 1:
                        open(mode='RIGHT')
                        open_ball = 0             
                        step = 0
                        open_R = 0

                    elif open_L == 1:
                        open(mode='LEFT')
                        open_ball = 0             
                        step = 0
                        open_L = 0
                    
                elif open_ball == 0:
                    is_busy = True
                    PTP_Move(pic_Point,50,20)
                    find_holes()
                    ball_information = yolo_pic()
                    M = find_cueball(ball_information, mode='pic_point')
                    ball_quantity = curve(ball_information, mode='pic_point')
                    print("M:{},{}".format(M[0],M[1]))
                    print("============================================================================")
                    time.sleep(0.1)
                    modbus.DO(IO_Port_2, 0)
                    
                    step = 2

            if step == 2:
                target_Strategy(ball_quantity)
                step = 3
                
            if step == 3:
                FinalAngle()
                step = 4

            if step == 4:
                target = [M[0]-Hit2camera[0], M[1]-Hit2camera[1], 373.081, 180.000, 0.000, 90]
                PTP_Move(target,acl,speed)
                time.sleep(0.1)
                ball_information = yolo_pic()
                M = find_cueball(ball_information, mode='second look')
                print("second look M:{},{}".format(M[0],M[1]))
                print("kissball = {}, luckyshoot = {}".format(kissball, luckyshoot))
                if (kissball == 0) and (luckyshoot == 0):
                    print("子球看兩次")
                    now_C = [C[0]-Hit2camera[0], C[1]-Hit2camera[1], 373.081, 180.000, 0.000, 90]
                    PTP_Move(now_C,acl,speed)
                    time.sleep(0.1)
                    ball_information = yolo_pic()
                    C = curve(ball_information, mode='second look')
                    FinalAngle()
                    PTP_Move(target,acl,speed)
                if luckyball == 1:
                    print("再次計算luckyball !!")
                    luckyball()
                print("============================================================================")
                step = 5

            if step == 5:
                if kissball == 1:
                    M_counter = OB_M_back()
                    boundary_check = boundary_back(mode='OB')
                    if boundary_check == 1:
                        print("干擾球，超邊出界")
                        hiwin_motor.motor_contorl(3)
                        boundary_flag = 1
                        OB_boundary_flag = 1
                        step = 6
                    elif boundary_check == 0:
                        boundary_flag = 0
                        OB_boundary_flag = 0
                        if M_counter == 1:
                            print("干擾球，母球後有阻擋")
                            hiwin_motor.motor_contorl(3)
                            M_back_flag = 1
                            OB_back_flag = 1
                            step = 6
                        elif M_counter == 0:
                            print("all good !!")
                            M_back_flag = 0
                            OB_back_flag = 0
                            step = 6

                elif kissball == 0:
                    M_counter = good_M_back()
                    boundary_check = boundary_back(mode='good')
                    if boundary_check == 1:
                        print("好球，超出邊界")
                        hiwin_motor.motor_contorl(3)
                        boundary_flag = 1
                        good_boundary_flag = 1
                        step = 6
                    elif boundary_check == 0:
                        boundary_flag = 0
                        good_boundary_flag = 0
                        if M_counter == 1:
                            print("好球，母球後有阻擋")
                            hiwin_motor.motor_contorl(3)
                            M_back_flag = 1
                            good_back_flag = 1
                            step = 6
                        elif M_counter == 0:
                            print("all good !!")
                            M_back_flag = 0
                            good_back_flag = 0
                            step = 6
                
            if step == 6:
                plot_table()
                is_busy = True
                hit_angle = 90-target_angle
                print(hit_angle)
                print("============================================================================")
                print("boundary_flag = {}, OB_boundary_flag = {}, good_boundary_flag = {}".format(boundary_flag, OB_boundary_flag, good_boundary_flag))
                print("M_back_flag = {}, OB_back_flag = {}, good_back_flag = {}".format(M_back_flag, OB_back_flag, good_back_flag))
                if boundary_flag == 1:
                    if good_boundary_flag == 1:
                        print("好球調整")
                        M2V = np.array([V[0]-M[0], V[1]-M[1]])
                        L_M2V = np.linalg.norm(M2V)
                        Unit_M2V = M2V/L_M2V
                        new_M = [M[0] - (50.76)*Unit_M2V[0], M[1] - (50.76)*Unit_M2V[1]]
                        target = [new_M[0], new_M[1], 159.595, 180.000, 0.000, hit_angle]       # z = 169.651
                    elif OB_boundary_flag == 1:
                        print("干擾調整")
                        M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
                        L_MV2 = np.linalg.norm(M2V2)
                        Unit_M2V2 = M2V2/L_MV2
                        new_M = [M[0] - (50.76)*Unit_M2V2[0], M[1] - (50.76)*Unit_M2V2[1]]
                        target = [new_M[0], new_M[1], 159.595, 180.000, 0.000, hit_angle]

                elif M_back_flag == 1:
                    if good_back_flag == 1:
                        print("好球調整")
                        M2V = np.array([V[0]-M[0], V[1]-M[1]])
                        L_M2V = np.linalg.norm(M2V)
                        Unit_M2V = M2V/L_M2V
                        new_M = [M[0] - (51.76)*Unit_M2V[0], M[1] - (51.76)*Unit_M2V[1]]
                        target = [new_M[0], new_M[1], 159.595, 180.000, 0.000, hit_angle]     # z = 169.651
                    elif OB_back_flag == 1:
                        print("干擾調整")
                        M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
                        L_MV2 = np.linalg.norm(M2V2)
                        Unit_M2V2 = M2V2/L_MV2
                        new_M = [M[0] - (51.76)*Unit_M2V2[0], M[1] - (51.76)*Unit_M2V2[1]]
                        target = [new_M[0], new_M[1], 159.595, 180.000, 0.000, hit_angle]

                elif (boundary_flag == 0) and (M_back_flag == 0):
                    if kissball == 0:
                        print("好球，無須調整")
                        M2V = np.array([V[0]-M[0], V[1]-M[1]])
                        L_M2V = np.linalg.norm(M2V)
                        Unit_M2V = M2V/L_M2V
                        new_M = [M[0] - (66.76)*Unit_M2V[0], M[1] - (66.76)*Unit_M2V[1]]                        
                        target = [new_M[0], new_M[1], 134.331, 180.000, 0.000, hit_angle]     # z = 134.331
                    elif kissball == 1:
                        M2V2 = np.array([V2[0]-M[0], V2[1]-M[1]])
                        L_MV2 = np.linalg.norm(M2V2)
                        Unit_M2V2 = M2V2/L_MV2
                        new_M = [M[0] - (66.76)*Unit_M2V2[0], M[1] - (66.76)*Unit_M2V2[1]]
                        target = [new_M[0], new_M[1], 134.331, 180.000, 0.000, hit_angle]


                step = 7

            if step == 7:
                is_busy = True
                modbus.DO(IO_Port_2,1)
                if (hit_angle <= 0) and (hit_angle >= -90):
                    yaw_zero = [M[0], M[1], 198.925, 180.000, 0.000, 0]
                    PTP_Move(yaw_zero,acl,speed)
                    time.sleep(0.5)
                    PTP_Move(target,acl,speed)
                elif (target_angle < -90) and (target_angle >= -180):
                    yaw_180 = [M[0], M[1], 198.925, 180.000, 0.000, 180]
                    PTP_Move(yaw_180,acl,speed)
                    time.sleep(0.5)
                    PTP_Move(target,acl,speed)
                else:
                    PTP_Move(target,acl,speed)

                # time.sleep(10)
                
                print("目前力道 : ",shoot)
                if shoot == 300:
                    modbus.DO(IO_Port_1,1)
                    time.sleep(0.1)
                    modbus.DO(IO_Port_1,0)
                if shoot == 255:
                    modbus.DO(IO_Port_5,1)
                    time.sleep(0.1)
                    modbus.DO(IO_Port_5,0)
                if shoot == 150:
                    modbus.DO(IO_Port_4,1)
                    time.sleep(0.1)
                    modbus.DO(IO_Port_4,0)
                step = 8
            
            if step == 8:
                is_busy = True
                if (hit_angle <= 0) and (hit_angle >= -90):
                    yaw_zero = [M[0], M[1], 198.925, 180.000, 0.000, 0]
                    PTP_Move(yaw_zero,acl,speed)
                    time.sleep(0.5)
                    PTP_Move(start_point,acl,speed)
                elif (target_angle < -90) and (target_angle >= -180):
                    yaw_180 = [M[0], M[1], 198.925, 180.000, 0.000, 180]
                    PTP_Move(yaw_180,acl,speed)
                    time.sleep(0.5)
                    go_home_180 = [M[0], M[1], 198.925, 180.000, 0.000, 90]
                    PTP_Move(go_home_180,acl,speed)
                    time.sleep(0.5)
                    PTP_Move(start_point,acl,speed)
                else:
                    go_home_zero = [M[0], M[1], 198.925, 180.000, 0.000, 90]
                    PTP_Move(go_home_zero,acl,speed)
                    time.sleep(0.5)
                    PTP_Move(start_point,acl,speed) 
                modbus.DO(IO_Port_6, 0)
                hiwin_motor.motor_contorl(2)
                step = 0

def test_pic():
    global M
    global loading_key

    loading_key =1

    frames = pipeline.wait_for_frames()
    img = frames.get_color_frame()
    img = np.asanyarray(img.get_data())
    ball_information = YOLO_Detect.detect_ALL(img,0.5)

    print(ball_information)
    M = find_cueball(ball_information)
    curve(ball_information)  

def pic_check():                 # 確認拍照位置
    PTP_Move(pic_Point,50,20)
    test_pic() 
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def look():                      # 開啟相機
    global start_end
    if start_end:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Convert images to numpy arrays 把图像转换为numpy data
        color_image = np.asanyarray(color_frame.get_data())
        # YOLO_Detect.detect_ALL(color_image,0.5)

        cv2.namedWindow('MyD415')
        cv2.imshow('MyD415', color_image)
        key = cv2.waitKey(1)

    main_menu.after(10, look)

def left_corner_check():           # 左下角落
    print('\033[''1;'+str(41)+'m left_corner_point height = \033[m',left_corner_point[2])
    time.sleep(2)
    PTP_Move(left_corner_point,50,20)
            
def right_corner_check():          # 右下角落
    print('\033[''1;'+str(41)+'m right_corner_point height = \033[m',right_corner_point[2])
    time.sleep(2)
    PTP_Move(right_corner_point,50,20)

def hit_check():                # 電磁炮
    modbus.DO(IO_Port_2, 0)
    PTP_Move(testing_point,50,20)
    hiwin_motor.motor_contorl(4)
    time.sleep(0.5)
    hiwin_motor.motor_contorl(3)
    time.sleep(0.5)
    hiwin_motor.motor_contorl(2)
    time.sleep(0.5)
    modbus.DO(IO_Port_2, 1)

    modbus.DO(IO_Port_5,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_5,0)
    time.sleep(1)
    modbus.DO(IO_Port_4,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_4,0)
    time.sleep(1)
    modbus.DO(IO_Port_1,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_1,0)


def n_open(event):
    global open_ball

    open_ball = 0
    print(open_ball)
    open_yes = tk.Label(main_menu, text="不開球", font=('Arial',30))
    open_yes.place(x=700, y=550)

def r_open(event):
    global open_ball, open_R
    open_ball = 1

    if open_ball == 1:
        open_R = 1
        open_L = 0
        print("open_R = {}, open_L = {}".format(open_R, open_L))
        open_right = tk.Label(main_menu, text="要右開球", font=('Arial',25))
        open_right.place(x=700, y=650)
        print(open_R)
    else:
        open_right = tk.Label(main_menu, text="不開球", font=('Arial',25))
        open_right.place(x=400, y=630)

def l_open(event):
    global open_ball, open_L
    open_ball = 1

    if open_ball == 1:
        open_L = 1
        open_R = 0
        print("open_R = {}, open_L = {}".format(open_R, open_L))
        open_left = tk.Label(main_menu, text="要左開球", font=('Arial',25))
        open_left.place(x=700, y=650)
        print(open_L)
    else:
        open_left = tk.Label(main_menu, text="不開球", font=('Arial',25))
        open_left.place(x=400, y=630)

def tk_input(event):
    global pic_x_float, pic_y_float, pic_z_float, pic_yaw_float
    global left_x_float, left_y_float
    global right_x_float, right_y_float

    if pic_x_input.get() != '':
        pic_x_str = pic_x_input.get()
        pic_x_float = float(pic_x_str)
        print("pic_x_float:",pic_x_float)

    if pic_y_input.get() != '':
        pic_y_str = pic_y_input.get()
        pic_y_float = float(pic_y_str)
        print("pic_y_float:",pic_y_float)    

    if pic_yaw_input.get() != '':
        pic_yaw_str = pic_yaw_input.get()
        pic_yaw_float = float(pic_yaw_str)
        print("pic_yaw_float:",pic_yaw_float)

    if left_x_input.get() != '':
        left_x_str = left_x_input.get()
        left_x_float = float(left_x_str)
        print("left_x_float:",left_x_float)

    if left_y_input.get() != '':
        left_y_str = left_y_input.get()
        left_y_float = float(left_y_str)
        print("left_y_float:",left_y_float)

    if right_x_input.get() != '':
        right_x_str = right_x_input.get()
        right_x_float = float(right_x_str)       
        print("right_x_float:",right_x_float)

    if right_y_input.get() != '':
        right_y_str = right_y_input.get()
        right_y_float = float(right_y_str)
        print("right_y_float:",right_y_float)

    print("==============================")

def shutdownbutton(event):
    os._exit(os.EX_OK) 

def test_start(event):
    global testing

    testing = 1
    print("test start")

def test_end(event):
    global testing

    testing = 0
    print("test end")

def StartLook(event):
    global start_end

    start_end = True

def EndLook(event):
    global start_end

    start_end = False
    cv2.destroyAllWindows()

def Process():
    global testing

    find_holes()
    
    process_menu = tk.Toplevel(main_menu)
    process_menu.title('測試')
    process_menu.geometry('1280x720')
    process_menu.configure(background = 'grey')

    pic = tk.Button(process_menu, text="確認拍照位置", bg="green", font=('Arial',20), width=12, height=7, command=pic_check)
    pic.place(x=0,y=0)
    look_table = tk.Button(process_menu, text="take_picture", bg="green", font=('Arial',20), width=12, height=7, command=look)
    look_table.place(x=0,y=300)
    look_table.bind("<Button>",StartLook)
    start_look = tk.Button(process_menu, text="stop_pic", bg="green", font=('Arial',20), width=12, height=4)
    start_look.place(x=0,y=550)
    start_look.bind("<Button>", EndLook)

    left_corner = tk.Button(process_menu, text="左下角落", bg="yellow", font=('Arial',20), width=12, height=7, command=left_corner_check)
    left_corner.place(x=300,y=0)
    right_corner = tk.Button(process_menu, text="右下角落", bg="yellow", font=('Arial',20), width=12, height=7, command=right_corner_check)
    right_corner.place(x=300,y=300)

    hit = tk.Button(process_menu, text="打擊測試", bg="pink", font=('Arial',20), width=12, height=7, command=hit_check)
    hit.place(x=600,y=0) 

    test_shutdown = tk.Button(process_menu, text='關閉程式', bg='red', font=('Arial',30), width=15, height=8, command=process_menu.destroy)
    test_shutdown.place(x=900, y=300)
    test_shutdown.bind("<Button>",test_end)

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
    Arm_state = 0
    IO_Port_in = 300  #開始按鈕
    IO_Port_1 = 300 # D1 最大力
    IO_Port_2 = 301 # D2 充電開關
    IO_Port_4 = 303 # D4 未降壓
    IO_Port_5 = 304 # D5 降壓
    IO_Port_6 = 305 # D6 電燈
    
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
    modbus.DO(IO_Port_2, 1)
    PTP_Move(start_point,50,20)
    PTP_Move(start_point,50,20)

    main_menu = tk.Tk()
    main_menu.title('主控')
    main_menu.geometry('1280x720')
    main_menu.configure(background = 'grey')

    pic_x_label = tk.Label(main_menu, text="拍照位置x:", font=('Arial',30))
    pic_x_label.grid(row=0, column=0)
    pic_x_input = tk.Entry(main_menu, bd=10)
    pic_x_input.grid(row=0, column=1)
    pic_x_input.bind("<Return>",tk_input)

    pic_y_label = tk.Label(main_menu, text="拍照位置y:", font=('Arial',30))
    pic_y_label.grid(row=1, column=0)
    pic_y_input = tk.Entry(main_menu, bd=10)
    pic_y_input.grid(row=1, column=1)
    pic_y_input.bind("<Return>",tk_input)

    pic_yaw_label = tk.Label(main_menu, text="拍照位置yaw:", font=('Arial',30))
    pic_yaw_label.grid(row=2, column=0)
    pic_yaw_input = tk.Entry(main_menu, bd=10)
    pic_yaw_input.grid(row=2, column=1)
    pic_yaw_input.bind("<Return>",tk_input)

    left_x_label = tk.Label(main_menu, text="左下角落x:", font=('Arial',30))
    left_x_label.grid(row=3, column=0)
    left_x_input = tk.Entry(main_menu, bd=10)
    left_x_input.grid(row=3, column=1)
    left_x_input.bind("<Return>",tk_input)

    left_y_label = tk.Label(main_menu, text="左下角落y:", font=('Arial',30))
    left_y_label.grid(row=4, column=0)
    left_y_input = tk.Entry(main_menu, bd=10)
    left_y_input.grid(row=4, column=1)
    left_y_input.bind("<Return>",tk_input)
     
    right_x_label = tk.Label(main_menu, text="右下角落x:", font=('Arial',30))
    right_x_label.grid(row=5, column=0)
    right_x_input = tk.Entry(main_menu, bd=10)
    right_x_input.grid(row=5, column=1)
    right_x_input.bind("<Return>",tk_input)

    right_y_label = tk.Label(main_menu, text="右下角落y:", font=('Arial',30))
    right_y_label.grid(row=6, column=0)
    right_y_input = tk.Entry(main_menu, bd=10)
    right_y_input.grid(row=6, column=1)
    right_y_input.bind("<Return>",tk_input)

    open_ball_R = tk.Button(main_menu, text="右開球", font=('Arial',15), width=5, height=2)
    open_ball_R.place(x=600, y=630)
    open_ball_R.bind("<Button>", r_open)

    open_ball_L = tk.Button(main_menu, text="左開球", font=('Arial',15), width=5, height=2)
    open_ball_L.place(x=450, y=630)
    open_ball_L.bind("<Button>", l_open)

    no_open = tk.Button(main_menu, text="不開球", font=('Arial',15), width=5, height=2)
    no_open.place(x=500, y=550)
    no_open.bind("<Button>", n_open)

    main = tk.Button(main_menu, text="主程式", bg="yellow", font=('Arial',15), width=27, height=10, command=Mission_trigger)
    main.place(x=450,y=270)

    process_test = tk.Button(main_menu, text="測試程式", bg="green", font=('Arial',15), width=27, height=10, command=Process)
    process_test.place(x=450,y=0)
    process_test.bind("<Button>", test_start)

    main_shutdown = tk.Button(main_menu, text='關閉程式', bg='red', font=('Arial',20), width=30, height=20)
    main_shutdown.place(x=800, y=0)
    main_shutdown.bind("<Button-1>",shutdownbutton)

    main_menu.mainloop()

    modbus.Modbus_Close()
    print("Modbus Close")  
