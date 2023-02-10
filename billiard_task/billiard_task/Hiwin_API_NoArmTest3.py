from calendar import c
from operator import mod
from re import I
from tracemalloc import start
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

is_busy = True
open_ball = 0
shoot = 255
billiard_radius = 12.45

start_point = [-367.708, 12.711 ,293.500, 180.000, 0.000, 90.605]
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
        V = [ (C[0] - 2*(billiard_radius)*Unit_C2H[0]), (C[1] - 2*(billiard_radius) * Unit_C2H[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

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


def OB_V():
    global M, C
    global gHole
    global target_angle
    global back_OB_storted
    global V1, V2

    #------------從洞口(gHole)推算子球(C)的虛擬球1(V1)-------------------
    print("C[2]::::",C[2])
    find_TargetHole()
    good_angle(mode = 'OB_plot')
    C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])                  #從C指向H的向量                       
    L_C2H = np.linalg.norm(C2H)                                     #CH向量的長度
    Unit_C2H = C2H / L_C2H                                              #CH向量的單位向量  
    V1 = [ (C[0]-2 * (billiard_radius) * Unit_C2H[0]), (C[1]-2 * (billiard_radius) * Unit_C2H[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 

    #------------從虛擬球1(V1)推算干擾球(OB_1)的虛擬球2(V2)-------------------
    OB2V1 = np.array([V1[0]-C[2][0], V1[1]-C[2][1]])
    L_OB2V1 = np.linalg.norm(OB2V1)
    Unit_OB2V1 = OB2V1/L_OB2V1
    V2 = [(C[2][0] - 2*(billiard_radius) * Unit_OB2V1[0]), (C[2][1]-2 * (billiard_radius) * Unit_OB2V1[1])]

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




def early_stage():
    global M, C
    global all_ball
    global billiard_radius, hit_power_check
    global back_OB_storted, gHole
    global ball_storted
    global kissball,shoot,luckyshoot

    luckyshoot = 0
    kissball = 0

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
                else:
                    print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))
            elif (front_OB == 0) and (back_OB == 1):
                angle = good_angle(mode = 'OB')
                OB_target.append([items[0], C[0], C[1], back_OB_storted[0], dis_MC])
                print("目前子球 C :[{:4.0f},{:4.0f}] , kissball !!".format(C[0], C[1]))
            else:
                print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))

    print("//////////////////////////////////////////////////////////////////////")
    
    if len(good_target) >= 1:
        good_target.sort(key = lambda x: x[3])
        good_target.sort(key = lambda x: x[4])
        print("Successfully set a good target C : [{}, {:4.0f}, {:4.0f}]".format(good_target[0][0], good_target[0][1], good_target[0][2]))
        C = [good_target[0][1], good_target[0][2]]
        if ((good_target[0][3] >= hit_power_check) and (good_target[0][4] >= hit_power_check)) or \
           ((good_target[0][3] >= hit_power_check) and (good_target[0][4] < hit_power_check)) or \
           ((good_target[0][3] < hit_power_check) and (good_target[0][4] >= hit_power_check)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (dis_MC < hit_power_check) and (dis_CH < hit_power_check):
            shoot = 150
            kissball = 0
            print("minimum power !!")

    elif (len(good_target) < 1) and (len(BigAngle_Good_target) >= 1):
        BigAngle_Good_target.sort(key = lambda x: x[3])
        BigAngle_Good_target.sort(key = lambda x: x[4])
        print("Successfully set a good but big angle target C : [{}, {:4.0f}, {:4.0f}]".format(BigAngle_Good_target[0][0], BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]))
        C = [BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]]
        shoot = 255
        kissball = 0
        print("maximum power !!")
    elif (len(good_target) < 1) and (len(OB_target) >= 1):
        OB_target.sort(reverse=True, key = lambda x: x[4])
        print("Successfully set a kissball target C : [{}, {:4.0f}, {:4.0f}]".format(OB_target[0][0], OB_target[0][1], OB_target[0][2]))
        C = [OB_target[0][1], OB_target[0][2], OB_target[0][3]]
        print("back_OB_storted : ",C[2])
        shoot = 255
        kissball = 1
        print("maximum power !!")
    else:
        luckyball()
        shoot = 255
        luckyshoot = 1
        print("luckyball !!")
    
    print("============================================================================")

def middle_stage():
    global M, C
    global all_ball
    global billiard_radius, hit_power_check
    global back_OB_storted, gHole
    global ball_storted
    global kissball,shoot,luckyshoot

    luckyshoot = 0
    kissball = 0

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
                else:
                    print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))
            elif (front_OB == 0) and (back_OB == 1):
                angle = good_angle(mode = 'OB')
                OB_target.append([items[0], C[0], C[1], back_OB_storted[0], dis_MC])
                print("目前子球 C :[{:4.0f},{:4.0f}] , kissball !!".format(C[0], C[1]))
            else:
                print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))

    print("//////////////////////////////////////////////////////////////////////")

    if len(good_target) >= 1:
        good_target.sort(key = lambda x: x[3])
        good_target.sort(key = lambda x: x[4])
        print("Successfully set a good target C : [{}, {:4.0f}, {:4.0f}]".format(good_target[0][0], good_target[0][1], good_target[0][2]))
        C = [good_target[0][1], good_target[0][2]]
        if ((good_target[0][3] >= hit_power_check) and (good_target[0][4] >= hit_power_check)) or \
           ((good_target[0][3] >= hit_power_check) and (good_target[0][4] < hit_power_check)) or \
           ((good_target[0][3] < hit_power_check) and (good_target[0][4] >= hit_power_check)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (dis_MC < hit_power_check) and (dis_CH < hit_power_check):
            shoot = 150
            kissball = 0
            print("minimum power !!")

    elif (len(good_target) < 1) and (len(BigAngle_Good_target) >= 1):
        BigAngle_Good_target.sort(key = lambda x: x[3])
        BigAngle_Good_target.sort(key = lambda x: x[4])
        print("Successfully set a good but big angle target C : [{}, {:4.0f}, {:4.0f}]".format(BigAngle_Good_target[0][0], BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]))
        C = [BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]]
        shoot = 255
        kissball = 0
        print("maximum power !!")

    elif (len(good_target) < 1) and (len(OB_target) >= 1):
        OB_target.sort(reverse=True, key = lambda x: x[4])
        print("Successfully set a kissball target C : [{}, {:4.0f}, {:4.0f}]".format(OB_target[0][0], OB_target[0][1], OB_target[0][2]))
        C = [OB_target[0][1], OB_target[0][2], OB_target[0][3]]
        print("back_OB_storted : ",C[2])
        shoot = 255
        kissball = 1
        print("maximum power !!")
    else:
        luckyball()
        shoot = 255
        luckyshoot = 1
        print("luckyball !!")
    
    print("============================================================================")

def final_stage():
    global M, C
    global all_ball
    global billiard_radius, hit_power_check
    global back_OB_storted, gHole
    global ball_storted
    global kissball,shoot,luckyshoot

    luckyshoot = 0
    kissball = 0

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
                else:
                    print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))
            elif (front_OB == 0) and (back_OB == 1):
                angle = good_angle(mode = 'OB')
                OB_target.append([items[0], C[0], C[1], back_OB_storted[0], dis_MC])
                print("目前子球 C :[{:4.0f},{:4.0f}] , kissball !!, OB : {}".format(C[0], C[1], back_OB_storted[0]))
            else:
                print("目前子球 C :[{:4.0f},{:4.0f}] , bad target !!".format(C[0], C[1]))

    print("//////////////////////////////////////////////////////////////////////")

    if len(good_target) >= 1:
        good_target.sort(key = lambda x: x[3])
        good_target.sort(key = lambda x: x[4])
        print("Successfully set a good target C : [{}, {:4.0f}, {:4.0f}]".format(good_target[0][0], good_target[0][1], good_target[0][2]))
        C = [good_target[0][1], good_target[0][2]]
        if ((good_target[0][3] >= hit_power_check) and (good_target[0][4] >= hit_power_check)) or \
           ((good_target[0][3] >= hit_power_check) and (good_target[0][4] < hit_power_check)) or \
           ((good_target[0][3] < hit_power_check) and (good_target[0][4] >= hit_power_check)):
            shoot = 255
            kissball = 0
            print("maximum power !!")
        elif (good_target[0][3] < hit_power_check) and (good_target[0][4] < hit_power_check):
            shoot = 150
            kissball = 0
            print("minimum power !!")

    elif (len(good_target) < 1) and (len(BigAngle_Good_target) >= 1):
        BigAngle_Good_target.sort(key = lambda x: x[3])
        BigAngle_Good_target.sort(key = lambda x: x[4])
        print("Successfully set a good but big angle target C : [{}, {:4.0f}, {:4.0f}]".format(BigAngle_Good_target[0][0], BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]))
        C = [BigAngle_Good_target[0][1], BigAngle_Good_target[0][2]]
        shoot = 255
        kissball = 0
        print("maximum power !!")

    elif (len(good_target) < 1) and (len(OB_target) >= 1):
        OB_target.sort(reverse=True, key = lambda x: x[4])
        print("Successfully set a kissball target C : [{}, {:4.0f}, {:4.0f}]".format(OB_target[0][0], OB_target[0][1], OB_target[0][2]))
        C = [OB_target[0][1], OB_target[0][2], OB_target[0][3]]
        print("back_OB_storted : ",C[2])
        shoot = 255
        kissball = 1
        print("maximum power !!")

    else:
        luckyball()
        shoot = 255
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
    global TCCW_C2H, TCW_C2H, TCCW_H2C, TCW_H2C

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
    global TCCW_M2V,TCW_M2V,TCCW_V2M,TCW_V2M

    print("===back===")
    back_OB_storted = [[]for i in range(3)]           #干擾球宣告用
    counter = 0
    dis_limit = 2.3*billiard_radius

    TCCW = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])
    TCW = np.array([[0, 1],      #+時針旋轉的旋轉矩陣
                   [-1,  0]])

    C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])        #從C指向H的向量                       
    L_C2H = np.linalg.norm(C2H)                          #CH向量的長度
    Unit_C2H = C2H / L_C2H                               #CH向量的單位向量  
    V = [ (C[0] - (2*(billiard_radius)) * Unit_C2H[0]), (C[1] - (2*(billiard_radius)) * Unit_C2H[1]) ] 

    M2V = np.array([V[0]-M[0], V[1]-M[1]])
    L_M2V = np.linalg.norm(M2V)
    Unit_M2V = M2V/L_M2V
    TCCW_Unit_M2V = TCCW.dot(Unit_M2V)
    TCW_Unit_M2V = TCW.dot(Unit_M2V)   
    TCCW_lim_M2V = dis_limit*TCCW_Unit_M2V
    TCW_lim_M2V = dis_limit*TCW_Unit_M2V

    TCCW_M2V = [TCCW_lim_M2V[0] + M[0],TCCW_lim_M2V[1] + M[1]]
    TCW_M2V = [TCW_lim_M2V[0] + M[0],TCW_lim_M2V[1] + M[1]]

    V2M = np.array([M[0]-V[0], M[1]-V[1]])
    L_V2M = np.linalg.norm(V2M)
    Unit_V2M = V2M/L_V2M
    TCCW_Unit_V2M = TCCW.dot(Unit_V2M)
    TCW_Unit_V2M = TCW.dot(Unit_V2M)
    TCCW_lim_V2M = dis_limit*TCCW_Unit_V2M
    TCW_lim_V2M = dis_limit*TCW_Unit_V2M

    TCCW_V2M = [TCCW_lim_V2M[0] + V[0],TCCW_lim_V2M[1] + V[1]]
    TCW_V2M = [TCW_lim_V2M[0] + V[0],TCW_lim_V2M[1] + V[1]]

    TCCW_M2VtoTCW_M2V = [TCW_M2V[0]-TCCW_M2V[0], TCW_M2V[1]-TCCW_M2V[1]]
    TCW_M2VtoTCTCCW_V2M = [TCCW_V2M[0]-TCW_M2V[0], TCCW_V2M[1]-TCW_M2V[1]]
    TCCW_V2MtoTCW_V2M = [TCW_V2M[0]-TCCW_V2M[0], TCW_V2M[1]-TCCW_V2M[1]]
    TCW_V2MtoTCCW_M2V = [TCCW_M2V[0]-TCW_V2M[0], TCCW_M2V[1]-TCW_V2M[1]]

    for items in ball_storted:
        if (items[0] != 'M') and ([items[2],items[3]] != C) and (counter < 2):
            OB_back = [items[2], items[3]]

            TCCW_M2VtoOB_back = [OB_back[0]-TCCW_M2V[0], OB_back[1]-TCCW_M2V[1]]
            TCW_M2VtoOB_back = [OB_back[0]-TCW_M2V[0], OB_back[1]-TCW_M2V[1]]
            TCCW_V2MtoOB_back = [OB_back[0]-TCCW_V2M[0], OB_back[1]-TCCW_V2M[1]]
            TCW_V2MtoOB_back = [OB_back[0]-TCW_V2M[0], OB_back[1]-TCW_V2M[1]]

            C2OB_back = np.array([OB_back[0] - M[0], OB_back[1] - M[1]])              #母球與干擾球之向量
            L = abs(np.cross(M2V, C2OB_back) / np.linalg.norm(M2V))                   #用面積(平面向量外積)除於底(母球與虛擬球之向量)得到高 = 母球與虛擬球的連線到干擾球之距離

            if V[1] >= M[1]:
                if V[0] >= M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoOB_back, TCCW_M2VtoTCW_M2V)
                    pos_parallel_R = np.cross(TCW_M2VtoTCTCCW_V2M, TCW_M2VtoOB_back)
                    pos_V2M_normal = np.cross(TCCW_V2MtoTCW_V2M, TCCW_V2MtoOB_back)
                    pos_parallel_L = np.cross(TCW_V2MtoOB_back, TCW_V2MtoTCCW_M2V)
                    if (L < dis_limit) and (pos_M2V_normal < 0) and (pos_parallel_R > 0) and (pos_V2M_normal > 0) and (pos_parallel_L < 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
                elif V[0] < M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoTCW_M2V, TCCW_M2VtoOB_back)
                    pos_parallel_R = np.cross(TCW_M2VtoTCTCCW_V2M, TCW_M2VtoOB_back)
                    pos_V2M_normal = np.cross(TCCW_V2MtoOB_back, TCCW_V2MtoTCW_V2M)
                    pos_parallel_L = np.cross(TCW_V2MtoOB_back, TCW_V2MtoTCCW_M2V)
                    if (L < dis_limit) and (pos_M2V_normal > 0) and (pos_parallel_R > 0) and (pos_V2M_normal < 0) and (pos_parallel_L < 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
            elif V[1] < M[1]:
                if V[0] >= M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoOB_back, TCCW_M2VtoTCW_M2V)
                    pos_parallel_R = np.cross(TCW_M2VtoOB_back, TCW_M2VtoTCTCCW_V2M)
                    pos_V2M_normal = np.cross(TCCW_V2MtoTCW_V2M, TCCW_V2MtoOB_back)
                    pos_parallel_L = np.cross(TCW_V2MtoTCCW_M2V, TCW_V2MtoOB_back)
                    if (L < dis_limit) and (pos_M2V_normal < 0) and (pos_parallel_R < 0) and (pos_V2M_normal > 0) and (pos_parallel_L > 0):
                        back_OB_storted[counter] = [OB_back[0], OB_back[1], items[0]]
                        print("back_OB_storted{} = {}".format(counter,back_OB_storted[counter]))
                        counter = counter + 1
                elif V[0] < M[0]:
                    pos_M2V_normal = np.cross(TCCW_M2VtoTCW_M2V, TCCW_M2VtoOB_back)
                    pos_parallel_R = np.cross(TCW_M2VtoOB_back, TCW_M2VtoTCTCCW_V2M)
                    pos_V2M_normal = np.cross(TCCW_V2MtoOB_back, TCCW_V2MtoTCW_V2M)
                    pos_parallel_L = np.cross(TCW_V2MtoTCCW_M2V, TCW_V2MtoOB_back)
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
    global extend_V, point_TCCW45, point_TCW45
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
        V1 = [ (C[0]-2 * (billiard_radius) * Unit_C2H[0]), (C[1]-2 * (billiard_radius) * Unit_C2H[1]) ]    

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
            return 1
        else:
            return 0

    if mode == 'OB_plot':
        C2H= np.array([gHole[0]-C[0],gHole[1]-C[1]])                                       
        L_C2H = np.linalg.norm(C2H)                                     
        Unit_C2H = C2H / L_C2H                                             
        V1 = [ (C[0]-2 * (billiard_radius) * Unit_C2H[0]), (C[1]-2 * (billiard_radius) * Unit_C2H[1]) ]    

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
            if quantity>10:
                print("early stage")
                early_stage()
            elif (quantity<=10) and (quantity>6):
                print("middle stage")
                middle_stage() 
            else:
                print("final stage")
                final_stage()
            no_cueball = 0              
            
    

def find_holes():
    global H1, H2, H3, H4, H5, H6
    global hit_power_check
    global left_x_float, left_y_float
    global right_x_float, right_y_float

    # BMX = -344.149   
    # BMY = 242.455   
    # BMX_1 = BMX + 692.95
    # BMY_1 = BMY   

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

    H1 = [BMX + 41*Uv[0] + 41*Uu[0], BMY + 41*Uv[1] + 41*Uu[1]]    #洞H1 左下

    H2 = [BMX + 41*Uv[0] + 332.7*Uu[0], BMY + 41*Uv[1] + 332.7*Uu[1]]   #洞H2 左上

    H3 = [((BMX + BMX_1)/2) + 332.7*Uu[0], ((BMY + BMY_1)/2) + 332.7*Uu[1]]   #洞H3 中上

    H4 = [BMX_1 - 41*Uv[0] + 332.7*Uu[0], BMY_1 - 41*Uv[1] + 332.7*Uu[1]]   #洞H4 右上

    H5 = [BMX_1 - 41*Uv[0] + 41*Uu[0], BMY_1 - 41*Uv[1] + 41*Uu[1]]   #洞H5 右下

    H6 = [(BMX + BMX_1)/2 + 41*Uu[0], (BMY + BMY_1)/2 + 41*Uu[1]]    #洞H6 中下  

    hit_power_check = abs(H1[0] - H6[0])
    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)
    print("============================================================================")



def curve(my_data):
    global all_ball, M
    global H1, H2, H3, H4, H5, H6
    global billiard_radius

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
            center_x = round((img_pos[0]-(1280/2))*0.5, 3)
            center_y = round((img_pos[1]-(720/2))*0.5, 3)

            BASE_X = pic_Point[0] + center_x + 1.061
            BASE_Y = pic_Point[1] - center_y - sucker2camera[1] + 5.961

           
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
                 if dis_BallandHole[j] <= 1*billiard_radius :
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
def mulit_cueball(multi_M):
    global all_ball
    storted = []
    storted_M = []
    i = 0
    for item in all_ball:
        if item[0] != -999:
            storted.append(item)
    for item in multi_M:
        if item[0] != -999:
            storted_M.append(item)
    for ball in storted:
        if (storted_M[i][0] > ball[2]-3 and storted_M[i][0] < ball[2]) and \
           (storted_M[i][0] < ball[2]+3 and storted_M[i][0] > ball[2]) and \
           (storted_M[i][0] > ball[3]-3 and storted_M[i][0] < ball[3]) and \
           (storted_M[i][0] < ball[3]+3 and storted_M[i][0] > ball[3]):

            print("not cueball")
        else:
            M = [storted_M[i][0], storted_M[i][1], 100.171 , 180 , 0 , -179.929]
        i+=1
    
    return M

def find_cueball(data):
    global multi_M
 #=====================================================================
    c_x = 637.36542424
    c_y = 359.93598824
    f_x = 907.42716381
    f_y = 908.47256525
    
    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([1.82612813e-01, -5.57847355e-01, 4.63194714e-04 , -5.94172999e-05, 5.18000478e-01])
    
    M_set = [[-999 for i in range(2)]for j in range(16)]
    cnt = 0
    multi_M = 0

    for item in data:
        if item[0] == 'M':
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]
            center_x = round((img_pos[0]-(1280/2))*0.5, 3)
            center_y = round((img_pos[1]-(720/2))*0.5, 3)

            BASE_X = pic_Point[0] + center_x + 1.061
            BASE_Y = pic_Point[1] - center_y - sucker2camera[1] + 5.961
            M_set[cnt] = [BASE_X , BASE_Y]
            cnt+=1

    if cnt >= 2:
        multi_M = 1

    return M_set

def yolo_pic(num):
    global M
    global loading_key

    loading_key =1

    imageName = "/home/oscaryang/picture/SmallBall_pic/higher_pic_point_" + str(num) + ".jpg"

    img = cv2.imread(imageName)
    # img = cv2.imread("/home/oscaryang/picture/SmallBall_pic/higher_pic_point_9_r.png")
    ball_information = YOLO_Detect.detect_ALL(img,0.5)

    print(ball_information)

    return ball_information

def plot_table():
    global H1, H2, H3, H4, H5, H6, M, C, V, V1, V2
    global all_ball, gHole
    global kissball
    global extend_V, point_TCCW45, point_TCW45
    global fin_point_TCCW45, fin_point_TCW45
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
        
        plt.plot(H2V_x,H2V_y)
        plt.plot(TCCW45_x,TCCW45_y, 'w')
        plt.plot(TCW45_x,TCW45_y, 'm')
        plt.plot(angle_V2M_x,angle_V2M_y, 'r')
        V_center = plt.Circle((V[0], V[1]), 12.45, color = 'y')
        axes.set_aspect(1)
        axes.add_artist(V_center)
    if kissball == 1:
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
    global is_busy
    global M
    global open_ball, shoot
    global no_cueball
    global pic_x_float, pic_y_float, pic_z_float, pic_yaw_float
    global pic_Point

    find_holes()

    step = 0
    pic_Point = [pic_x_float, pic_y_float, pic_z_float, 180.000, 0, pic_yaw_float]

    start = int(input("input 1 to start : "))
    while 1:
        if is_busy == True:
            is_busy = False
        if is_busy == False:
            if start == 1:
                is_busy = True
                print("go to start point")
                step = 1

            if step == 1:
                if open_ball == 1:
                    is_busy = True
                    print("go to above open point")
                    print("go to open point")

                    print("hit max power")
                    print("back to above point")
                    open_ball = 0               
                    step = 0

                elif open_ball == 0:
                    is_busy = True
                    print("go to pic point ")
                    print("pic_point:",pic_Point)
                    time.sleep(0.1)
                    num = int(input("enter pic number : "))
                    ball_information = yolo_pic(num)
                    M_set = find_cueball(ball_information)
                    ball_quantity = curve(ball_information)
                    if multi_M == 1:
                        M = mulit_cueball(M_set)
                    
                    M = [M_set[0][0], M_set[0][1], 100.171 , 180 , 0 , -179.929]
                    print("M:{},{}".format(M[0],M[1]))
                    print("============================================================================")
                    
                    step = 2

            if step == 2:
                target_Strategy(ball_quantity)
                if no_cueball == 1:
                    print("turn on light")
                    print("go to pic point again")
                    M_doublecheck = yolo_pic()
                    M_set = find_cueball(M_doublecheck)
                    if multi_M == 1:
                        M = mulit_cueball(M_set)  
                    M = [M_set[0][0], M_set[0][1], 100.171 , 180 , 0 , -179.929]
                    print("turn off light")
                    no_cueball = 0
                    step = 2
                elif no_cueball == 2:
                    step = 0

                step = 3

            if step == 3:
                FinalAngle()
                plot_table()

            if step == 4:
                is_busy = True
                hit_angle = 90-target_angle
                print(hit_angle)
                print("============================================================================")
                target = [M[0], M[1], 23.803, 180.000, 0.000, hit_angle]
                if (hit_angle <= 0) and (hit_angle >= -90):
                    yaw_zero = [M[0], M[1], 41.072, 180.000, 0.000, 0]
                    print("go to target 1")
                elif (target_angle < -90) and (target_angle >= -180):
                    yaw_180 = [M[0], M[1], 41.072, 180.000, 0.000, 180]
                    print("go to target 2")
                else:
                    print("go to target 3")
                step = 5

            if step == 5:
                is_busy = True
                if shoot == 255:
                    print("255")
                if shoot == 150:
                    print("150")
                step = 6
            
            if step == 6:
                is_busy = True
                if (hit_angle <= 0) and (hit_angle >= -90):
                    yaw_zero = [M[0], M[1], 41.072, 180.000, 0.000, 0]
                    print("go home 1")
                elif (target_angle < -90) and (target_angle >= -180):
                    go_home_180 = [M[0], M[1], 41.072, 180.000, 0.000, 90]
                    print("go home 2")
                else:
                    go_home_zero = [M[0], M[1], 41.072, 180.000, 0.000, 90]
                    print("go home 3")
                step = 0
            
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

    if pic_z_input.get() != '':
        pic_z_str = pic_z_input.get()
        pic_z_float = float(pic_z_str)
        print("pic_z_float:",pic_z_float)       

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

    
if __name__ == '__main__':
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

    pic_z_label = tk.Label(main_menu, text="拍照位置z:", font=('Arial',30))
    pic_z_label.grid(row=3, column=0)
    pic_z_input = tk.Entry(main_menu, bd=10)
    pic_z_input.grid(row=3, column=1)
    pic_z_input.bind("<Return>",tk_input)

    pic_yaw_label = tk.Label(main_menu, text="拍照位置yaw:", font=('Arial',30))
    pic_yaw_label.grid(row=4, column=0)
    pic_yaw_input = tk.Entry(main_menu, bd=10)
    pic_yaw_input.grid(row=4, column=1)
    pic_yaw_input.bind("<Return>",tk_input)

    left_x_label = tk.Label(main_menu, text="左下角落x:", font=('Arial',30))
    left_x_label.grid(row=5, column=0)
    left_x_input = tk.Entry(main_menu, bd=10)
    left_x_input.grid(row=5, column=1)
    left_x_input.bind("<Return>",tk_input)

    left_y_label = tk.Label(main_menu, text="左下角落y:", font=('Arial',30))
    left_y_label.grid(row=6, column=0)
    left_y_input = tk.Entry(main_menu, bd=10)
    left_y_input.grid(row=6, column=1)
    left_y_input.bind("<Return>",tk_input)
     
    right_x_label = tk.Label(main_menu, text="右下角落x:", font=('Arial',30))
    right_x_label.grid(row=7, column=0)
    right_x_input = tk.Entry(main_menu, bd=10)
    right_x_input.grid(row=7, column=1)
    right_x_input.bind("<Return>",tk_input)

    right_y_label = tk.Label(main_menu, text="右下角落y:", font=('Arial',30))
    right_y_label.grid(row=8, column=0)
    right_y_input = tk.Entry(main_menu, bd=10)
    right_y_input.grid(row=8, column=1)
    right_y_input.bind("<Return>",tk_input)

    main = tk.Button(main_menu, text="主程式", bg="yellow", font=('Arial',20), width=20, height=20, command=Mission_trigger)
    main.place(x=450,y=0)

    main_shutdown = tk.Button(main_menu, text='關閉程式', bg='red', font=('Arial',20), width=30, height=20)
    main_shutdown.place(x=800, y=0)
    main_shutdown.bind("<Button-1>",shutdownbutton)

    main_menu.mainloop()

