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

from logging import root, shutdown
import tkinter as tk
from _tkinter import *
import os, sys
from unicodedata import name

import YOLO_Detect
import hiwin_motor
# import take_picture

is_busy = True
open_ball = 1
shoot = 255
open_L = 1
open_R = 0

flag = [0, 0, 0, 0, 0]      # flag = [拍照, 左角落, 右角落, 左開球, 右開球]

start_point = [-367.708, 12.711 ,293.500, 180.000, 0.000, 90.605]
pic_Point = [1.563, 392.056, 380.721, 180.000, 0, 90.552]
left_corner_point = [-348.389, 238.593, 30, 180, 0, 90]
right_corner_point = [347.585, 240.51, 30, 180, 0, 90]
open_point_right = [156.912, 450.010, 9.072, 180.000, 0, 180]
open_point_left = [156.912, 450.010, 9.072, 180.000, 0, 180]
return_home = [0.000, 368.000, 293.5, 180, 0, 90]

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


# # Realsense
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# pipeline.start(config)
# sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
# sensor.set_option(rs.option.auto_exposure_priority, True)
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

def curve(my_data):
    global all_ball, M
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
        if (item[0] != 'M') & (float(item[1]) > 80): 
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


def find_cueball(data):
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

            BASE_X = pic_Point[0] + new_center_x - 2
            BASE_Y = pic_Point[1] - center_y - sucker2camera[1] + 6.531
            M = [BASE_X , BASE_Y , 100.171 , 180 , 0 , -179.929]

    return M

# def yolo_pic():
#     global M
#     global loading_key

#     loading_key =1

#     frames = pipeline.wait_for_frames()
#     img = frames.get_color_frame()
#     img = np.asanyarray(img.get_data())
#     ball_information = YOLO_Detect.detect_ALL(img,0.5)

#     print(ball_information)
#     M = find_cueball(ball_information)
#     curve(ball_information)
#     plot_table()   

def plot_table():
    global H1, H2, H3, H4, H5, H6, M
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
    plt.ion()
    plt.pause(1.5)
    plt.close()
 

def find_holes():
    global H1, H2, H3, H4, H5, H6
    global BMX, BMY, BMX_1, BMY_1
    global openpoint_L, openpoint_R
    BMX = -345.378    
    BMY = 242.258   
    BMX_1 = BMX + 692.95
    BMY_1 = BMY       

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

    center_H1H2 = [BMX+31.2, (H1[1]+H2[1])/2]
    openpoint_L = [center_H1H2[0]+150, center_H1H2[1]]
    center_H4H5 = [BMX_1-31.2, (H4[1]+H5[1])/2]
    openpoint_R = [center_H4H5[0]-150, center_H4H5[1]]

    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)
    print("============================================================================")

# def pic_check():                 # 確認拍照位置
#     PTP_Move(pic_Point,50,20)
#     yolo_pic() 
#     cv2.destroyAllWindows()
#     flag = [1,0,0,0,0]
#     print(flag)

def look():                      # 開啟相機
    take_look = 0
    while take_look != 1:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Convert images to numpy arrays 把图像转换为numpy data
        color_image = np.asanyarray(color_frame.get_data())

        cv2.namedWindow('MyD415')
        cv2.imshow('MyD415', color_image)

        key = cv2.waitKey(1)

        flag = [1,0,0,0,0]
        print(flag)

        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            pipeline.stop()
            take_look = 1

def left_corner_check():           # 左下角落
    PTP_Move(left_corner_point,50,20)
    flag = [0,1,0,0,0]
    print(flag)
            
def right_corner_check():          # 右下角落
    PTP_Move(right_corner_point,50,20)
    flag = [0,0,1,0,0]
    print(flag)

def left_open_check():           # 左開球    
    global openpoint_L, openpoint_R

    hiwin_motor.motor_contorl(4)
    open_point_left = [openpoint_L[0], openpoint_L[1], -8.968, 180.000, 0, 0]
    open_point_left_above = [openpoint_L[0], openpoint_L[1], 40, 180.000, 0, 90]
    PTP_Move(open_point_left_above,50,20)
    time.sleep(1)
    PTP_Move(open_point_left,50,20)
    flag = [0,0,0,1,0]
    print(flag)

def right_open_check():         # 右開球
    global openpoint_L, openpoint_R

    hiwin_motor.motor_contorl(4)
    open_point_right = [openpoint_R[0], openpoint_R[1], -8.968, 180.000, 0, 180]
    open_point_right_above = [openpoint_R[0], openpoint_R[1], 40, 180.000, 0, 90]
    
    PTP_Move(open_point_right_above,50,20)
    time.sleep(1)
    PTP_Move(open_point_right,50,20)
    flag = [0,0,0,0,1]
    print(flag)

def hit_check():                # 電磁炮
    PTP_Move(return_home,50,20)
    hiwin_motor.motor_contorl(3)
    time.sleep(0.5)
    hiwin_motor.motor_contorl(2)
    time.sleep(0.5)
    hiwin_motor.motor_contorl(1)
    time.sleep(0.5)
    modbus.DO(IO_Port_4,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_4,0)
    time.sleep(1)
    modbus.DO(IO_Port_5,1)
    time.sleep(0.1)
    modbus.DO(IO_Port_5,0)

# def move(event):
#     global pic_Point, left_corner_point, right_corner_point, open_point_left, open_point_right
#     global openpoint_L, openpoint_R

#     pic_Point = [1.563, 392.056, 380.721, 180.000, 0, 90.552]
#     left_corner_point = [-348.389, 238.593, 30, 180, 0, 90]
#     right_corner_point = [348.741, 238.596, 30, 180, 0, 90]
#     open_point_left = [openpoint_L[0], openpoint_L[1], -8.968, 180.000, 0, 0]
#     open_point_right = [openpoint_R[0], openpoint_R[1], -8.968, 180.000, 0, 180]

#     # flag = [拍照, 左角落, 右角落, 左開球, 右開球]

#     up = 0
#     down = 0
#     left = 0
#     right = 0

#     if flag[0] == 1: 
#         if move_up.bind("<Button>"):
#             up += 0.2
#             pic_Point = [1.563, 392.056+up, 380.721, 180.000, 0, 90.552]
#         if move_down.bind("<Button>"):
#             down += 0.2
#             pic_Point = [1.563, 392.056-down, 380.721, 180.000, 0, 90.552]
#         if move_right.bind("<Button>"):
#             right += 0.2
#             pic_Point = [1.563+right, 392.056, 380.721, 180.000, 0, 90.552]
#         if move_left.bind("<Button>"):
#             left += 0.2
#             pic_Point = [1.563-left, 392.056, 380.721, 180.000, 0, 90.552]
#         print(pic_Point)

#     elif flag[1] == 1: 
#         if move_up.bind("<Button>"):
#             up += 0.2
#             left_corner_point = [-348.389, 238.593+up, 30, 180, 0, 90]
#         if move_down.bind("<Button>"):
#             down += 0.2
#             left_corner_point = [-348.389, 238.593-down, 30, 180, 0, 90]
#         if move_right.bind("<Button>"):
#             right += 0.2
#             left_corner_point = [-348.389+right, 238.593, 30, 180, 0, 90]
#         if move_left.bind("<Button>"):
#             left += 0.2
#             left_corner_point = [-348.389-left, 238.593, 30, 180, 0, 90]
#         print(left_corner_point)

#     elif flag[2] == 1: 
#         if move_up.bind("<Button>"):
#             up += 0.2
#             right_corner_point = [348.741, 238.596+up, 30, 180, 0, 90]
#         if move_down.bind("<Button>"):
#             down += 0.2
#             right_corner_point = [348.741, 238.596-down, 30, 180, 0, 90]
#         if move_right.bind("<Button>"):
#             right += 0.2
#             right_corner_point = [348.741+right, 238.596, 30, 180, 0, 90]
#         if move_left.bind("<Button>"):
#             left += 0.2
#             right_corner_point = [348.741-left, 238.596, 30, 180, 0, 90]
#         print(right_corner_point)

#     elif flag[3] == 1: 
#         if move_up.bind("<Button>"):
#             up += 0.2
#             open_point_left = [openpoint_L[0], openpoint_L[1]+up, -8.968, 180.000, 0, 0]
#         if move_down.bind("<Button>"):
#             down += 0.2
#             open_point_left = [openpoint_L[0], openpoint_L[1]-down, -8.968, 180.000, 0, 0]
#         if move_right.bind("<Button>"):
#             right += 0.2
#             open_point_left = [openpoint_L[0]+right, openpoint_L[1], -8.968, 180.000, 0, 0]
#         if move_left.bind("<Button>"):
#             left += 0.2
#             open_point_left = [openpoint_L[0]-left, openpoint_L[1], -8.968, 180.000, 0, 0]
#         print(open_point_left)

#     elif flag[4] == 1: 
#         if move_up.bind("<Button>"):
#             up += 0.2
#             open_point_right = [openpoint_R[0], openpoint_R[1]+up, -8.968, 180.000, 0, 180]
#         if move_down.bind("<Button>"):
#             down += 0.2
#             open_point_right = [openpoint_R[0], openpoint_R[1]-down, -8.968, 180.000, 0, 180]
#         if move_right.bind("<Button>"):
#             right += 0.2
#             open_point_right = [openpoint_R[0]+right, openpoint_R[1], -8.968, 180.000, 0, 180]
#         if move_left.bind("<Button>"):
#             left += 0.2
#             open_point_right = [openpoint_R[0]-left, openpoint_R[1], -8.968, 180.000, 0, 180]
#         print(open_point_right)


def shutdownbutton(event):
    os._exit(os.EX_OK)   

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
    IO_Port_4 = 303 # D4 弱
    IO_Port_5 = 304 # D5 強
    #IO_Port_5 = 304 # D5 降壓

    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()
    PTP_Move(start_point,50,20)
    PTP_Move(start_point,50,20)
    hiwin_motor.motor_contorl(1)
    find_holes()


    main_menu = tk.Tk()
    main_menu.title('主控')
    main_menu.geometry('1280x720')
    main_menu.configure(background = 'grey')

    pic = tk.Button(main_menu, text="確認拍照位置", bg="green", font=('Arial',20), width=12, height=7, command=pic_check)
    pic.place(x=0,y=0)
    look_table = tk.Button(main_menu, text="take_picture", bg="green", font=('Arial',20), width=12, height=7, command=look)
    look_table.place(x=0,y=300)

    left_corner = tk.Button(main_menu, text="左下角落", bg="yellow", font=('Arial',20), width=12, height=7, command=left_corner_check)
    left_corner.place(x=300,y=0)
    right_corner = tk.Button(main_menu, text="右下角落", bg="yellow", font=('Arial',20), width=12, height=7, command=right_corner_check)
    right_corner.place(x=300,y=300)

    left_open = tk.Button(main_menu, text="左開球", bg="blue", font=('Arial',20), width=12, height=7, command=left_open_check)
    left_open.place(x=600,y=0)
    right_open = tk.Button(main_menu, text="右開球", bg="blue", font=('Arial',20), width=12, height=7, command=right_open_check)
    right_open.place(x=600,y=300)   

    hit = tk.Button(main_menu, text="打擊測試", bg="pink", font=('Arial',20), width=12, height=7, command=hit_check)
    hit.place(x=900,y=0) 

    main_shutdown = tk.Button(main_menu, text='關閉程式', bg='red', font=('Arial',30), width=15, height=8)
    main_shutdown.place(x=900, y=300)
    main_shutdown.bind("<Button>",shutdownbutton)

    # move_up = tk.Button(main_menu, text='Up : y+0.2', bg='gray', font=('Arial',15), width=10, height=2)
    # move_up.place(x=140, y=550)
    # move_up.bind("<Button>",move)
    # move_down = tk.Button(main_menu, text='Down : y-0.2', bg='gray', font=('Arial',15), width=10, height=2)
    # move_down.place(x=140, y=650)
    # move_down.bind("<Button>",move)
    # move_left = tk.Button(main_menu, text='Left : x+0.2', bg='gray', font=('Arial',15), width=10, height=2)
    # move_left.place(x=0, y=600)
    # move_left.bind("<Button>",move)
    # move_right = tk.Button(main_menu, text='Right : x+0.2', bg='gray', font=('Arial',15), width=10, height=2)
    # move_right.place(x=280, y=600)
    # move_right.bind("<Button>",move)

    main_menu.mainloop()

    modbus.Modbus_Close()
    print("Modbus Close")  