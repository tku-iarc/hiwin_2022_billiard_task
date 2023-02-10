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

import YOLO_Detect


###
start_point = [-368.000, -0.042 ,293.500, 180.000, 0.000, 90.000]
pic_Point = [-1.034, 408.671, 359.568, 180.000, 0, 90.000]
cueball_point = [000.000, 000.000, 000.000, 000.000, 000.000]
doublecheck_point = [000.000, 000.000, 000.000, 000.000, 000.000]
sucker2camera = [-47.0, -32.5] 
###

###
error_x = -0.2
error_y = -0.3
pic_height_1 = 49.3
pic_height_2 = 18.3
pic_pos_x = 0.0
pic_pos_y = 36.8
pic_pos_z = pic_height_1



# Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)

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

def find_cueball(data):
    global  A, pic_pos_x, pic_pos_y, pic_pos_z
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
    z_world = pic_pos_z
    for item in data:
        if item[0] == 'M':
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

            BASE_X = pic_Point[0] + round((img_pos[0]-(1280/2))*0.5, 3) + sucker2camera[0]+13.796
            BASE_Y = pic_Point[1] - round((img_pos[1]-(720/2))*0.5, 3) - sucker2camera[1]+26.885
            
            M = [BASE_X , BASE_Y , -61 , 180 , 0 , -179.929]

    return M

# 15/30 = 
# 105/202
# y : 26.885
# x : 13.796

# def find_cueball(data, mode):
    
#     for item in data:
#         print(item)
#         if item[0] == 'M':
#             mx = float(item[2] + item[4])/2
#             my = float(item[3] + item[5])/2
#             if mode == 'first_TakePic':
#                 BASE_X = pic_Point[0] + round((mx-(1280/2))*0.519, 3) + sucker2camera[0]
#                 BASE_Y = pic_Point[1] - round((my-(720/2))*0.519, 3) - sucker2camera[1]
#                 M = [BASE_X , BASE_Y , 100.171 , 180.000, 0, -179.929]
#             if mode == 'doublecheck':
#                 BASE_X = pic_Point[0] + round((mx-(1280/2))*0.217, 3) + sucker2camera[0]
#                 BASE_Y = pic_Point[1] - round((my-(720/2))*0.217, 3) - sucker2camera[1]
#                 M = [BASE_X , BASE_Y , -70 , 180.000, 0, -179.929]
#             print(M)

#     return M
    # first_TakePic
    # 15/29=0.517
    # 105/202=0.519

    # doublecheck
    # 15/69 = 0.217
    # 105/482 = 0.217
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

    # ball_information = [[-999 for i in range(6)] for j in range(16)]

    Arm_state = 0
    IO_Port = 301 # D0
    
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        
        PTP_Move(pic_Point,50,20)
        PTP_Move(pic_Point,50,20)
        time.sleep(0.1)
        frames = pipeline.wait_for_frames()
        img = frames.get_color_frame()
        img = np.asanyarray(img.get_data())
        ball_information = YOLO_Detect.detect_ALL(img)

        # print(len(ball_information))

        # for i in len(ball_information):
        #     print("label: {} , confidence: {:3.2f} , xmin: {:4.0f} , ymin: {:4.0f} , xmax: {:4.0f} , ymax: {:4.0f}".format(ball_information[i][0], float(ball_information[i][1]), ball_information[i][2], ball_information[i][3], ball_information[i][4], ball_information[i][5]))

        print(ball_information)
        
        cv2.waitKey(0)

        # cueball_point = find_cueball(ball_information ,mode = 'first_TakePic')
        # PTP_Move(cueball_point,50,20)
        # time.sleep(0.1)

        # frames = pipeline.wait_for_frames()
        # img = frames.get_color_frame()
        # img = np.asanyarray(img.get_data())
        # ball_information = YOLO_Detect.detect_ALL(img)
        # cv2.waitKey(0)
        # doublecheck_point = find_cueball(ball_information ,mode = 'doublecheck')
        # PTP_Move(doublecheck_point,20,20)
        # time.sleep(0.1)
        cueball_point = find_cueball(ball_information)
        print(cueball_point)

        PTP_Move(cueball_point,20,20)
        time.sleep(0.1)
        if(modbus.Arm_State_REGISTERS() == 1):
            break

    modbus.Modbus_Close()
    print("Modbus Close")  

    
