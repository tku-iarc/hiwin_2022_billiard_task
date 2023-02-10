from calendar import c
from errno import EEXIST
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
start_point = [-367.708, 12.711 ,445.693, 180.000, 0.000, 180.00]
pic_Point = [-0.881, 342.345, 373.081, 180.000, 0, 92.118]
close_pic_point = [-7.411, 397.058, 388.887, 180.000, 0, 91.249]
higher_pic_point = [6.624, 399.776, 410.976, 180.000, 0, 91.663]
cueball_point = [-1.034, 408.671, 100.171, 180.000, 0, -179.929]
doublecheck_point = [-1.034, 408.671, 100.171, 180.000, 0, -180]
sucker2camera = [-47.0, -32.5] 
hole1 = [-999, -999, -999, -999, -999, -999]
hole2 = [-999, -999, -999, -999, -999, -999]
hole3 = [-999, -999, -999, -999, -999, -999]
hole4 = [-999, -999, -999, -999, -999, -999]
hole5 = [-999, -999, -999, -999, -999, -999]
hole6 = [-999, -999, -999, -999, -999, -999]
###



# # Realsense
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# pipeline.start(config)
# sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
# sensor.set_option(rs.option.auto_exposure_priority, True)

def find_holes():
    global H1, H2, H3, H4, H5, H6
    BMX = -361.381
    BMY = 252.039 
    BMX_1 = 367.164
    BMY_1 = 261.067

    TCC = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])

    B2B= np.array([BMX_1-BMX,BMY_1-BMY])    #從BM指向BM_1的向量                       
    B2BL = np.linalg.norm(B2B)              #B2B向量的長度
    Uv = B2B / B2BL                         #B2B向量的單位向量  
    Uu = TCC.dot(Uv)                      #B2B向量逆時針90度旋轉的單位向量
# 5.1 36.9
    H1 = [BMX + 47.2*Uv[0] + 47.2*Uu[0], BMY + 47.2*Uv[1] + 47.2*Uu[1]]    #洞H1 左下

    H2 = [BMX + 47.2*Uv[0] + 360*Uu[0], BMY + 47.2*Uv[1] + 360*Uu[1]]   #洞H2 左上

    H3 = [((BMX + BMX_1)/2) + 360*Uu[0], ((BMY + BMY_1)/2) + 360*Uu[1]]   #洞H3 中上

    H4 = [BMX_1 - 47.2*Uv[0] + 360*Uu[0], BMY_1 - 47.2*Uv[1] + 360*Uu[1]]   #洞H4 右上

    H5 = [BMX_1 - 47.2*Uv[0] + 47.2*Uu[0], BMY_1 - 47.2*Uv[1] + 47.2*Uu[1]]   #洞H5 右下

    H6 = [(BMX + BMX_1)/2 + 47.2*Uu[0], (BMY + BMY_1)/2 + 47.2*Uu[1]]    #洞H6 中下  

    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)


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
        # frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        # frames.get_color_frame()                    # 同上
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
    find_holes()
    hole1 = [H1[0], H1[1], 197, 180.000, 0.000, 90]
    hole2 = [H2[0], H2[1], 197, 180.000, 0.000, 90]
    hole3 = [H3[0], H3[1], 197, 180.000, 0.000, 90]
    hole4 = [H4[0], H4[1], 197, 180.000, 0.000, 90]
    hole5 = [H5[0], H5[1], 197, 180.000, 0.000, 90]
    hole6 = [H6[0], H6[1], 197, 180.000, 0.000, 90]
    print(hole1)
    print(hole2)
    print(hole3)
    print(hole4)
    print(hole5)
    print(hole6)
    aa = int(input("in :"))
    while aa == 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        
        # PTP_Move(start_point,50,20)
        # PTP_Move(start_point,50,20)

        # PTP_Move(pic_Point,50,20)


        # time.sleep(0.1)
        # frames = pipeline.wait_for_frames()
        # img = frames.get_color_frame()
        # img = np.asanyarray(img.get_data())
        # YOLO_Detect.detect_ALL(img)

        PTP_Move(hole1,50,20)
        time.sleep(0.5)
        PTP_Move(hole2,50,20)
        time.sleep(0.5)
        PTP_Move(hole3,50,20)
        time.sleep(0.5)
        PTP_Move(hole4,50,20)
        time.sleep(0.5)
        PTP_Move(hole5,50,20)
        time.sleep(0.5)
        PTP_Move(hole6,50,20)
        time.sleep(0.5)
        PTP_Move(hole1,50,20)
        time.sleep(0.5)

        # center_H1H2 = [(H1[0]+H2[0])/2, (H1[1]+H2[1])/2]
        # openpoint_L = [center_H1H2[0]+140, center_H1H2[1], 40.000, 180.000, 0.000, 90]
        # center_H4H5 = [(H4[0]+H5[0])/2, (H4[1]+H5[1])/2]
        # openpoint_R = [center_H4H5[0]-140, center_H4H5[1], 40.000, 180.000, 0.000, 90]

        # PTP_Move(openpoint_L,50,20)
        # time.sleep(10)
        # PTP_Move(openpoint_R,50,20)

        
        if(modbus.Arm_State_REGISTERS() == 1):
            break

    modbus.Modbus_Close()
    print("Modbus Close")  

    
