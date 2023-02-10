from calendar import c
from operator import mod
from pickletools import uint8
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
is_busy = True

start_point = [-368.000, -0.042 ,293.500, 180.000, 0.000, 180.000]
pic_Point = [-1.847, 403.268, 373.876, 180.000, 0, 90.000]
hit_point = [149.711, 449.517, 6.903, 180.000, 0, 180]
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


def Mission_trigger():
    global is_busy

    IO_Port_in = 300  #開始按鈕
    IO_Port_4 = 303 # D4 未降壓
    IO_Port_5 = 304 # D5 降壓

    if is_busy == True:
        is_busy = False
    if is_busy == False:
        print(DI_State)
        print("***********************************************")
        PTP_Move(hit_point,50,20)
        
        modbus.DO(IO_Port_4,1)
        time.sleep(0.1)
        modbus.DO(IO_Port_4,0)
        # time.sleep(5)
        # modbus.DO(IO_Port_5,1)
        # time.sleep(0.1)
        # modbus.DO(IO_Port_5,0)


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
    # 300 -> DO1(氣閥)
    # 301 -> DO2

    # 電工 = 上面比較大的鄉子

    Arm_state = 0
    DI_State = 0
    IO_Port_in = 300  #開始按鈕
    IO_Port_4 = 303 # D4 弱
    IO_Port_5 = 304 # D5 強
    IO_Port_6 = 305 # D5 降壓

    modbus.DI.argtypes = [c_int]
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
    PTP_Move(start_point,50,20)
    PTP_Move(start_point,50,20)
    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        a = int(input("input:"))
        if a == 1:
            modbus.DO(IO_Port_4,1)
            time.sleep(1)
            modbus.DO(IO_Port_4,0)
        if a == 2:
            modbus.DO(IO_Port_5,1)
            time.sleep(1)
            modbus.DO(IO_Port_5,0)
        if a == 3:
            modbus.DO(IO_Port_6,1)
            time.sleep(1)
            modbus.DO(IO_Port_6,0)

        # if(modbus.DI(IO_Port_in) == 1):
        #     Mission_trigger()
        
        # if(modbus.Arm_State_REGISTERS() == 1):
        #     break

    modbus.Modbus_Close()
    print("Modbus Close")  

    
