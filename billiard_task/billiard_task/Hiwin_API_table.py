from calendar import c
from operator import mod
import numpy as np
from ctypes import *
import time
import rospy

import numpy as np
import matplotlib
from matplotlib import pyplot as plt 
import cv2
import YOLO_Detect
import pyrealsense2 as rs

###
start_point = [-368.000, -0.042 ,293.500, 180.000, 0.000, -180.000]
pic_Point = [-1.034, 408.671, 359.568, 180.000, 0, -179.929]
sucker2camera = [-47.0, -32.5] 
hole1 = [-999, -999, -999, -999, -999, -999]
hole2 = [-999, -999, -999, -999, -999, -999]
hole3 = [-999, -999, -999, -999, -999, -999]
hole4 = [-999, -999, -999, -999, -999, -999]
hole5 = [-999, -999, -999, -999, -999, -999]
hole6 = [-999, -999, -999, -999, -999, -999]
###

# Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)


def find_holes():
    BMX = -347.402
    BMY = 263.554        
    BMX_1 = 340.180
    BMY_1 = 263.554

    TCC = np.array([[0, -1],    #逆時針旋轉的旋轉矩陣
                    [1,  0]])

    B2B= np.array([BMX_1-BMX,BMY_1-BMY])    #從BM指向BM_1的向量                       
    B2BL = np.linalg.norm(B2B)              #B2B向量的長度
    Uv = B2B / B2BL                         #B2B向量的單位向量  
    Uu = TCC.dot(Uv)                      #B2B向量逆時針90度旋轉的單位向量
# 5.1 36.9
    global H1
    H1 = [BMX + 5.1*Uv[0] + 5.1*Uu[0], BMY + 5.1*Uv[1] + 5.1*Uu[1]]    #洞H1 左下
    global H2
    H2 = [BMX + 5.1*Uv[0] + 36.9*Uu[0], BMY + 5.1*Uv[1] + 36.9*Uu[1]]   #洞H2 左上
    global H3
    H3 = [((BMX + BMX_1)/2) + 36.9*Uu[0], ((BMY + BMY_1)/2) + 36.9*Uu[1]]   #洞H3 中上
    global H4
    H4 = [BMX_1 - 5.1*Uv[0] + 36.9*Uu[0], BMY_1 - 5.1*Uv[1] + 36.9*Uu[1]]   #洞H4 右上
    global H5
    H5 = [BMX_1 - 5.1*Uv[0] + 5.1*Uu[0], BMY_1 - 5.1*Uv[1] + 5.1*Uu[1]]   #洞H5 右下
    global H6
    H6 = [(BMX + BMX_1)/2 + 4.1*Uu[0], (BMY + BMY_1)/2 + 4.1*Uu[1]]    #洞H6 中下  
    print('\r\n', 'H1 : ', H1,'\r\n', 'H2 : ',H2,'\r\n', 'H3 : ',H3,'\r\n', 'H4 : ',H4,'\r\n', 'H5 : ',H5,'\r\n', 'H6 : ',H6)


def find_ball(data):
    i = 0
    c_x = 642.69902197
    c_y = 370.27915175
    f_x = 918.73516739
    f_y = 919.13339111
    
    all_ball = [[-999 for i in range(2)] for j in range(16)]

    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    #dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    dist_coef = np.array([1.04827825e-01, 1.62037134e-01, -9.84883489e-04 , 1.89111101e-03, -1.35428691e+00])
    for item in data:
        if item[0] != -999:
            mx = float(item[2])
            my = float(item[3])
            img_pos = np.array([[[mx, my]]])
            img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

            BASE_X = pic_Point[0] + round((img_pos[0]-(1280/2))*0.5, 3) + sucker2camera[0]+13.796
            BASE_Y = pic_Point[1] - round((img_pos[1]-(720/2))*0.5, 3) - sucker2camera[1]+26.885
            
            all_ball[i] = [BASE_X , BASE_Y]
            i+=1


    return all_ball


def find_cueball(data):
    global M

    i = 0
    c_x = 642.69902197
    c_y = 370.27915175
    f_x = 918.73516739
    f_y = 919.13339111
    
    all_ball = [[-999 for i in range(2)] for j in range(16)]

    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    #dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    dist_coef = np.array([1.04827825e-01, 1.62037134e-01, -9.84883489e-04 , 1.89111101e-03, -1.35428691e+00])
    for item in data:
        if item[0] != -999:
            if item[0] == 'M':
                mx = float(item[2])
                my = float(item[3])
                img_pos = np.array([[[mx, my]]])
                img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

                BASE_X = pic_Point[0] + round((img_pos[0]-(1280/2))*0.5, 3) + sucker2camera[0]+13.796
                BASE_Y = pic_Point[1] - round((img_pos[1]-(720/2))*0.5, 3) - sucker2camera[1]+26.885
                
                M = [BASE_X , BASE_Y]
                i+=1
                break

    print(M)            

    return M

def plot_table(data):
    global H1, H2, H3, H4, H5, H6, M
    hole_x = [H1[0], H2[0], H3[0], H4[0], H5[0], H6[0], H1[0]]
    hole_y = [H1[1], H2[1], H3[1], H4[1], H5[1], H6[1], H1[1]]
    ball_sorted = []
    ball_x = []
    ball_y = []

    for item in data:
        if item[0] != -999:
            ball_sorted.append(item)
    print(ball_sorted)

    for balls in ball_sorted:
        ball_x.append(balls[0])
        ball_y.append(balls[1])
    print(ball_x)
    print(ball_y)

    fig = plt.figure() 
    axes = fig.add_subplot(111)
    axes.set_facecolor('g')
    
    plt.plot(hole_x,hole_y)

    for i in range(6):
        hole_circle = plt.Circle((hole_x[i], hole_y[i]), 2, fill=False)
        axes.set_aspect(1)
        axes.add_artist(hole_circle)

    for j in range(len(ball_sorted)):
        if (ball_x[j] == M[0]) & (ball_y[j] == M[1]):
            ball_center = plt.Circle((ball_x[j], ball_y[j]), 5, color = 'w')
            ball_circle = plt.Circle((ball_x[j], ball_y[j]), 15, fill=False)
            axes.set_aspect(1)
            axes.add_artist(ball_center) 
            axes.add_artist(ball_circle) 
        else:    
            ball_center = plt.Circle((ball_x[j], ball_y[j]), 5)
            ball_circle = plt.Circle((ball_x[j], ball_y[j]), 15, fill=False)
            axes.set_aspect(1)
            axes.add_artist(ball_center) 
            axes.add_artist(ball_circle) 

    plt.title("Table")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.xlim(-100,100)
    plt.ylim(-100,100)
    plt.show()

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
    global M

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
    hole1 = [H1[0], H1[1], 4.524, 180.000, 0.000, -180.000]
    hole2 = [H2[0], H2[1], 4.524, 180.000, 0.000, -180.000]
    hole3 = [H3[0], H3[1], 4.524, 180.000, 0.000, -180.000]
    hole4 = [H4[0], H4[1], 4.524, 180.000, 0.000, -180.000]
    hole5 = [H5[0], H5[1], 4.524, 180.000, 0.000, -180.000]
    hole6 = [H6[0], H6[1], 4.524, 180.000, 0.000, -180.000]
    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        
        PTP_Move(start_point,50,20)
        PTP_Move(start_point,50,20)

        PTP_Move(hole1,50,20)
        cv2.waitKey(0)

        PTP_Move(pic_Point,50,20)
        time.sleep(0.1)
        frames = pipeline.wait_for_frames()
        img = frames.get_color_frame()
        img = np.asanyarray(img.get_data())

        ball_information = YOLO_Detect.detect_ALL(img)
        all_ball = find_ball(ball_information)  
        M = find_cueball(ball_information)
        plot_table(all_ball)


        PTP_Move(hole1,50,20)
        time.sleep(0.1)
        PTP_Move(hole2,50,20)
        time.sleep(0.1)
        PTP_Move(hole3,50,20)
        time.sleep(0.1)
        PTP_Move(hole4,50,20)
        time.sleep(0.1)
        PTP_Move(hole5,50,20)
        time.sleep(0.1)
        PTP_Move(hole6,50,20)
        cv2.waitKey(0)
        if(modbus.Arm_State_REGISTERS() == 1):
            break

    modbus.Modbus_Close()
    print("Modbus Close")  


    
