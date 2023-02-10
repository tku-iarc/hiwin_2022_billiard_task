import pyrealsense2 as rs           # 版本目前不支援python3.10
import cv2
import numpy as np
import random
import math
import time
import copy
import statistics

from calendar import c
from ctypes import *
from operator import mod



def nothing(x):
    pass

cv2.namedWindow("setting", cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar('k_gauss','setting',13,25,nothing)
cv2.createTrackbar('k_close','setting',6,25,nothing)
cv2.createTrackbar('low_threshold','setting',60,200,nothing)
cv2.createTrackbar('high_threshold','setting',90,200,nothing)



target_height = 60.0 # (壓克力板方格紙)
base_z = [0,0,0]

target_area = 350000

# 手臂移動速度
speed_setting = 40
acceleration_setting = speed_setting


# 目前座標
Point_now = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]            # 預設為家座標

# 計算座標(下一個點)
Point_count = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]          # 預設為家座標

# 基本點位(pixel2mm_maker使用相同數值)
Point_basic = [ [-53.792, -23.449, target_height, -180, 0, 90], # 左下(-55.733, -23.829)
                [-55.183, 221.694, target_height, -180, 0, 90], # 左上(-56.844, 221.493)
                [456.755, -23.906, target_height, -180, 0, 90], # 右下(458.575, -21.367)
                [456.755, 221.694, target_height, -180, 0, 90], # 右上(456.755, 221.694)
                [250.000,  68.000,         443.5, -180, 0, 90]] # 家

Point_saved = [ [999.999, 999.999, 999.999, -180, 0, 90],
                [999.999, 999.999, 999.999, -180, 0, 90],
                [999.999, 999.999, 999.999, -180, 0, 90]]

"""
    設定
"""
# 手臂控制so檔
so_file = "/home/weng/RoboticArm_Puzzle/main/modbus_file/Hiwin_API.so"
modbus = CDLL(so_file)


'''
    用完後必關！！！！！！！！重要！！！！！
'''
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)



def detect(img):
    gradient = Canny(img)
    outline_img,angle,center = detect_angle(img,gradient)

    return angle, center, outline_img



def Canny(orig_img, k_gauss=13, low_threshold=60, high_threshold=90, k_close=6):
    # 灰階
    # gray_img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    gauss_img = cv2.GaussianBlur(orig_img, (k_gauss, k_gauss), 0)

    # Canny邊緣運算
    # low_threshold = 100
    # high_threshold = 120
    canny_img = cv2.Canny(gauss_img, low_threshold, high_threshold)

    # 閉運算(緩解Canny斷線問題)
    kernel = np.ones((k_close,k_close),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    return gradient


def detect_angle(orig_img, canny_img):
    outline_img = orig_img.copy()

    # 輪廓檢測(使用Canny檢測後影像繼續檢測)
    ret,thresh = cv2.threshold(canny_img,180,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    cnt = 0
    check = 0

    for i in range(len(contours)):
        if cv2.contourArea(contours[i])<500000 and cv2.contourArea(contours[i])>80000:
            cnt = contours[i]
            check = 1
            # print('area=',cv2.contourArea(contours[i]))

    if check == 1:
        rect = cv2.minAreaRect(cnt)     # 生成最小外接矩形
        box = cv2.boxPoints(rect)       # 生成外接矩形各點座標
        box = np.int0(box)              # 轉換為整數
        cv2.drawContours(outline_img, [box], 0, (0, 0, 255), 2)
        cv2.putText(outline_img,'1. '+str(box[0]),tuple(box[0]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'2. '+str(box[1]),tuple(box[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'3. '+str(box[2]),tuple(box[2]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'4. '+str(box[3]),tuple(box[3]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        tem_length_1to2 = math.sqrt(pow(box[0][0]-box[1][0],2) + pow(box[0][1]-box[1][1],2)).real
        tem_length_1to4 = math.sqrt(pow(box[0][0]-box[3][0],2) + pow(box[0][1]-box[3][1],2)).real
        if tem_length_1to2 > tem_length_1to4:
            star = (0,3)
            end = (1,2)
        else:
            star = (0,1)
            end = (2,3)
        x_start = ( (box[star[0]][0]+box[star[1]][0])/2 )
        y_start = ( (box[star[0]][1]+box[star[1]][1])/2 )
        x_end = ( (box[end[0]][0]+box[end[1]][0])/2 )
        y_end = ( (box[end[0]][1]+box[end[1]][1])/2 )
        origin_coordinate_x = int((x_start+x_end)/2)
        origin_coordinate_y = int((y_start+y_end)/2)
        if y_start>y_end:
            # 左上角(0,0)
            tem = x_start
            x_start = x_end
            x_end = tem
            tem = y_start
            y_start = y_end
            y_end = tem
        else:
            pass
        cv2.line(outline_img, (int(x_start),int(y_start)), (int(x_end),int(y_end)), (0, 100, 255), 1)

        angle_1 = round(math.degrees(math.atan2( -(y_start-((y_start+y_end)/2)) , x_start-((x_start+x_end)/2) )), 5)
        angle_2 = round(math.degrees(math.atan2( -(y_end - ((y_start+y_end)/2)) , x_end - ((x_start+x_end)/2) )), 5)
        # print('angle= ',angle_1,'or',angle_2)
        cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int(x_start),int(y_start)), (255, 0, 0), 2)
        cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int(x_end),int(y_end)), (0, 0, 255), 2)
        cv2.putText(outline_img,str(angle_1),(int(x_start),int(y_start)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 20, 0), 2, cv2.LINE_AA)
        cv2.putText(outline_img,str(angle_2),(int(x_end),int(y_end)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 20, 100), 2, cv2.LINE_AA)
        
        x_offset = ((x_start+x_end)/2)-(1280/2)
        y_offset = ((y_start+y_end)/2)-(720/2)
        if x_offset>0:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2)+200,int((y_start+y_end)/2)), (50, 255, 0), 3)
            cv2.putText(outline_img,'X['+str(x_offset)+']',(int((x_start+x_end)/2)+210,int((y_start+y_end)/2)-10),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2)-200,int((y_start+y_end)/2)), (50, 255, 0), 3)
            cv2.putText(outline_img,'X['+str(x_offset)+']',(int((x_start+x_end)/2)-210,int((y_start+y_end)/2)-10),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)

        if y_offset>0:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2),int((y_start+y_end)/2)+200), (50, 255, 0), 3)
            cv2.putText(outline_img,'Y['+str(y_offset)+']',(int((x_start+x_end)/2-10),int((y_start+y_end)/2)+220),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2),int((y_start+y_end)/2)-200), (50, 255, 0), 3)
            cv2.putText(outline_img,'Y['+str(y_offset)+']',(int((x_start+x_end)/2-10),int((y_start+y_end)/2)-220),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)


        # print(x_offset, y_offset)
        area = cv2.contourArea(cnt)

        return outline_img, angle_1, [x_offset,y_offset],area
    else:
        return orig_img,0,[0,0],0


'''
    手臂移動
    輸入: 模式(PTP or LINE)、 速度、 加速度
    輸出: 無
    更新全域變數: Point_now
'''
def arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting):
    global Point_now

    if mode=='PTP':
        PTP_Move(Point_count, speed, acceleration)
    else:
        LIN_Move(Point_count, speed, acceleration)
    
    Point_now = copy.deepcopy(Point_count)


'''
    PTP移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def PTP_Move(Point, speed = speed_setting, acceleration = acceleration_setting):
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.PTP(1, speed, acceleration, 0, 1, C_PTP_XYZ)
    modbus.Arm_State_REGISTERS()
    time.sleep(0.1)
    modbus.Arm_State_REGISTERS()
    modbus.Arm_State_REGISTERS()
    while 1:
        # frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        # frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
        time.sleep(0.01)


def Correction():
    global k_gauss,k_close,low_threshold,high_threshold

    


    while True:

        # 獲取數值
        k_gauss = cv2.getTrackbarPos('k_gauss','setting')
        k_close = cv2.getTrackbarPos('k_close','setting')
        low_threshold = cv2.getTrackbarPos('low_threshold','setting')
        high_threshold = cv2.getTrackbarPos('high_threshold','setting')

        # 過濾數值
        if k_gauss%2==0:
            k_gauss+=1


        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        gradient = Canny(color_image, k_gauss=k_gauss, low_threshold=low_threshold, high_threshold=high_threshold, k_close=k_close)
        outline_img,angle,center,area = detect_angle(color_image,gradient)

        gradient = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)
        gradient = cv2.resize(gradient,(640,360))
        color_image = cv2.resize(color_image,(640,360))
        outline_img = cv2.resize(outline_img,(640,360))

        vtitch = np.hstack((color_image, gradient))
        vtitch = np.hstack((vtitch, outline_img))
        cv2.imshow("setting", vtitch)

        # cv2.imshow('gradient', color_image)
        # cv2.imshow('gradient', gradient)

        key = cv2.waitKey(10)
        if key & 0xFF == ord('q') or key == 27:
            break

    
    # 對正校正底板
    while True:

        center_x_list = [0]*10
        center_y_list = [0]*10
        for i in range(10):
            frames = pipeline.wait_for_frames()
            image = frames.get_color_frame()
            image = np.asanyarray(image.get_data())

            gradient = Canny(image, k_gauss=k_gauss, low_threshold=low_threshold, high_threshold=high_threshold, k_close=k_close)
            outline_img,angle,center,area = detect_angle(image,gradient)
            center_x_list[i]=center[0]
            center_y_list[i]=center[1]
        center[0] = statistics.median(center_x_list)
        center[1] = statistics.median(center_y_list)
        # print(center)


        if abs(center[0])<=0.25 and abs(center[1])<=0.25:
            break

        if center[1]>5:
            Point_count[0]-=0.2
        elif center[1]>3:
            Point_count[0]-=0.1
        elif center[1]>0:
            Point_count[0]-=0.02
        elif center[1]<-5:
            Point_count[0]+=0.2
        elif center[1]<-3:
            Point_count[0]+=0.1
        elif center[1]<0:
            Point_count[0]+=0.02

        if center[0]>5:
            Point_count[1]-=0.2
        elif center[0]>3:
            Point_count[1]-=0.1
        elif center[0]>0:
            Point_count[1]-=0.02
        elif center[0]<-5:
            Point_count[1]+=0.2
        elif center[0]<-3:
            Point_count[1]+=0.1
        elif center[0]<0:
            Point_count[1]+=0.02

        
        arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
        print(center)
        # time.sleep(1)


        gradient = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)
        gradient = cv2.resize(gradient,(640,360))
        color_image = cv2.resize(color_image,(640,360))
        outline_img = cv2.resize(outline_img,(640,360))

        vtitch = np.hstack((color_image, gradient))
        vtitch = np.hstack((vtitch, outline_img))
        cv2.imshow("setting", vtitch)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            break



    # 開始校正
    while 1:
        area_list = [0]*10
        for i in range(10):
            frames = pipeline.wait_for_frames()
            image = frames.get_color_frame()
            image = np.asanyarray(image.get_data())

            gradient = Canny(image, k_gauss=k_gauss, low_threshold=low_threshold, high_threshold=high_threshold, k_close=k_close)
            outline_img,angle,center,area = detect_angle(image,gradient)
            area_list[i]=area
        area = statistics.median(area_list)
        # print(area-150000)
        # time.sleep(0.5)

        gradient = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)
        gradient = cv2.resize(gradient,(640,360))
        color_image = cv2.resize(color_image,(640,360))
        outline_img = cv2.resize(outline_img,(640,360))

        vtitch = np.hstack((color_image, gradient))
        vtitch = np.hstack((vtitch, outline_img))
        cv2.imshow("setting", vtitch)
        # cv2.waitKey(1)

        print('=============\n',area-target_area,'\n=============')
        

        if abs(area-target_area)<40:
            print(abs(area)-target_area)
            print("error_height= ",-150+(Point_now[2]-target_height)+4.71-2.87)
            break
        elif area-target_area>6000 and area-target_area>0:
            Point_count[2] += 1
            print(1)
        elif area-target_area<-6000 and area-target_area<0:
            Point_count[2] -= 1
            print(2)
        elif area-target_area>3000 and area-target_area>0:
            Point_count[2] += 0.5
            print(3)
        elif area-target_area<-3000 and area-target_area<0:
            Point_count[2] -= 0.5
            print(4)
        elif area-target_area>1000 and area-target_area>0:
            Point_count[2] += 0.1
            print(5)
        elif area-target_area<-1000 and area-target_area<0:
            Point_count[2] -= 0.1
            print(6)
        elif area-target_area>0 and area-target_area>0:
            Point_count[2] += 0.01
            print(7)
        else:
            Point_count[2] -= 0.01
            print(8)
        
        arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
        
        if cv2.waitKey(2) & 0xFF == ord('q'):
            break


if __name__=="__main__":
    # arm setting
    Arm_state = 0
    sucker_Port = 300       # 吸盤開關(D0)
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.LIN.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()
    modbus.DO(sucker_Port,0) # 1 -> on ; 0 -> off

    # -150+(Point_now[2]-target_height)+4.71
    while 1:
        num = input("Input(0:左下, 1:左上, 2:右下, 4:家, 5:Break):")
        if num == '0':
            Point_count = Point_basic[0]
            arm_move(mode='PTP', speed = 25, acceleration = 10)
            Correction()
            Point_saved[0] = copy.deepcopy(Point_count)
            base_z[0] = -150+(Point_now[2]-target_height)+4.71-2.87
        
        elif num == '1':
            Point_count = Point_basic[1]
            arm_move(mode='PTP', speed = 25, acceleration = 10)
            Correction()
            Point_saved[1] = copy.deepcopy(Point_count)
            base_z[1] = -150+(Point_now[2]-target_height)+4.71-2.87

        elif num == '2':
            Point_count = Point_basic[2]
            arm_move(mode='PTP', speed = 25, acceleration = 10)
            Correction()
            Point_saved[2] = copy.deepcopy(Point_count)
            base_z[2] = -150+(Point_now[2]-target_height)+4.71-2.87

        elif num == '4':
            Point_count = Point_basic[4]
            arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)

        elif num == '5':
            break

        else:
            #z
            print("z[0] = ", base_z[0])
            print("z[1] = ", base_z[1])
            print("z[2] = ", base_z[2])

            #point_saved[2]
            print("Point_saved[0] = ", Point_saved[0])
            print("Point_saved[1] = ", Point_saved[1])
            print("Point_saved[2] = ", Point_saved[2])

            #z
            print("\n=============================================")
            print("z = ", base_z[0])

            #pitch
            if(Point_saved[0][2] > Point_saved[1][2]):
                pitch_sin = (Point_saved[0][2] - Point_saved[1][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
                pitch = math.asin(pitch_sin)*(-1)
                print("pitch = ", math.degrees(pitch))
            else:
                pitch_sin = (Point_saved[1][2] - Point_saved[0][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
                pitch = math.asin(pitch_sin)
                print("pitch = ", math.degrees(pitch))

            #roll
            if(Point_saved[0][2] > Point_saved[2][2]):
                roll_sin = (Point_saved[0][2] - Point_saved[2][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
                roll = math.asin(roll_sin)
                print("roll = ", math.degrees(roll))
            else:
                roll_sin = (Point_saved[2][2] - Point_saved[0][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
                roll = math.asin(roll_sin)*(-1)
                print("roll = ", math.degrees(roll))

    #z
    print("z[0] = ", base_z[0])
    print("z[1] = ", base_z[1])
    print("z[2] = ", base_z[2])

    #point_saved[2]
    print("Point_saved[0] = ", Point_saved[0])
    print("Point_saved[1] = ", Point_saved[1])
    print("Point_saved[2] = ", Point_saved[2])

    #z
    print("\n=============================================")
    print("z = ", base_z[0])

    #pitch
    if(Point_saved[0][2] > Point_saved[1][2]):
        pitch_sin = (Point_saved[0][2] - Point_saved[1][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
        pitch = math.asin(pitch_sin)*(-1)
        print("pitch = ", math.degrees(pitch))
    else:
        pitch_sin = (Point_saved[1][2] - Point_saved[0][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
        pitch = math.asin(pitch_sin)
        print("pitch = ", math.degrees(pitch))

    #roll
    if(Point_saved[0][2] > Point_saved[2][2]):
        roll_sin = (Point_saved[0][2] - Point_saved[2][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
        roll = math.asin(roll_sin)
        print("roll = ", math.degrees(roll))
    else:
        roll_sin = (Point_saved[2][2] - Point_saved[0][2])/math.sqrt((Point_saved[0][0] - Point_saved[1][0])**2 + (Point_saved[0][1] - Point_saved[1][1])**2)
        roll = math.asin(roll_sin)*(-1)
        print("roll = ", math.degrees(roll))

    