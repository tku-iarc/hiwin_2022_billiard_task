'''
步驟一: 新增教導器BASE(1)
X = -250
Y = 300
Z = -158.26 (要+error_height)((-151.9))

步驟二: 執行此程式

步驟三: BASE的Z軸更新

'''

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

target_height = 60.330 # (壓克力板方格紙)
error_height = 0

# 手臂移動速度
speed_setting = 40
acceleration_setting = speed_setting

start_point = [-367.708, 12.711 ,293.500, 180.000, 0.000, 90.832]
# 目前座標
Point_now = [0, 368.000, 293.500, -180.000, 0, 90.000]            # 預設為家座標

# 計算座標(下一個點)
Point_count = [0, 368, 293.5, -180, 0, 90.000]          # 預設為家座標

# 基本點位(pixel2mm_maker使用相同數值)
Point_basic = [3.296, 393.233, -48.922, 180.000, 0, 90]        # 0: 左收集區中點 #原pitch-179.091



# 60.330


"""
    設定
"""
# 手臂控制so檔
so_file = "/home/oscaryang/hiwin_BilliardBall/src/libmodbus_ROS/src/My_test/Hiwin_API.so"
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



def Canny(orig_img):
    # 灰階
    # gray_img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    gauss_img = cv2.GaussianBlur(orig_img, (3, 3), 0)

    # Canny邊緣運算
    low_threshold = 1
    high_threshold = 120
    canny_img = cv2.Canny(gauss_img, low_threshold, high_threshold)

    # 閉運算(緩解Canny斷線問題)
    kernel = np.ones((15,15),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    return gradient


def detect_angle(orig_img, canny_img):
    outline_img = orig_img.copy()

    # 輪廓檢測(使用Canny檢測後影像繼續檢測)Point_now
    ret,thresh = cv2.threshold(canny_img,180,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    cnt = 0
    check = 0

    for i in range(len(contours)):
        if cv2.contourArea(contours[i])<300000 and cv2.contourArea(contours[i])>80000:
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
    Point_now = copy.deepcopy(Point_count)



'''
    PTP移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def PTP_Move(Point, speed = speed_setting, acceleration = acceleration_setting):
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.PTP(1, speed, acceleration, 1, 0, C_PTP_XYZ)
    modbus.Arm_State_REGISTERS()
    time.sleep(0.1)
    modbus.Arm_State_REGISTERS()
    modbus.Arm_State_REGISTERS()
    while 1:
        frames = pipeline.wait_for_frames()         # 不斷更新Realsense預防模糊
        frames.get_color_frame()                    # 同上
        if(modbus.Arm_State_REGISTERS() == 1):
            break
        time.sleep(0.01)




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

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    PTP_Move(start_point, speed = speed_setting, acceleration = acceleration_setting)
    PTP_Move(start_point, speed = speed_setting, acceleration = acceleration_setting)
    Point_count = Point_basic
    arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)


    # 對正校正底板
    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        cv2.line(color_image, (640, 0), (640, 720), (0, 0, 255), 1)
        cv2.line(color_image, (0, 360), (1280, 360), (0, 0, 255), 1)

        cv2.line(color_image, (960, 260), (960, 460), (0, 0, 255), 1)
        cv2.line(color_image, (320, 260), (320, 460), (0, 0, 255), 1)
        cv2.line(color_image, (540, 680), (740, 680), (0, 0, 255), 1)
        cv2.line(color_image, (540, 40), (740, 40), (0, 0, 255), 1)

        cv2.imshow('MyD415', color_image)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break



    # 開始校正
    while 1:

        area_list = [0]*10
        for i in range(10):
            frames = pipeline.wait_for_frames()
            image = frames.get_color_frame()
            image = np.asanyarray(image.get_data())

            gradient = Canny(image)
            outline_img,angle,center,area = detect_angle(image,gradient)
            area_list[i]=area
        area = statistics.median(area_list)
        print(area)

        cv2.imshow('gradient', gradient)
        cv2.imshow('outline_img', outline_img)


        if abs(area-150000)<130:
            print(abs(area)-150000)
            print("ss")
            break
        elif (area-150000)>0 :
            Point_count[2] += 1
            arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
        else:
            Point_count[2] -= 1
            arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)

        if cv2.waitKey(200) & 0xFF == ord('q'):
            break


        while 1:
            area_list = [0]*10
            for i in range(10):
                frames = pipeline.wait_for_frames()
                image = frames.get_color_frame()
                image = np.asanyarray(image.get_data())

                gradient = Canny(image)
                outline_img,angle,center,area = detect_angle(image,gradient)
                area_list[i]=area
            area = statistics.median(area_list)
            print(area)

            cv2.imshow('gradient', gradient)
            cv2.imshow('outline_img', outline_img)


            if abs(area-150000)<90:
                print(abs(area)-150000)
                print("ss")
                break
            elif (area-150000)>0 :
                Point_count[2] += 0.05
                arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
            else:
                Point_count[2] -= 0.05
                arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)

            while 1:
                area_list = [0]*10
                for i in range(10):
                    frames = pipeline.wait_for_frames()
                    image = frames.get_color_frame()
                    image = np.asanyarray(image.get_data())

                    gradient = Canny(image)
                    outline_img,angle,center,area = detect_angle(image,gradient)
                    area_list[i]=area
                area = statistics.median(area_list)
                print(area)

                cv2.imshow('gradient', gradient)
                cv2.imshow('outline_img', outline_img)


                if abs(area-150000)<60:
                    print(abs(area)-150000)
                    print("error_height= ",Point_now[2]-target_height)
                    break
                elif (area-150000)>0 :
                    Point_count[2] += 0.02
                    arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)
                else:
                    Point_count[2] -= 0.02
                    arm_move(mode='PTP', speed = speed_setting, acceleration = acceleration_setting)

        



        if cv2.waitKey(20) & 0xFF == ord('q'):
            break


cv2.destroyAllWindows()
