import numpy as np
import cv2

from logging import root, shutdown
import tkinter as tk
from _tkinter import *
import os, sys
from unicodedata import name

def HoughCircles():

    
    img=cv2.imread('/home/oscaryang/hough/Pic_height_12.jpg')
    print("img in")

    ball_imformation = [[-999 for i in range(3)] for j in range(16)]
    i = 0
    #載入並顯示圖片
    cv2.imshow('1',img)
    cv2.waitKey(0)
    #降噪（模糊處理用來減少瑕疵點）
    img_smooth = cv2.medianBlur(img,9)
    cv2.imshow('img_smooth',img_smooth)
    cv2.waitKey(0)
    # 灰度化,就是去色（類似老式照片）
    gray=cv2.cvtColor(img_smooth,cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray',gray)
    cv2.waitKey(0)

    #param1的具體實現，用於邊緣檢測    
    canny = cv2.Canny(gray, 15, 45)   
    cv2.imshow('canny', canny) 
    cv2.waitKey(0) 


    # hsv = cv2.cvtColor(img_smooth,cv2.COLOR_BGR2HSV)
    # lower = np.array([35,43,46])   #濾掉綠色
    # upper = np.array([100,255,255])
    # mask = cv2.inRange(hsv,lower,upper)
    # cv2.imshow('2',mask)
    # cv2.waitKey(0)

    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny, cv2.MORPH_GRADIENT, kernel)
    cv2.imshow('3',gradient)
    cv2.waitKey(0)


    #霍夫變換圓檢測
    circles= cv2.HoughCircles(gradient,cv2.HOUGH_GRADIENT,5,42,param1=50,param2=50,minRadius=31,maxRadius=31)
    #輸出返回值，方便查看類型
    print(circles)

    #輸出檢測到圓的個數
    print(len(circles[0]))

    circles = np.uint16(np.around(circles))

    print('-------------我是條分割線-----------------')
    #根據檢測到圓的信息，畫出每一個圓
    for circle in circles[0]:
        #圓的基本信息
        print(circle[2])
        #坐標行列(就是圓心)
        x=int(circle[0])
        y=int(circle[1])
        #半徑
        r=int(circle[2])
        #在原圖用指定顏色圈出圓，參數設定為int所以圈畫存在誤差
        cv2.circle(img,(x,y),r,(0,0,255),1,8,0)
        cv2.circle(img,(x,y),5,(0,0,0),-1)
        ball_imformation[i] = [int(circle[0]), int(circle[1]), circle[2]]
        i += 1 
    #顯示新圖像
    cv2.imshow('final',img)


    #按任意鍵退出
    cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return ball_imformation

def shutdownbutton(event):
    os._exit(os.EX_OK)   

def hello():
    process = tk.Toplevel(main_menu)
    process.title('測試程式')
    process.geometry('1280x720')
    process.configure(background = 'grey')

    run_file = tk.Button(process, text='執行', bg='green', font=('Arial,20'), height=30, width=35, command=HoughCircles)
    run_file.place(x=0, y=0)
    myinput = tk.Entry(process)
    myinput.place(x=400, y=0)
    test_shutdown = tk.Button(process, text='關閉程式', bg='red', font=('Arial,20'), height=30, width=35, command=process.destroy)
    test_shutdown.place(x=800, y=0)


if __name__ == '__main__':
    main_menu = tk.Tk()
    main_menu.title('主控')
    main_menu.geometry('1280x720')
    main_menu.configure(background = 'grey')

    test = tk.Button(text="測試程式", bg="green", font=('Arial,20'), width=30, height=35, command=hello)
    test.place(x=0,y=0)
    main = tk.Button(text="主程式", bg="yellow", font=('Arial,20'), width=30, height=35)
    main.place(x=400,y=0)
    main_shutdown = tk.Button(text='關閉程式', bg='red', font=('Arial,20'), height=30, width=35)
    main_shutdown.place(x=800, y=0)
    main_shutdown.bind("<Button-1>",shutdownbutton)

    main_menu.mainloop()
