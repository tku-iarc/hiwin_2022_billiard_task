from logging import root, shutdown
import tkinter as tk
from _tkinter import *
import os, sys
from turtle import down, left, right
from unicodedata import name

go_point = [2.687, 395.521, 372.013, 180.000, 0, 90.736]

def go_up(event):
    global go_point
    up = round(0.199,2)

    go_point[1] += up
    print("y + 0.2, go_poing = ", go_point)

def go_down(event):
    global go_point
    down = round(0.199,2)

    go_point[1] -= down
    print("y - 0.2, go_poing = ", go_point)

def go_left(event):
    global go_point
    left = round(0.199,2)

    go_point[0] -= left
    print("x - 0.2, go_poing = ", go_point)

def go_right(event):
    global go_point
    right = round(0.199,2)

    go_point[0] += right
    print("x + 0.2, go_poing = ", go_point)

def pprint():
    print("pic_point = ",go_point)

def shutdownbutton(event):
    os._exit(os.EX_OK)   

def hello():
    process = tk.Toplevel(main_menu)
    process.title('測試程式')
    process.geometry('300x300')
    process.configure(background = 'grey')

    up_button = tk.Button(process, text='y + 0.2', bg='grey', font=('Arial',10))
    up_button.place(x=0, y=0)
    up_button.bind("<Button>", go_up)

    down_button = tk.Button(process, text='y - 0.2', bg='grey', font=('Arial',10))
    down_button.place(x=70, y=0)
    down_button.bind("<Button>", go_down)

    left_button = tk.Button(process, text='x - 0.2', bg='grey', font=('Arial',10))
    left_button.place(x=140, y=0)
    left_button.bind("<Button>", go_left)

    right_button = tk.Button(process, text='x - 0.2', bg='grey', font=('Arial',10))
    right_button.place(x=210, y=0)
    right_button.bind("<Button>", go_right)

    test_shutdown = tk.Button(process, text='關閉程式', bg='red', font=('Arial',15), command=process.destroy)
    test_shutdown.place(x=0, y=100)


if __name__ == '__main__':
    main_menu = tk.Tk()
    main_menu.title('主控')
    main_menu.geometry('1280x720')
    main_menu.configure(background = 'grey')

    test = tk.Button(text="測試程式", bg="green", font=('Arial',10), width=30, height=35, command=hello)
    test.place(x=0,y=0)
    main = tk.Button(text="主程式", bg="yellow", font=('Arial',10), width=30, height=35, command=pprint)
    main.place(x=400,y=0)
    main_shutdown = tk.Button(text='關閉程式', bg='red', font=('Arial',10), height=30, width=35)
    main_shutdown.place(x=800, y=0)
    main_shutdown.bind("<Button-1>",shutdownbutton)

    main_menu.mainloop()