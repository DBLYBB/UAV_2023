import tkinter as tk
from PIL import Image, ImageTk
import time
import numpy as np
import cv2 
# 创建窗口对象
class tk_gui:
    task_id = -1
    ready_to_go = False
    root = tk.Tk()
    # 获取屏幕宽度和高度
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    # 设置窗口大小和位置
    root.geometry("%dx%d+0+0" % (screen_width, screen_height))
    # 设置窗口题
    root.title("全屏无边框窗口")
    # 隐藏窗口的边框和菜单栏
    root.overrideredirect(1)
    label=tk.Label(root)
    label.pack()

    def __init__(self):
        self.button_font = ("Arial", 12)
        self.label_font = ("Arial", 12)
        self.button_width = 20
        self.button_height = 8
        

    def button_clicked(self, button_id):
        if (1 <= button_id <= 2):
            self.task_id = button_id
        elif (button_id == 3):
            self.ready_to_go = True
        text = str(self.task_id) + " " + str(self.ready_to_go)
        self.output_text.configure(text=text)

    def gui_window(self):
        self.root.title("uav")
        # 创建按钮
        for i in range(2):
            button = tk.Button(
                self.root, text=f"task {i+1}", command=lambda button_id=i+1: self.button_clicked(button_id))
            button.config(font=self.button_font, width=self.button_width, height=self.button_height)
            button.pack(padx=10, pady=10, side=tk.RIGHT)
        button = tk.Button(
            self.root, text=f"setup", command=lambda button_id=3: self.button_clicked(button_id))
        button.config(font=self.button_font, width=self.button_width, height=self.button_height)
        button.pack(padx=10, pady=10, side=tk.RIGHT)
        # 创建一个文本输出框
        self.output_text = tk.Label(self.root, text="", font=self.label_font, anchor=tk.CENTER)
        self.output_text.pack(padx=10, pady=10, side=tk.BOTTOM)
        text = str(self.task_id) + " " + str(self.ready_to_go)
        self.output_text.configure(text=text)
    def show_img(self,img):
        img1=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo=ImageTk.PhotoImage(image=Image.fromarray(img1))
        self.label.config(image=photo)
        self.label.image=photo
        self.label.place(x=0, y=0)
        #self.label.after(1000, self.show_img, img)
        #print("更新")
        #cv2.imshow("img",img)
    def start(self):
        self.gui_window()
        self.root.mainloop()

class cv_draw:
    def __init__(self):
        self.width = 1200
        self.height = 1200

    def draw(self, points):
        image = cv2.imread('base.png')
        #image=img.resize((480,400))
        for i in range(len(points)):
            x =( -points[i][1] + 35)
            y =( -points[i][0] + 365)
            cv2.circle(image, (x,y), 2, (0, 0, 255), -1)
        return image
    
    def show_draw_res(self,arr):
        res = self.draw(arr)
        #cv2.imshow("route", res)
        #cv2.waitKey(1)
        return res
