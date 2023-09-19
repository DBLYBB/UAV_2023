import tkinter as tk
import time
import numpy as np
import cv2 

class tk_gui:
    task_id = -1
    ready_to_go = False
    root = tk.Tk()
    window_width = 300
    window_height = 300
    root.geometry(f"{window_width}x{window_height}")

    def __init__(self):
        self.button_font = ("Arial", 12)
        self.label_font = ("Arial", 12)
        self.button_width = 10
        self.button_height = 2

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
            button.pack(padx=10, pady=10, side=tk.TOP)
        button = tk.Button(
            self.root, text=f"setup", command=lambda button_id=3: self.button_clicked(button_id))
        button.config(font=self.button_font, width=self.button_width, height=self.button_height)
        button.pack(padx=10, pady=10, side=tk.TOP)
        # 创建一个文本输出框
        self.output_text = tk.Label(self.root, text="", font=self.label_font, anchor=tk.CENTER)
        self.output_text.pack(padx=10, pady=10, side=tk.TOP)
        text = str(self.task_id) + " " + str(self.ready_to_go)
        self.output_text.configure(text=text)


class cv_draw:
    def __init__(self):
        self.width = 1200
        self.height = 1200

    def draw(self, points):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for i in range(len(points)):
            x =( -points[i][1] + self.width // 2)
            y =( -points[i][0] + self.height // 2)
            cv2.circle(image, (x,y), 2, (255, 255, 255), -1)
        return image
    
    def show_draw_res(self,arr):
        res = self.draw(arr)
        #cv2.imshow("route", res)
        #cv2.waitKey(1)
        return res

""" test = tk_gui()
test.gui_window()
test.root.mainloop()
 """