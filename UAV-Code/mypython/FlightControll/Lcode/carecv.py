import socket
import time
import pickle
import numpy as np
import cv2
from Lgui import tk_gui, cv_draw 
import tkinter as tk
send_port = 3333
recv_port = 2887
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
recv_address = ("0.0.0.0", recv_port)
send_address = ("255.255.255.255", send_port)
server_socket.bind(recv_address)
server_socket.settimeout(0.1)
print("接收启动")
received_data = []
datalist=[[0,0]]
timecount=0
packagecount=0
lastxy=[0,0]
lastdata=[0,0]
totaldis=0
status=False
def main():
    global datalist,img,timecount,received_data,gui,lastxy,lastdata,packagecount,totaldis,status
    try:
        data, client_address = server_socket.recvfrom(1024)
        #print("接收到的数据是",data)
        realdata=pickle.loads(data)
        if realdata[0]==170 and realdata[4]==255:
            received_data=[realdata[1],realdata[2]]
            status=realdata[3]
            datalist.append(received_data)
            lastdata=received_data
            if realdata[1]==64 and realdata[2]==64 and realdata[3]==64:
                gui.output_text4.configure("任务已启动 跑")
            print("realdata x y",received_data)
    except:
        pass 
    if gui.task_id==1:
        if packagecount<3:
            senddata=[170,160,160,255]
            bydata=pickle.dumps(senddata)
            send_socket.sendto(bydata, send_address)
            packagecount+=1
        else:
            gui.task_id=-1
            packagecount=0
    elif gui.task_id==2:
        if packagecount<3:
            senddata=[170,160,161,255]
            bydata=pickle.dumps(senddata)
            send_socket.sendto(bydata, send_address)
            packagecount+=1
        else:
            gui.task_id=-1
            packagecount=0
    if gui.ready_to_go==True:
        if packagecount<3:
            senddata=[170,192,192,255]
            bydata=pickle.dumps(senddata)
            send_socket.sendto(bydata, send_address)
            packagecount+=1
        else:
            packagecount=0
            gui.ready_to_go=False
    if time.time()-timecount>1:
        distance=int(((lastdata[0]-lastxy[0])**2+(lastdata[1]-lastxy[1])**2)**0.5)
        totaldis+=distance
        lastxy=lastdata
        #text="distance:{}".format(str(distance))
        gui.output_text2.configure(text="distance: {}".format(totaldis))
        gui.output_text3.configure(text="x y: {}".format(received_data))
        img=draw.show_draw_res(datalist)
        timecount=time.time()
        gui.show_img(img)
    gui.root.after(50,main)
draw=cv_draw()
gui=tk_gui()
gui.output_text0=tk.Label(gui.root, text="", font=gui.label_font, anchor=tk.CENTER,fg="red")
gui.output_text0.pack(padx=10, pady=10, side=tk.BOTTOM)
gui.output_text0.configure(text='10cm/小格 80cm/大格')
gui.output_text2=tk.Label(gui.root, text="", font=gui.label_font, anchor=tk.CENTER,fg="red")
gui.output_text2.pack(padx=10, pady=10, side=tk.BOTTOM)
gui.output_text3=tk.Label(gui.root, text="", font=gui.label_font, anchor=tk.CENTER,fg="red")
gui.output_text3.pack(padx=10, pady=10, side=tk.BOTTOM)
gui.output_text4=tk.Label(gui.root, text="", font=gui.label_font, anchor=tk.CENTER,fg="red")
gui.output_text4.pack(padx=10, pady=10, side=tk.RIGHT)
gui.root.after(50,main)
gui.gui_window()
gui.root.mainloop()
