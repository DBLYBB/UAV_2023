import serial
import cv2
import numpy as ny
import time
import binascii
import struct
import RPi.GPIO as GPIO
from FlightController.Solutions.Vision_Net import FastestDetOnnx, FastestDet
from FlightController.Solutions.Vision import pid_speed
GPIO.setmode(GPIO.BOARD)  # BOARD编号方式，基于插座引脚编号
GPIO.setwarnings(False)
GPIO.setup(32, GPIO.OUT)
ser0 = serial.Serial('/dev/ttyAMA0', 460800)
if ser0.isOpen == False:
    ser0.open()
deep = FastestDetOnnx(drawOutput=False)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640.0)
sp_side=21
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
com = [170, 0, sp_side, sp_side, sp_side, sp_side, 0, 255]  # AA,MODE,X,Y,Z,YAW,NEXT,FF
stage = 0
change32_sign = 0
receive_data = [0,0,0,0,0]
last_trigger_time = 0
laser_time = 0
laser_sign = 0
speed_xy=0
speed_yaw=0
while True:
    # 发送数据
    img = cap.read()[1]
    if img is None:
        continue
    if (laser_sign == 1) & (time.time() - laser_time > 1):
        GPIO.output(32, not GPIO.input(32))
        laser_time = time.time()
    # 每次循环等待0.01秒
    for value in com:
        hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
        ser0.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口0
    print("Com: ", com)
    # 读取数据
    byte_data = ser0.read()
    if byte_data == b'\xaa':
        # 读取接下来的四个字节数据
        response = ser0.read(4)
        # 判断数据是否符合通信协议，即以0xFF结尾
        if response[3] == 0xff:
            # 将读取到的五个字节数据存入receive_data数组中
            receive_data = list(byte_data + response)
            # 打印数组receive_data
            print("Received data: ", receive_data)
    if (receive_data[3] == 1) & (change32_sign == 0) & (time.time() - last_trigger_time > 0.06):
                stage = stage + 0x01
                change32_sign = 1
    # 如果派阶段和32阶段不同，发送命令让32切换阶段
    if (change32_sign == 1) & (stage != receive_data[2]):
        com[6] = 1
    else:
        change32_sign = 0
        com[6] = 0
    if receive_data[2] == 0x08 :
        laser_sign = 1
        speed_xy, speed_yaw =pid_speed(img,8)
        com[2] = int(speed_xy)+sp_side
        com[3] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x09 :
        laser_sign = 1
        speed_xy, speed_yaw =pid_speed(img,6)
        com[3] = int(speed_xy)+sp_side
        com[2] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x0A :
        laser_sign = 1
        speed_xy, speed_yaw = pid_speed(img,6)
        com[3] = int(speed_xy)+sp_side
        com[2] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x0B :
        speed_xy, speed_yaw = pid_speed(img,2)
        com[2] = int(speed_xy)+sp_side
        com[3] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x0C  :
        com[2] = sp_side
        com[3] = sp_side
        com[4] = sp_side
        com[5] = sp_side
        '''speed_xy, speed_yaw = pid_speed(img, 6)
        com[3] = int(speed_xy)+sp_side
        com[2] = sp_side
        com[5] = int(speed_yaw)+sp_side'''
    if receive_data[2] == 0x0D :
        speed_xy, speed_yaw = pid_speed(img, 8)
        com[2] = int(speed_xy)+sp_side
        com[3] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x0E :
        speed_xy, speed_yaw = pid_speed(img, 6)
        com[3] = int(speed_xy)+sp_side
        com[2] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x0F:
        speed_xy, speed_yaw = pid_speed(img, 2)
        com[2] = int(speed_xy)+sp_side
        com[3] = sp_side
        com[5] = int(speed_yaw)+sp_side
    if receive_data[2] == 0x10:
        com[2] = sp_side
        com[3] = sp_side
        com[4] = sp_side
        com[5] = sp_side
        '''speed_xy, speed_yaw = pid_speed(img, 4)
        com[3] = int(speed_xy)+sp_side
        com[2] = sp_side
        com[5] = int(speed_yaw)+sp_side'''
    if receive_data[2] == 0x11:
        speed_xy, speed_yaw = pid_speed(img, 2)
        com[2] = int(speed_xy)+sp_side
        com[3] = sp_side
        com[5] = int(speed_yaw)+sp_side
        print(speed_xy,speed_yaw)
        print(com[2],com[3])
    if receive_data[2] == 0x12:
        com[2] = sp_side
        com[3] = sp_side
        com[4] = sp_side
        com[5] = sp_side
        laser_sign = 0
    if receive_data[2] == 0x13:
        com[2] = sp_side
        com[3] = sp_side
        com[4] = sp_side
        com[5] = sp_side
        
    if receive_data[2] == 0x14:
        com[6] = 1
        #神经网络
    k = cv2.waitKey(1) & 0xFF
    if k == ord ("q"):
        break
