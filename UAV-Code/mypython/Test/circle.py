import cv2 
import time
import numpy as np
from VisionLib import black_line,vision_debug,color_recognition
from typing import List, Optional, Tuple, Union
from PIL import Image 
import serial
import time
import binascii
mode=1
move_x="00"
move_y="00"
move_yaw="00"
mode=0x01
ser0 = serial.Serial('/dev/ttyAMA0',460800)
if ser0.isOpen  ==False:
    ser0.open()
if __name__ == '__main__':
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    LOWER = np.array([0, 0, 0])
    UPPER = np.array([100, 100 ,100])
    red=0
    t=0
    while True:
        ret,frame = cap.read()
        result=color_recognition(frame)
        print(result)
        if result == "red" :
            t=t+10
        else :
            t=t-10
        if t >=50 :
            red=1
        if t<=20 :
            red=0
        if t>=80:
            t=80
        if t<=0 :
            t=0
        print(red)
            