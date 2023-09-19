import cv2
import time
import serial
import numpy as np
import RPi.GPIO as GPIO
from FlightController.Solutions.Vision_Net import FastestDetOnnx, FastestDet
from FlightController.Solutions.Vision import pid_speed


deep = FastestDetOnnx(drawOutput=False)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(32 , GPIO.OUT)

while True:
    img = cap.read()[1]
    if img is None:
        continue
    cv2.imshow("Origin", img)

    # 深度学习实例
    #get = deep.detect(img)
    #print(get)

    # 直线检测+返回PID速度
    speed =pid_speed(img , 6)
    print(speed)


    # GPIO操作实例
    #GPIO.output(32,not GPIO.input(32))

    time.sleep(0.1)


    #PID传入参数为识别坐标，传给飞控参数为delta_output，见PID,py

    #cv2.imshow("Result", img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord ("q"):
        break
cv2.destroyAllWindows()