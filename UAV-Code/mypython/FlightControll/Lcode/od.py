import cv2
from Lcode.Logger import logger
import threading
from RadarDrivers_reconstruct.Radar import Radar
from Lcode.global_variable import lock
import time
import Vision.Vision
import numpy as np 
class cv_class(object):
    def __init__(self,width=640,height=480) -> None:
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap_running=False
        self.qr_scanning=False
        self.retcount=0
        self.qr_scan_complete=False
        self.result=""
        self.last_time=0
        self.task_list1=[]
        self.task_list2=[]
        self.red_mask=[156,43,46,180,255,255]
        self.green_mask=[35,80,55,90,255,255]
        self.blue_mask=[100,80,55,124,255,255]
        self.xycenter=[0,0]
        self.color_detecting=False
        self.colornow=1
        #红绿蓝 1 2 3
    def run(self):
        self.cap_running=True
        cap_thread=threading.Thread(target=self.cap_thread,args=())
        cap_thread.daemon=True
        cap_thread.start()
        logger.info("摄像头启动")
    def cap_thread(self):
        while self.cap_running==True:
            img=self.cap.read()[1]
            if img is None:
                logger.error("摄像头读取失败")
                continue
            elif self.qr_scanning==True:
                ret=Vision.find_QRcode_zbar(img)[0]
                if ret==True:
                    self.retcount+=1
                else:
                    self.retcount=0
                if self.retcount>10:
                    self.result=Vision.find_QRcode_zbar(img)[3]
                    self.qr_scan_complete=True
                    logger.info("二维码识别完成 结果为"+self.result)
                    self.task_list1,self.task_list2=self.extract_numbers(self.result)
                if time.time()-self.last_time>0.2:
                    logger.info("二维码识别中")
                    self.last_time=time.time()
            elif self.color_detecting==True:
                self.xycenter=self.color_detect(img,self.colornow)
            cv2.waitKey(10)
    def stop(self):
        self.cap_running=False
        logger.info("摄像头关闭")
    #目前缺少颜色识别
    def extract_numbers(input_string):
    # 从输入字符串中获取加号前后的三个数字
        numbers = input_string.split("+")
        num1 = numbers[0][-3:]
        num2 = numbers[1][:3]

        # 将数字转换为列表形式
        list1 = [int(d) for d in num1]
        list2 = [int(d) for d in num2]

        return list1, list2
    def color_detect(self,img,color):
        if color==1:
            mask=self.red_mask
        elif color==2:
            mask=self.green_mask
        elif color==3:
            mask=self.blue_mask
        pcs1=Vision.color_filter(img,mask[0],mask[1],mask[2],mask[3],mask[4],mask[5])
        kernel1 = np.ones((9,9),np.uint8)
        dilated_img = cv2.dilate(pcs1, kernel1, iterations=1)
        kernel2 = np.ones((6,6),np.uint8)
        eroded_img = cv2.erode(dilated_img, kernel2, iterations=1)
        dilated_img = cv2.dilate(eroded_img, kernel1, iterations=1)
        eroded_img = cv2.erode(dilated_img, kernel2, iterations=1)
        gray = cv2.cvtColor(eroded_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                if  area > 450:
                    M = cv2.moments(max_contour)
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    cv2.drawContours(eroded_img, [max_contour], -1, (0, 255, 0), 2)
                    print(max_contour)
        cv2.imshow('origin', img)
        cv2.imshow('eroded_img', eroded_img)
        return center_x,center_y
