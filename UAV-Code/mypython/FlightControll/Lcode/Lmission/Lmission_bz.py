import threading
import time
from Lcode.od import cv_cap
from typing import List
from Lcode.Logger import logger
from Lcode.global_variable import sp_side,lock
import math
import simple_pid
from RadarDrivers_reconstruct.Radar import Radar
from yolo_v2.get_yolo_res import yolo_det 
import cv2
import os
cap = cv2.VideoCapture(0)
yolo=yolo_det()
radar=Radar()
count_time=0
img=None
yolo_res=None
point1,point2=None,None
radar_sign=0
color_sign=0
class mission(object) :
        def __init__(self,fc_data:List[int],com_fc:List[int],com_gpio:List[int]) -> None:
            self.fc_data=fc_data
            self.com_fc=com_fc
            self.com_gpio=com_gpio
            self.mission_step=0
            self.task_running=False
            self.change_count=0
        def run(self):
            self.task_running=True
            task_thread=threading.Thread(target=self.task)
            task_thread.daemon=True
            task_thread.start()
            self.mission_step=0
            logger.info("任务启动")
            pass
        def task(self):
            global count_time,point1,point2,radar_sign,last_angle,color_sign
            while self.task_running==True :
                self.fc_mission_step = self.fc_data[2]
                if self.fc_mission_step ==0x05 :
                    if radar_sign==1:
                        try:
                            point1,point2=radar.find_obstacles_with_filter()
                        except:
                            logger.info("雷达丢失")
                            pass
                    if self.mission_step==0:
                        count_time=time.time()
                        logger.info("树莓派任务阶段0 摄像头和雷达启动")
                        radar.start('COM3', 'LD06')
                        radar_sign=1
                        self.mission_step+=1
                        pass
                    elif self.mission_step==1:
                        if (time.time()- count_time)<3 or point1==None or point2==None :
                             self.speed_set(0,0,0,0)
                        else:
                             logger.info("发现杆")
                             self.gpio_set(7,128)
                             self.mission_step+=1
                             target_point=self.find_closer(point1,point2)
                             angle=self.yaw_calculate(target_point)
                             speed_yaw=self.yaw_pid(angle)
                             count_time=time.time()
                             self.change_count=0
                    elif self.mission_step==2:
                        self.gpio_set(0,128)
                        if abs(angle)>10:
                             target_point=self.find_closer(point1,point2)
                             angle=self.yaw_calculate(target_point)
                             logger.info(angle)
                             self.speed_set(0,0,0,self.yaw_pid(angle))
                        else:
                             self.change_count+=1
                        if self.change_count>10:
                            self.mission_step+=1
                            target_point=self.find_closer(point1,point2)
                            distance=self.distance_calculate(target_point)
                            angle=self.yaw_calculate(target_point)
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                            count_time=time.time()
                            logger.info("开始识别杆颜色")
                    elif self.mission_step==3:
                        if time.time()-count_time<5:
                            ret, frame = cap.read()
                        else:
                            file_path = os.path.join(os.getcwd(), 'test', 'img_2.jpg')  # 在当前工作目录下的 test 文件夹中保存图像
                            cv2.imwrite(file_path, frame)
                            self.mission_step+=1
                            yolo_res=yolo.runtime()
                            color_sign=yolo_res[0][0]
                            logger.info("颜色为:%d",color_sign)
                            if color_sign==1:
                                self.gpio_set(2,1)
                                self.gpio_set(7,128)
                            else :
                                self.gpio_set(1,1)
                                self.gpio_set(7,128)
                    elif self.mission_step==4:
                        target_point=self.find_closer(point1,point2)
                        distance=self.distance_calculate(target_point)
                        angle=self.yaw_calculate(target_point)
                        if color_sign==1:
                                self.gpio_set(2,1)
                                self.gpio_set(7,128)
                        else :
                                self.gpio_set(1,1)
                                self.gpio_set(7,128)
                        logger.info("前进")
                        logger.info(distance)
                        if distance>75:
                            self.speed_set(10,0,0,self.yaw_pid(angle))
                            self.change_count=0
                        else:
                            self.change_count+=1
                        if self.change_count>5:
                            self.mission_step+=1
                            target_point=self.find_closer(point1,point2)
                            angle=self.yaw_calculate(target_point)
                            speed_yaw=self.yaw_pid(angle)
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                            count_time=time.time()
                            self.change_count=0
                            logger.info("进入绕圈 现在是流程4")
                            self.gpio_set(2,0)
                            self.gpio_set(1,0)
                            self.gpio_set(7,0)
                    elif self.mission_step==5:
                        if (time.time()- count_time)<18:
                            target_point=self.find_closer(point1,point2)
                            angle=self.yaw_calculate(target_point)
                            speed_yaw=self.circle_yaw_pid(angle)
                            logger.info(angle)
                            if color_sign==0 or color_sign==1:
                                self.speed_set(self.circle_pid(target_point),15,0,speed_yaw)
                            else:
                                self.speed_set(self.circle_pid(target_point),-15,0,speed_yaw)
                        else:
                            self.mission_step+=1
                            self.speed_set(0,0,0,0)
                            while math.sqrt((target_point[0]-500)**2+(target_point[1]-500)**2)<130:
                                try:
                                    target_point=self.find_farther(point1,point2)
                                    point1,point2=radar.find_obstacles_with_filter()
                                except:
                                    pass
                            distance=self.distance_calculate(target_point)
                            angle=self.yaw_calculate(target_point)
                            last_far=target_point
                            if color_sign==2:
                                self.gpio_set(2,1)
                                self.gpio_set(7,128)
                            else :
                                self.gpio_set(1,1)
                                self.gpio_set(7,128)
                            logger.info("进入流程5  寻找杆2")
                    elif self.mission_step==6:
                        target_point=self.find_farther(point1,point2)
                        if math.sqrt((target_point[0]-500)**2+(target_point[1]-500)**2)>100:
                            last_far=target_point
                        else:
                            target_point=last_far
                        angle=self.yaw_calculate(target_point)
                        if abs(angle)>7:
                             logger.info(angle)
                             self.speed_set(0,0,0,self.yaw_pid(angle))
                             self.change_count=0
                        else:
                             self.change_count+=1
                        if self.change_count>10:
                            self.mission_step+=1
                            self.speed_set(0,0,0,0)
                            logger.info("进入流程6 前往杆2 ")
                            last_angle=0
                            count_time=time.time()
                    elif self.mission_step==7:
                        if(time.time()-count_time)<5:
                            self.speed_set(20,0,0,0)
                        else:
                            self.mission_step+=1
                            self.change_count=0
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                            target_point=self.find_closer(point1,point2)
                            distance=self.distance_calculate(target_point)
                            angle=self.yaw_calculate(target_point)
                            logger.info("进入流程7 切换杆 ")
                        '''
                        target_point=self.find_farther(point1,point2)
                        distance=self.distance_calculate(target_point)
                        angle=self.yaw_calculate(target_point)
                        logger.info(distance)
                        if abs(angle-last_angle)<120:
                            self.speed_set(10,0,0,self.yaw_pid(angle))
                            self.change_count
                            last_angle=angle
                        else:
                            logger.info("超出")
                            self.change_count+=1
                        if self.change_count>10:
                            self.mission_step+=1
                            self.change_count=0
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                            target_point=self.find_closer(point1,point2)
                            distance=self.distance_calculate(target_point)
                            angle=self.yaw_calculate(target_point)
                            logger.info("进入流程7 切换杆 ")
                        '''
                    elif self.mission_step==8:
                        target_point=self.find_closer(point1,point2)
                        distance=self.distance_calculate(target_point)
                        angle=self.yaw_calculate(target_point)
                        if distance>80:
                            self.speed_set(10,0,0,self.yaw_pid(angle))
                        else:
                            self.change_count+=1
                        if self.change_count>5:
                            self.mission_step+=1
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                            count_time=time.time()
                            self.change_count=0
                            logger.info("进入绕圈")
                    elif self.mission_step==9:
                        target_point=self.find_closer(point1,point2)
                        distance=self.distance_calculate(target_point)
                        angle=self.yaw_calculate(target_point)
                        if (time.time()- count_time)<20:
                            target_point=self.find_closer(point1,point2)
                            angle=self.yaw_calculate(target_point)
                            speed_yaw=self.circle_yaw_pid(angle)
                            if color_sign==0 or color_sign==1:
                                self.speed_set(self.circle_pid(target_point),-15,0,speed_yaw)
                            else:
                                self.speed_set(self.circle_pid(target_point),15,0,speed_yaw)
                        else:
                            self.mission_step+=1
                            self.speed_set(0,0,0,self.yaw_pid(angle))
                    else:
                         self.end()
                        #雷达寻找杆，等待返回
                time.sleep(0.01)
        def gpio_set(self,gpion,value=0):
            self.com_gpio[gpion]=value
        def speed_set(self,x=0,y=0,z=0,yaw=0):
            self.com_fc[2]=x+sp_side
            self.com_fc[3]=y+sp_side
            self.com_fc[4]=z+sp_side
            self.com_fc[5]=yaw+sp_side
        def end(self):
            lock.acquire()
            self.com_fc[6]=1
            lock.release()
            logger.info("任务结束")
        def find_closer(self,point1,point2):
            distance1=(point1[0]-500)**2+(point1[1]-500)**2
            distance2=(point2[0]-500)**2+(point2[1]-500)**2
            if distance1<distance2:
                return point1
            else:
                return point2
        def find_farther(self,point1,point2):
            distance1=(point1[0]-500)**2+(point1[1]-500)**2
            distance2=(point2[0]-500)**2+(point2[1]-500)**2
            if distance1>distance2:
                if distance1>120000:
                    return point2
                else:
                    return point1
            else:
                if distance2>120000:
                    return point1
                else:
                    return point2
        def find_closer_kai(self,point1,point2):
            distance1=(point1[0]-500)**2+(point1[1]-500)**2
            distance2=(point2[0]-500)**2+(point2[1]-500)**2
            if distance1<distance2:
                if distance1<32000:
                    return point2
                else:
                    return point1
            else:
                if distance2<32000:
                    return point1
                else:
                    return point2
        def yaw_calculate(self,point):
            long_side=math.sqrt((point[0]-500)**2+(point[1]-500)**2)
            if long_side !=0:
                cos=-(point[1]-500)/long_side
                if point[0]>500:
                    return math.degrees(math.acos(cos))
                else:
                    return -math.degrees(math.acos(cos))
            else:
                return 0
        def yaw_pid(self,angle):
            pid=simple_pid.PID(0.5,0.25,0,setpoint=0)
            pid.output_limits=(-20,20)
            return int(pid(angle))
        def circle_yaw_pid(self,angle):
            pid=simple_pid.PID(3,1,0,setpoint=0)
            pid.output_limits=(-30,30)
            return int(pid(angle))
        def distance_calculate(self,point):
            distance=math.sqrt((point[0]-500)**2+(point[1]-500)**2)
            return distance
        def circle_pid(slef,point):
            pid=simple_pid.PID(1,0.3,0,setpoint=70)
            pid.output_limits=(-20,20)
            return -int(pid(math.sqrt((point[0]-500)**2+(point[1]-500)**2)))