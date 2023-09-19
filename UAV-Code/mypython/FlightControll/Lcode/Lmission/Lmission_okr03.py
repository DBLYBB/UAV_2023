import threading
import time
import math
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side,lock,task_start_sign
from RadarDrivers_reconstruct.Radar import Radar
put_height=50
fly_height=120
threshold=3
radar=Radar()
radar.start('COM3','LD06')
radar.start_resolve_pose(size=1200)
class mission(object) :
        #左正右负，前正后负
        def __init__(self,fc_data:List[int],com_fc:List[int],com_gpio:List[int],gpio_data:List[int]) -> None:
            self.fc_data=fc_data
            self.com_fc=com_fc
            self.com_gpio=com_gpio
            self.gpio_data=gpio_data
            self.mission_step=0
            self.task_running=False
            self.time_count=0
            self.change_count=0
            self.bias=[0,0]
            self.self_pos = [0,0]
            self.target = [[130,491],[407,440],[252,288],[325,213],[99,139],[402,138],[174,213],[175,364],[328,365],[252,440],[100,288],[404,287],[248,139]]
            self.P1=[0,0]
            self.P2=[0,0]
        def run(self):
            self.task_running=True
            task_thread=threading.Thread(target=self.task)
            task_thread.daemon=True
            task_thread.start()
            self.mission_step=0
            logger.info("任务启动")
            pass
        def task(self):
            global put_height,fly_height
            while self.task_running==True :
                #print(radar.rt_pose)
                time.sleep(0.01)
                if task_start_sign.value==True :
                    if self.mission_step==0:
                        """ self.bias=[int(radar.rt_pose[0])-self.target[0][0],int(radar.rt_pose[1])-self.target[0][1]]
                        for i in range(len(self.target)):
                            self.target[i][0] += self.bias[0]
                            self.target[i][1] += self.bias[1] """
                        self.P1=self.target[self.gpio_data[1]]
                        self.P2=self.target[self.gpio_data[2]]
                        logger.info("取得偏置，设置点位，准备开始")
                        x_pid = PID(0,self.P1[0])
                        y_pid = PID(0,self.P1[1])
                        yaw_pid=PID(1,0)
                        logger.info("P1x:%s",self.P1[0])
                        logger.info("P1y:%s",self.P1[1])
                        logger.info("P1:%s",self.P1)
                        logger.info("P2:%s",self.P2)
                        self.time_count=time.time()
                        self.mission_step=1
                    elif self.mission_step==1:
                        if time.time()-self.time_count<1:
                            pass
                        else:
                            self.mission_step=2
                    elif self.mission_step==2:
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        if abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                            self.change_count=0
                        else:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                            self.change_count+=1
                        if self.change_count >10:
                            self.mission_step=3
                            self.change_count=0
                            self.time_count=time.time()
                            logger.info("到达P1，悬停开始")
                    elif self.mission_step==3:
                        if time.time()-self.time_count<20:
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                            self.gpio_set(1,0)
                            self.gpio_set(2,80)
                        else:
                            self.mission_step=4
                            self.time_count=time.time()
                            logger.info("悬停结束，恢复高度")
                    elif self.mission_step==4:
                        if time.time()-self.time_count<20:
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            self.gpio_set(1,1)
                            self.gpio_set(2,48)
                        else:
                            self.mission_step=5
                            logger.info("目标指定：P2 准备前进")
                            x_pid = PID(0,self.P2[0])
                            y_pid = PID(0,self.P2[1])
                    elif self.mission_step==5:
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.gpio_set(2,64)
                        if abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                            self.change_count=0
                        else:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                            self.change_count+=1
                        if self.change_count >10:
                            self.mission_step=6
                            self.change_count=0
                            self.time_count=time.time()
                            logger.info("到达P2，悬停开始")
                    elif self.mission_step==6:
                        if time.time()-self.time_count<20:
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                            self.gpio_set(1,0)
                            self.gpio_set(2,80)
                        else:
                            self.mission_step=7
                            self.time_count=time.time()
                            logger.info("悬停结束，恢复高度")
                    elif self.mission_step==7:
                        if time.time()-self.time_count<20:
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            self.gpio_set(1,1)
                            self.gpio_set(2,48)
                        else:
                            self.mission_step=8
                            logger.info("目标指定：起飞点 准备前进")
                            x_pid = PID(0,self.target[0][0])
                            y_pid = PID(0,self.target[0][1])
                    elif self.mission_step==8:
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.gpio_set(2,64)
                        if abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                        else:
                            self.mission_step=9
                            self.time_count=time.time()
                            logger.info("到达起飞点，降落")
                    elif self.mission_step==9:
                        if time.time()-self.time_count<5:
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                        else:
                            self.mission_step=10
                    else:
                        self.end()
            time.sleep(0.01)
        def gpio_set(self,gpion,value=0):
            self.com_gpio[gpion]=value
        def speed_set(self,x=0,y=0,yaw=0):
            self.com_fc[2]=x+sp_side
            self.com_fc[3]=y+sp_side
            self.com_fc[5]=yaw+sp_side
        def height_set(self,height):
            self.com_fc[4]=height
        def end(self):
            lock.acquire()
            self.com_fc[6]=101
            lock.release()
            logger.info("任务结束")
    
