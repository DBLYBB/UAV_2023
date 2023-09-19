import threading
import time
import math
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side,lock,task_start_sign
from Lcode.Lprotocol import udp_terminal
from RadarDrivers_reconstruct.Radar import Radar
threshold=5
put_height=50
fly_height=120
terminal=udp_terminal()
terminal.listen_start()
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
            self.x_intergral_base=0
            self.y_intergral_base=0
            self.nowpoint=0
            self.nowtargetlist=[]
            self.target=[[130,491],[407,440],[252,288],[325,213],[99,139],[402,138],[174,213],[175,364],[328,365],[252,440],[100,288],[404,287],[248,139]]
        def run(self):
            self.task_running=True
            task_thread=threading.Thread(target=self.task)
            task_thread.daemon=True
            task_thread.start()
            self.mission_step=0
            logger.info("任务启动")
            pass
        def task(self):
            global put_height,fly_height,threshold
            while self.task_running==True :
                if terminal.takeoff_sign==True:
                    terminal.food_all_take=True
                    self.fc_take_off()
                    self.nowtargetlist=terminal.task_list
                    terminal.takeoff_sign=False
                if terminal.food_all_take==False:
                    if self.mission_step>2:
                        self.height_set(fly_height)
                        x_pid = PID(0,self.target[5][0])
                        y_pid = PID(0,self.target[5][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        x_pid = PID(0,self.target[1][0])
                        y_pid = PID(0,self.target[1][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        x_pid = PID(0,self.target[0][0])
                        y_pid = PID(0,self.target[0][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.fc_fall()
                        self.mission_step=0
                        terminal.food_all_take=True
                    elif self.mission_step==2:
                        x_pid = PID(0,self.target[1][0])
                        y_pid = PID(0,self.target[1][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        x_pid = PID(0,self.target[0][0])
                        y_pid = PID(0,self.target[0][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.fc_fall()
                        self.mission_step=0
                        terminal.food_all_take=True
                    elif self.mission_step<=1:
                        x_pid = PID(0,self.target[0][0])
                        y_pid = PID(0,self.target[0][1])
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        while abs(xsp)>threshold or abs(ysp)>threshold:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(put_height)
                            xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                            ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                            yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.fc_fall()
                        self.mission_step=0
                        terminal.food_all_take=True

                if task_start_sign.value==True and terminal.food_all_take==True:#32进入程控状态，任务开始
                    if self.mission_step==0:
                        x_pid = PID(0,self.target[1][0])
                        y_pid = PID(0,self.target[1][1])
                        yaw_pid=PID(1,0)
                        self.mission_step=1
                    elif self.mission_step==1:
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
                            self.mission_step=2
                            self.change_count=0
                            x_pid = PID(0,self.target[5][0])
                            y_pid = PID(0,self.target[5][1])
                            logger.info("到达1,准备前往5")
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
                            logger.info("到达5,飞行状态2,准备开始送餐")
                            x_pid = PID(0,self.target[terminal.task_list[self.nowpoint]][0])
                            y_pid = PID(0,self.target[terminal.task_list[self.nowpoint]][1])
                    elif self.mission_step==3:
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
                        else:
                            self.speed_set(xsp,ysp,-yawsp)
                            self.height_set(fly_height)
                            logger.info("X速度为%d",xsp)
                            logger.info("Y速度为%d",ysp)
                            logger.info("YAW速度为%d",yawsp)
                            logger.info(radar.rt_pose)
                            if terminal.fall_sign ==True:
                                self.mission_step=4
                                terminal.fall_sign=False
                                logger.info("开始下降")
                                self.height_set(put_height)
                    elif self.mission_step==4:
                        xsp=x_pid.get_pid((int(radar.rt_pose[0])))
                        ysp=y_pid.get_pid((int(radar.rt_pose[1])))
                        yawsp=yaw_pid.get_pid((int(radar.rt_pose[2])))
                        self.speed_set(xsp,ysp,-yawsp)
                        if terminal.fall_sign ==True:
                            self.height_set(put_height)
                            logger.info("送餐结束 上升中")
                            if time.time()-self.time_count>5:
                                logger.info("下降结束，准备返回任务阶段2 回到5号点再次开始")
                                self.height_set(fly_height)
                                terminal.task_list.remove(terminal.task_list[self.nowpoint])
                                terminal.fall_sign=False
                                self.mission_step=2
                        else:
                            self.time_count=time.time()
                    pass
                time.sleep(0.02)
        def fc_take_off(self):
            self.com_fc[1]=1
        def fc_fall(self):
            self.com_fc[1]=0
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
            self.com_fc[6]=1
            lock.release()
            logger.info("任务结束")
        def rerun(self):
            self.mission_step=0
            self.task_running=True
            logger.info("任务重启")
            pass
        def gpio_init(self):
            self.com_gpio[1]=1
            self.com_gpio[2]=64
        def com_init(self):
            self.com_fc[1]=0
            self.com_fc[2]=sp_side
            self.com_fc[3]=sp_side
            self.com_fc[4]=fly_height
            self.com_fc[5]=sp_side
            self.com_fc[6]=0
    
