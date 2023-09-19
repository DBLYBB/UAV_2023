import threading
import time
import math
import pickle
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side,lock,task_start_sign
#from Lcode.Lprotocol import udp_terminal
import socket
from RadarDrivers_reconstruct.Radar import Radar
#from t265_realsense import t265
radar=Radar()
radar.start('/dev/ttyUSBradar','LD06')
radar.start_resolve_pose(size=1200)
put_height=90
fly_height=171  
#realsense=t265.t265_class()
threshold=7
posthreshold=20
stxt=[170,64,64,64,255]
sendstxt=pickle.dumps(stxt)
udp=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
address=("255.255.255.255",2887)
udp.sendto(sendstxt,address)
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
            self.send_count=0
            self.radar_wait_time=0
            self.change_count=0
            self.t265sign=False
            self.radarsign=False
            self.x_intergral_base=0
            self.y_intergral_base=0
            self.pointcount=0
            self.iscvcap=False
            self.xyz=[0,0,0]
            self.xy=[]
            self.yaw=0
            self.radarbias=[0,0]
            self.target=[[0,-380],[80,-380],[80,20],[160,20],[160,-380],[240,-380],[240,20],[300,20],[300,-380],[0,0]]
        def run(self):
            self.radar_wait_time=time.time()
            self.gpio_set(5,1)
            self.time_count=0
            #realsense.autoset()
            self.task_running=True
            task_thread=threading.Thread(target=self.task)
            task_thread.daemon=True
            task_thread.start()
            self.mission_step=0
            logger.info("wait for radar ")
            pass
        def task(self):
            global put_height,fly_height
            while self.task_running==True:
                if time.time()-self.radar_wait_time>10 and self.radarsign==False:
                    logger.info("计算雷达偏置")
                    self.radarbias=[60,470] 
                    for i in range(len(self.target)):
                        self.target[i][0] += int(self.radarbias[0])
                        self.target[i][1] += int(self.radarbias[1])
                    self.radarsign=True
                    logger.info(self.radarbias)
                    self.fc_take_off()
                    self.height_set(fly_height)
                    logger.info("take off")
                if self.radarsign==True:
                    self.xyz=radar.rt_pose
                    self.yaw=radar.rt_pose[2]
                    if self.send_count<2:
                            self.send_count+=1
                    else:
                            self.xy=[170,int(self.xyz[0]-self.radarbias[0]),int(self.xyz[1]-self.radarbias[1]),0,255]
                            changedata=pickle.dumps(self.xy)
                            #print(self.xy)
                            udp.sendto(changedata,address)
                            self.send_count=0
                if task_start_sign.value==True :
                    if self.mission_step==0:
                        logger.info("开始延时")
                        self.time_count=time.time()
                        self.mission_step=1
                        self.gpio_set(5,0)
                    elif self.mission_step==1:
                        if time.time()-self.time_count<0:
                            
                            pass
                        else:
                            logger.info("前进")
                            self.mission_step=2
                    elif self.mission_step==2:
                        if self.pointcount<len(self.target) and self.iscvcap==False:
                            self.move_point(self.target[self.pointcount])
                        elif self.iscvcap==True:
                            self.mission_step=2
                            self.iscvcap=False
                            logger.info("发现火源 开始接近")
                        else:
                            logger.info("gg")
                            self.mission_step=101
                    elif self.mission_step==3: #发现火源后处理阶段 处理完成后返回阶段1
                        self.mission_step=101
                        pass
                    else:
                        self.end()
                time.sleep(0.05)
        def fc_take_off(self):
            self.com_fc[1]=1
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
            self.com_fc[1]=0
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
        def move_point(self,point):
            x_pid=PID(0,point[0])
            y_pid=PID(0,point[1])
            yaw_pid=PID(1,0)
            x_speed=x_pid.get_pid(radar.rt_pose[0])
            y_speed=y_pid.get_pid(radar.rt_pose[1])
            yaw_speed=yaw_pid.get_pid(radar.rt_pose[2])
            while((abs(x_speed)>threshold or abs(y_speed)>threshold)) or((abs(self.xyz[0]-point[0])>posthreshold or abs(self.xyz[1]-point[1])>posthreshold)) and self.iscvcap==False:
                if self.send_count<2:
                            self.send_count+=1
                else:
                            self.xy=[170,int(radar.rt_pose[0]-self.radarbias[0]),int(radar.rt_pose[1]-self.radarbias[1]),0,255]
                            changedata=pickle.dumps(self.xy)
                            udp.sendto(changedata,address)
                            self.send_count=0
                x_speed=x_pid.get_pid(radar.rt_pose[0])
                y_speed=y_pid.get_pid(radar.rt_pose[1])
                yaw_speed=yaw_pid.get_pid(radar.rt_pose[2])
                print("雷达返回数据为%s",radar.rt_pose) 
                print("dx=%d,dy=%d",radar.rt_pose[0]-point[0],radar.rt_pose[1]-point[1])
                print("xs=%d,ys=%d,yaws=%d",x_speed,y_speed,yaw_speed)
                self.speed_set(x_speed,y_speed,-yaw_speed)
                time.sleep(0.03)
            #self.speed_set(0,0,0)
            if self.iscvcap==False:
                self.pointcount+=1
                try:
                    x_pid=PID(0,self.target[self.pointcount][0])
                    y_pid=PID(0,self.target[self.pointcount][1])
                    x_speed=x_pid.get_pid(radar.rt_pose[0])
                    y_speed=y_pid.get_pid(radar.rt_pose[1])
                    if y_speed>0:
                        y_speed=y_speed*1
                    yaw_speed=yaw_pid.get_pid(radar.rt_pose[2])
                    self.speed_set(x_speed,y_speed,-yaw_speed)
                except:
                    pass
