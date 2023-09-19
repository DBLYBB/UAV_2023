import serial
import threading
import pickle
import socket
from typing import List
from Lcode.Logger import logger
import time
from Lcode.global_variable import lock,task_start_sign
DEBUG=False
class Serial_fc(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate)
        self.fclisten_running=False
        self.fcsend_running=False
        self.rate=460800
        self.startbyte=b'\xAA'
        self.endbyte=0xFF
    def port_open(self):
        self.ser.close()
        if self.ser.is_open==False:
            self.ser.open()
            logger.info("目前飞控串口状态：%s",self.ser.is_open)
    def listen_start(self,rxbuffer:List[int]):
        self.fclisten_running=True
        listen_thread=threading.Thread(target=Serial_fc.listen_fc,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("飞控串口监听线程启动")
    def listen_end(self):
        self.fclisten_running=False
        logger.info("飞控串口监听线程关闭")
    def listen_fc(self,rxbuffer:List[int]):
        while self.fclisten_running ==True:
            byte_data = self.ser.read() 
            if byte_data == self.startbyte:
                # 读取接下来的四个字节数据
                recv = self.ser.read(6)
                # 判断数据是否符合通信协议，即以0xFF结尾
                if recv[5] == self.endbyte:
                    intergral_x = ((recv[1] << 8) | recv[2])-0x4000
                    intergral_y = ((recv[3] << 8) | recv[4])-0x4000
                    rxbuffer.clear()
                    rxbuffer.append(recv[0])
                    rxbuffer.append(intergral_x)
                    rxbuffer.append(intergral_y)
                    if recv[0]==0x05:
                        task_start_sign.value=True
                    else:
                        task_start_sign.value=False
                    if DEBUG :
                        logger.info(rxbuffer)
            time.sleep(0.05)
    def send_fc(self,comlist:List[int]):
        while self.fcsend_running==True:
            for value in comlist:
                hex_value = hex(int(value))[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                self.ser.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口
            time.sleep(0.01)
    def send_start(self,comlist:List[int]):
        self.fcsend_running=True
        fcsend_thread=threading.Thread(target=Serial_fc.send_fc,args=(self,comlist))
        fcsend_thread.daemon=True
        fcsend_thread.start()
        logger.info("飞控串口发送线程启动")
    def send_end(self):
        self.fcsend_running=False
        logger.info("飞控串口发送线程关闭")
        
class Serial_gpio(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate)
        self.gpiosend_running=False
        self.gpiolisten_running=False
        self.rate=460800
    def port_open(self):
        self.ser.close()
        if self.ser.is_open==False:
            self.ser.open()
            logger.info("目前gpio串口状态：%s",self.ser.is_open)
    def send_gpio(self,comlist:List[int]):
        while self.gpiosend_running==True:
            for value in comlist:
                hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                self.ser.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口
            time.sleep(0.02)
    def send_start(self,comlist:List[int]):
        self.gpiosend_running=True
        gpiosend_thread=threading.Thread(target=Serial_gpio.send_gpio,args=(self,comlist))
        gpiosend_thread.daemon=True
        gpiosend_thread.start()
        logger.info("gpio串口发送线程启动")
    def send_end(self):
        self.gpiosend_running=False
        logger.info("gpio串口发送线程关闭")
    def listen_start(self,rxbuffer:List[int]):
        self.gpiolisten_running=True
        listen_thread=threading.Thread(target=Serial_gpio.listen_gpio,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("gpio串口监听线程启动")
    def listen_end(self):
        self.gpiolisten_running=False
        logger.info("gpio串口监听线程关闭")
    def listen_gpio(self,rxbuffer:List[int]):
        while self.gpiolisten_running ==True:
            byte_data = self.ser.read() 
            if byte_data == b'\xAA':
                # 读取接下来的四个字节数据
                recv = self.ser.read(5)
                # 判断数据是否符合通信协议，即以0xFF结尾
                if recv[4] == 0xFF:
                    lock.acquire()
                    rxbuffer.clear()
                    for i in range(0,4):
                        rxbuffer.append(recv[i])
                    if task_start_sign.value ==False:
                        logger.info(rxbuffer)
                    lock.release()
            time.sleep(0.05)
class udp_terminal(object):
    def __init__(self):
        self.udp_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_listen_running=False
        self.udp_send_running=False
        self.takeoff_sign=False
        self.task_hjm=False
        self.task_number=0
    def listen_start(self,IP,PORT):
        self.udp_listen_running=True
        self.udp_socket.bind((IP,PORT))
        listen_thread=threading.Thread(target=udp_terminal.listen_udp,args=(self,))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("udp监听线程启动")
    def listen_end(self):
        self.udp_listen_running=False
        logger.info("udp监听线程关闭")
    def listen_udp(self):
        received_data=[]
        state=0
        while self.udp_listen_running==True:
            data, client_address = self.udp_socket.recvfrom(1024)
            #logger.info("接收到的数据是%s",data)
            realdata=pickle.loads(data)
            if realdata[0]==170 and realdata[3]==255:
                if realdata[1]==160 and realdata[2]==160:
                    self.task_number=1
                elif realdata[1]==160 and realdata[2]==161:
                    self.task_number=2
                elif realdata[1]==192 and realdata[2]==192:
                    self.task_hjm=True
            time.sleep(0.02)
    """ def tasksort(self):
        self.task_list.sort(key=lambda x:x[0],reverse=True)
        logger.info("任务列表排序后:%s",self.task_list) """
    
    def send_start(self,IP,PORT,senddata):
        self.udp_send_running=True
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # 设置允许发送广播数据
        send_thread=threading.Thread(target=udp_terminal.send_udp,args=(self,IP,PORT,senddata))
        send_thread.daemon=True
        send_thread.start()
        logger.info("udp发送线程启动")
    def send_udp(self,IP,PORT,senddata):
        while self.udp_send_running==True:
            changedata=pickle.dumps(senddata)
            self.udp_socket.sendto(changedata,(IP,PORT))
            time.sleep(0.05)