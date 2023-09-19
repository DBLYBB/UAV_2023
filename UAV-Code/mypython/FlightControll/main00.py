import os,sys
dir_path = os.path.dirname(os.path.realpath(__file__))
filename = "fc_log.log"
if os.path.isfile(os.path.join(dir_path, filename)):
    os.remove(os.path.join(dir_path, filename))
sys.path.append(os.path.abspath('./yolo_v2'))
#os.system("netsh wlan connect name= LESP32WF")
#########################################  清除日志  #########################################
from Lcode.global_variable import sp_side,lock
import Lcode.Lprotocol
import time
from Lcode.Logger import logger
from RadarDrivers_reconstruct.Radar import Radar
#from Lcode.Lmission.Lmission_final import mission
##############################################变量############################################
rxbuffer_fc=[0,0,0]#飞控反传信息 任务模式/x积分值/y积分值
rxbuffer_gpio=[0,0,0,0]#gpio反传信息,启动/P1/P2/模式
com_fc = [170, 0, sp_side, sp_side, 120, sp_side, 0,sp_side, 255]#发送给飞控的数据 帧头/启动命令/x/y/z高度/yaw/任务切换标志位/速度偏置量/帧尾
com_gpio =[170,0,0,0,0,0,0,0,0,255]#发送给esp32/arduino的数据 GPIO输出 帧头/低功率输出/高功率输出
run_sign=False
recv_address='0.0.0.0'
recv_port=3333
##########################################  任务  #############################################
serial_fc=Lcode.Lprotocol.Serial_fc("/dev/ttyAMA0",460800)
serial_fc.port_open()
serial_fc.listen_start(rxbuffer_fc)
serial_fc.send_start(com_fc)
serial_gpio=Lcode.Lprotocol.Serial_gpio("/dev/ttyUSBArduino",38400)
#serial_gpio.port_open()
serial_gpio.send_start(com_gpio)
serial_gpio.listen_start(rxbuffer_gpio)
terminal=Lcode.Lprotocol.udp_terminal()
terminal.listen_start(recv_address,recv_port)
""" radar=Radar()
radar.start('COM3','LD06')
radar.start_resolve_pose() """
# terminal.task_number=2
# terminal.task_hjm=True
while(1):
     if terminal.task_hjm==True and run_sign==False:
         if terminal.task_number==1:
             from Lcode.Lmission import Lmission_final
             logger.info("任务1选择")
             mission1=Lmission_final.mission(rxbuffer_fc,com_fc,com_gpio,rxbuffer_gpio)
             time.sleep(0.5)
             mission1.run()
             run_sign=True
         elif terminal.task_number==2:
             from Lcode.Lmission import Lmission_final_kai
             logger.info("任务2选择")
             mission2=Lmission_final_kai.mission(rxbuffer_fc,com_fc,com_gpio,rxbuffer_gpio)
             time.sleep(0.5)
             mission2.run()
             run_sign=True
    #print(radar.rt_pose)
     time.sleep(0.1)