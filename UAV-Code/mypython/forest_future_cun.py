import serial
import numpy as ny
import time
import binascii
import struct
import RPi.GPIO as GPIO
debounce_counter_36 = 0
debounce_counter_38 = 0
debounce_counter_40 = 0
GPIO.setmode(GPIO.BOARD)  # BOARD编号方式，基于插座引脚编号
GPIO.setwarnings(False)
GPIO.setup(31, GPIO.OUT)  # LED OUT
GPIO.setup(33, GPIO.OUT)  # LED OUT
GPIO.setup(35, GPIO.OUT)  # LED OUT
GPIO.setup(37, GPIO.OUT)  # LED OUT
GPIO.setup(32, GPIO.OUT)  # START LED OUT
GPIO.setup(36, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)  # BUTTON IN
GPIO.setup(38, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)  # BUTTON IN
GPIO.setup(40, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)  # BUTTON IN
ser0 = serial.Serial('/dev/ttyAMA0', 460800)
if ser0.isOpen == False:
    ser0.open()
com = [170,0,0,0,0,0,0,255]#AA,MODE,X,Y,Z,YAW,NEXT,FF
while True:
     # 扫描GPIO36输入状态
    GPIO.setwarnings(False)
    if GPIO.input(36) == GPIO.HIGH:
        # 如果GPIO36输入为高电平，将计数器加1
        debounce_counter_36 += 1
        if debounce_counter_36 == 3:
            # 当计数器达到20时，将GPIO32设置为高电平
            com[6]=1
    else:
        # 如果GPIO36输入为低电平，将计数器清零
        debounce_counter_36 = 0
    # 扫描GPIO38输入状态
    if GPIO.input(38) == GPIO.HIGH:
        # 如果GPIO38输入为高电平，将计数器加1
        debounce_counter_38 += 1
        if debounce_counter_38 == 3:
            # 当计数器达到20时，将com数组中第二个元素加1
            com[1] = min(com[1]+1, 3)
    else:
        # 如果GPIO38输入为低电平，将计数器清零
        debounce_counter_38 = 0

    # 扫描GPIO40输入状态
    if GPIO.input(40) == GPIO.HIGH:
        # 如果GPIO40输入为高电平，将计数器加1
        debounce_counter_40 += 1
        if debounce_counter_40 == 3:
            # 当计数器达到20时，将com数组中第二个元素减1
            com[1] = max(com[1]-1, 0)
    else:
        # 如果GPIO40输入为低电平，将计数器清零
        debounce_counter_40 = 0

    # 控制LED
    for value in com:
        hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
        ser0.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口0
    #serial
    # 检查串口接收到的数据长度是否大于等于5
     # 从串口缓冲区读取一个字节数据
    print("Com: ",com)
    # 如果读取到的数据是0xAA，则读取接下来的四个字节数据并输出
    if byte_data == b'\xaa':
        # 读取接下来的四个字节数据
        response = ser0.read(4)

        # 判断数据是否符合通信协议，即以0xFF结尾
        if response[3] == 0xff:
            # 将读取到的五个字节数据存入receive_data数组中
            receive_data = list(byte_data + response)
            # 打印数组receive_data
            print("Received data: ", receive_data)

            # 根据接收到的数据进行相应的控制操作
            if receive_data[3] == 1:
                com[6] = 0
            if receive_data[1] == 0:
                GPIO.output(31, not GPIO.input(31))
                GPIO.output(33, 0)
                GPIO.output(35, 0)
                GPIO.output(37, 0)
            if receive_data[1] == 1:
                GPIO.output(33, not GPIO.input(33))
                GPIO.output(31, 0)
                GPIO.output(35, 0)
                GPIO.output(37, 0)
            if receive_data[1] == 2:
                GPIO.output(35, not GPIO.input(35))
                GPIO.output(33, 0)
                GPIO.output(31, 0)
                GPIO.output(37, 0)
            if receive_data[1] == 3:
                GPIO.output(37, not GPIO.input(37))
                GPIO.output(33, 0)
                GPIO.output(35, 0)
                GPIO.output(31, 0)
            GPIO.output(32, receive_data[2] == 1)

    # 每次循环等待0.04秒
    time.sleep(0.01)
      