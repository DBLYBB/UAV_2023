import serial
import numpy as ny
import time
import binascii
import struct
import RPi.GPIO as GPIO

ser0 = serial.Serial('/dev/ttyAMA0', 460800)
if ser0.isOpen == False:
    ser0.open()
com = [170, 0, 0, 0, 0, 0, 0, 255]  # AA,MODE,X,Y,Z,YAW,NEXT,FF
stage = 0
change32_sign = 0
receive_data = []
while True:
    # 发送数据
    for value in com:
        hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
        ser0.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口0
    print("Com: ", com)
    # 读取数据
    byte_data = ser0.read()

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

            # 根据数据切换阶段
            if (receive_data[2] == 1) & (change32_sign == 0):
                stage += 1
                change32_sign = 1
    # 如果派阶段和32阶段不同，发送命令让32切换阶段
    if (change32_sign == 1) & (stage != receive_data[1]):
        com[6] = 1
    else:
        change32_sign = 0
        com[6] = 0

    # 每次循环等待0.01秒
    time.sleep(0.01)
