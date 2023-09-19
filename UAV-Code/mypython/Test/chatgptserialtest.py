import serial
import numpy as ny
import time
import binascii
import struct

ser0 = serial.Serial('/dev/ttyAMA0', 460800)
if ser0.isOpen == False:
    ser0.open()

com = [170,0,2,3,4,5,6,255]#AA,MODE,X,Y,Z,YAW,NEXT,FF
receive_data=[]
while True:
    for value in com:
        hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
        ser0.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口0
    size = ser0.inWaiting()
    if size != 0:
         # 接收数据并保存到数组data
        response = ser0.read(2)
        a, b = struct.unpack('BB', response)
        receive_data.append(a)
        receive_data.append(b)
        # 打印数组data
        print("Received data: ", receive_data)
        # 清空输入缓冲区
        ser0.flushInput()
        # 等待1秒
    time.sleep(1)
