import serial
import numpy as ny
import time
import binascii

ser0 = serial.Serial('/dev/ttyAMA0',460800)
if ser0.isOpen  ==False:
    ser0.open()

while  True:
    ser0.write(bytes.fromhex("AA"))
    ser0.write(bytes.fromhex("01"))
    ser0.write(bytes.fromhex("01"))
    ser0.write(bytes.fromhex("01"))
    ser0.write(bytes.fromhex("01"))
    ser0.write(bytes.fromhex("01"))
    ser0.write(bytes.fromhex("FF"))
    size = ser0.inWaiting()
    if size !=0:
        response = ser0.read(size)
        print (response)
        ser0.flushInput()
    time.sleep(1)
