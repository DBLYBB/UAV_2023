import serial
import numpy as ny
import time
import binascii
import struct
import RPi.GPIO as GPIO
ser0 = serial.Serial('/dev/ttyAMA0', 460800)
if ser0.isOpen == False:
    ser0.open()