import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)  # BOARD编号方式，基于插座引脚编号
GPIO.setup(36, GPIO.OUT)  # 输出模式

# GPIO.setmode(GPIO.BCM)  # 若使用BCM编号方式，上两行代码应这样写
# GPIO.setup(17, GPIO.OUT)  # 对应为17号脚
 
while True:

    GPIO.output(36, GPIO.LOW)
    time.sleep(1)