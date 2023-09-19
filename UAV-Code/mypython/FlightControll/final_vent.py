import cv2
import sys
import os
import time
point1,point2=[],[]
sys.path.append(os.path.abspath('./RadarDribers_reconstruct'))
from RadarDrivers_reconstruct.Radar import Radar
if __name__ == "__main__":
    radar = Radar()
    radar.start('/dev/ttyUSB0', 'LD06')
    radar.start_resolve_pose()
while True:
    print(radar.rt_pose)
    time.sleep(0.05)


