# from typing import Any
import serial
import threading
from Lcode.Logger import logger
from RadarDrivers_reconstruct.RadarMapApplication import radar_map_application


class Radar(radar_map_application):
    def __init__(self) -> None:
        self.com_port = None
        self.radar_type = None
        super().__init__()

    def start(self, com_port: str, radar_type: str = "LD06") -> None:
        # 开始监听串口
        self.start_serial_task(com_port=com_port, radar_type=radar_type)
        # 开始地图解算
        self.start_pose_task()
