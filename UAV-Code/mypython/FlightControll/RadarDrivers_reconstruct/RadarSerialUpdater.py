import threading
import time
import struct
import serial
from Lcode.Logger import logger
from RadarDrivers_reconstruct.RadarMapBase import Map_360, Radar_Package


class radar_serial_updater(Map_360):
    """
    雷达是双线程的模块,包括串口接收线程和地图解析线程,两个线程间的同步靠的是线程锁event
    23.5.5:
    重构,此模块的内容是完成数据解析,并且更新地图
    """

    CRC_TABLE = [
        0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,  # 0x00 - 0x0F
        0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,  # 0x10 - 0x1F
        0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,  # 0x20 - 0x2F
        0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,  # 0x30 - 0x3F
        0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,  # 0x40 - 0x4F
        0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,  # 0x50 - 0x5F
        0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,  # 0x60 - 0x6F
        0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,  # 0x70 - 0x7F
        0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,  # 0x80 - 0x8F
        0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,  # 0x90 - 0x9F
        0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,  # 0xA0 - 0xAF
        0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,  # 0xB0 - 0xBF
        0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,  # 0xC0 - 0xCF
        0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,  # 0xD0 - 0xDF
        0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,  # 0xE0 - 0xEF
        0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,  # 0xF0 - 0xFF
    ]  # fmt: skip

    def __init__(self):
        super().__init__()
        self.serial_running = False
        self.package = Radar_Package()
        self.serial = serial.Serial()
        self._update_callback = None
        self._map_updated_event = threading.Event()
        self.radar_unpack_fmt = "<HH" + "HB" * 12 + "HH"  # 雷达数据解析格式

    def start_serial_task(self, com_port, radar_type: str = "LD06"):
        """
        开始监听雷达数据
        radar_type: LD08 or LD06
        update_callback: 回调函数，每次更新雷达数据时调用
        """
        if self.serial_running:
            self.serial_stop()
        if radar_type == "LD08":
            baudrate = 115200
        elif radar_type == "LD06":
            baudrate = 230400
        else:
            raise ValueError("Unknown radar type")
        self.serial = serial.Serial(com_port, baudrate=baudrate)
        self.serial_running = True
        thread = threading.Thread(target=self.read_serial_task)
        thread.daemon = True
        thread.start()
        self.thread_list.append(thread)
        logger.info("[radar] serial task start")

    def serial_stop(self, joined=False):
        """
        停止监听雷达数据
        """
        self.serial_running = False
        if joined:
            thread = self.thread_list[0]
            thread.join()  # 日常解释join():认为是让主线程等待其执行完，其实就是在没执行完其中一个线程时，这句话就不会继续往下走
        if self.serial is not None:
            self.serial.close()
        logger.info("[RADAR] Stopped serial threads")

    def read_serial_task(self):
        """
        基本串口操作,大部分备注留在Serial文件里了
        """
        reading_flag = False
        start_bit = b"\x54\x2C"
        package_length = 45
        read_buffer = bytes()
        wait_buffer = bytes()
        while self.serial_running:
            # try:
            if self.serial.in_waiting > 0:
                if not reading_flag:  # 等待包头
                    wait_buffer += self.serial.read(1)
                    if len(wait_buffer) >= 2:
                        if wait_buffer[-2:] == start_bit:
                            reading_flag = True
                            # read_count = 0
                            wait_buffer = bytes()
                            read_buffer = start_bit
                else:  # 读取数据
                    read_buffer += self.serial.read(package_length)
                    reading_flag = False
                    # 直接传入写在RadarResolver中的解析函数解析(这里之前写的有问题（但可能是我有问题））
                    self.package = self.resolve_radar_data(
                        read_buffer, self.package
                    )
                    if self.package.recent_update_result:
                        self.update(self.package)
                        self._map_updated_event.set()  # 保证有输入之后才更新地图，减少占用，其实可以理解为两个线程之间的同步过程
                    else:
                        logger.error("[RADAR] Map Update Error")
                    if self._update_callback is not None:
                        self._update_callback()
            else:
                time.sleep(0.001)
            # except Exception as e:
            #     # logger.error(f"[RADAR] Listenning thread error: {e}")
            #     time.sleep(0.5)

    def resolve_radar_data(self, data: bytes, to_package: Radar_Package):
        """
        解析雷达原始数据
        data: bytes 原始数据
        to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
        return: 解析后的RadarPackage对象
        CRC8为校验码,用于判断数据是否正确
        """
        if len(data) != 47:  # fixed length of radar data
            logger.warning(f"[RADAR] Invalid data length: {len(data)}")
            to_package.recent_update_result = False
            return to_package
        if self.calculate_crc8(data[:-1]) != data[-1]:
            logger.warning("[RADAR] Invalid CRC8")
            to_package.recent_update_result = False
            return to_package
        if data[:2] != b"\x54\x2C":
            logger.warning(f"[RADAR] Invalid header: {data[:2]:X}")
            to_package.recent_update_result = False
            return to_package
        datas = struct.unpack(self.radar_unpack_fmt, data[2:-1])
        if to_package is None:
            to_package = Radar_Package(datas)
            to_package.recent_update_result = True
            return to_package
        else:
            to_package.fill_data(datas)
            to_package.recent_update_result = True
            return to_package

    def calculate_crc8(self, data: bytes) -> int:
        """
        Calculate CRC 8 of data.
        data: bytes-like object
        """
        crc = 0x00
        for byte in data:
            crc = self.CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc
