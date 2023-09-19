import time
import threading
import serial
import cv2
import numpy as np
from RadarDrivers_reconstruct.RadarMapResolve import radar_map_resolve
from RadarDrivers_reconstruct.RadarMapBase import Point_2D
from Lcode.Logger import logger


class radar_map_application(radar_map_resolve):

    """
    雷达点云图像应用器
    """

    def __init__(self):
        super().__init__()
        self.pose_running = False
        self._fp_flag = False
        self._rtpose_flag = False
        self._rtpose_flag_two_point = False
        self.fp_points = []
        self.rt_pose = [0.0, 0.0, 0.0]
        self._rt_pose_inited = [False, False, False]
        self.rt_pose_update_event = threading.Event()

    def start_pose_task(self):
        """
        开始使用雷达数据进行位置解算
        """
        self.pose_running = True
        thread = threading.Thread(target=self.map_resolve_task)
        thread.daemon = True  # 守护线程的意思就是在这个线程运行时，主线程的退出不会导致整个程序的退出，会等到所有守护线程结束
        thread.start()
        self.thread_list.append(thread)
        logger.info("[RADAR] Map resolve thread started")

    def pose_stop(self, joined=False):
        """
        停止监听雷达数据
        """
        self.pose_running = False
        if joined:
            thread = self.thread_list[1]
            thread.join()  # 日常解释join():认为是让主线程等待其执行完，其实就是在没执行完其中一个线程时，这句话就不会继续往下走
        logger.info("[RADAR] Stopped all threads")

    def start_resolve_pose(
        self, size: int = 1000, scale_ratio: float = 1, low_pass_ratio: float = 0.5
    ):
        """
        开始使用点云图解算位姿
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        """
        self._rtpose_flag = True
        self._rtpose_size = size
        self._rtpose_scale_ratio = scale_ratio
        self._rtpose_low_pass_ratio = low_pass_ratio
        self.rt_pose = [0.0, 0.0, 0.0]
        self._rt_pose_inited = [False, False, False]

    def start_resolve_pose_with_two_points(
        self, size: int = 1000, scale_ratio: float = 1, low_pass_ratio: float = 0.5
    ):
        """
        开始使用两点解算位姿
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        """
        self._rtpose_flag_two_point = True
        self._rtpose_size = size
        self._rtpose_scale_ratio = scale_ratio
        self._rtpose_low_pass_ratio = low_pass_ratio
        self.rt_pose = [0.0, 0.0, 0.0]
        self._rt_pose_inited = [False, False, False]

    def map_resolve_task(self):
        while self.pose_running:
            try:
                if self._map_updated_event.wait(1):  # 保证在更新位置时，串口数据已经读取完毕
                    self._map_updated_event.clear()
                    if self._fp_flag:
                        self.check_target_point()
                        if self._fp_type == 0:
                            self._update_target_point(self.find_nearest(*self._fp_arg))
                        elif self._fp_type == 1:
                            self._update_target_point(
                                self.find_nearest_with_ext_point_opt(*self._fp_arg)
                            )

                    if self._rtpose_flag:
                        x, y, yaw = self.map_visual_resolve_rt_pose(
                            0, self._rtpose_size, self._rtpose_scale_ratio, DEBUG=False
                        )

                        if x is not None:
                            if self._rt_pose_inited[0]:
                                self.rt_pose[0] += (
                                    x / self._rtpose_scale_ratio - self.rt_pose[0]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[0] = x / self._rtpose_scale_ratio
                                self._rt_pose_inited[0] = True
                        if y is not None:
                            if self._rt_pose_inited[1]:
                                self.rt_pose[1] += (
                                    y / self._rtpose_scale_ratio - self.rt_pose[1]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[1] = y / self._rtpose_scale_ratio
                                self._rt_pose_inited[1] = True
                        if yaw is not None:
                            if self._rt_pose_inited[2]:
                                self.rt_pose[2] += (
                                    yaw - self.rt_pose[2]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[2] = yaw
                                self._rt_pose_inited[2] = True
                        self.rt_pose_update_event.set()  # 通知位姿更新
                else:
                    logger.warning("[RADAR] Map resolve thread wait timeout")
            except Exception as e:
                import traceback

                logger.error(
                    f"[RADAR] Map resolve thread error: {traceback.format_exc()}"
                )
                time.sleep(0.5)

    def pose_resolve_task_with_two_points(self):
        """
        两点定位任务
        """
        while self.pose_running:
            try:
                if self._map_updated_event.wait(1):
                    self._map_updated_event.clear()
                    if self._rtpose_flag_two_point and not self._rtpose_flag:
                        x, y, yaw = self.find_obstacles_with_filter(
                            _rtpose_size=1000, _rtpose_scale=1, DEBUG=True
                        )
                    elif self._rtpose_flag and not self._rtpose_flag_two_point:
                        x, y, yaw = self.map_visual_resolve_rt_pose(
                            self._rtpose_size, self._rtpose_scale_ratio
                        )
                        if x is not None:
                            if self._rt_pose_inited[0]:
                                self.rt_pose[0] += (
                                    x / self._rtpose_scale_ratio - self.rt_pose[0]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[0] = x / self._rtpose_scale_ratio
                                self._rt_pose_inited[0] = True
                        if y is not None:
                            if self._rt_pose_inited[1]:
                                self.rt_pose[1] += (
                                    y / self._rtpose_scale_ratio - self.rt_pose[1]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[1] = y / self._rtpose_scale_ratio
                                self._rt_pose_inited[1] = True
                        if yaw is not None:
                            if self._rt_pose_inited[2]:
                                self.rt_pose[2] += (
                                    yaw - self.rt_pose[2]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[2] = yaw
                                self._rt_pose_inited[2] = True
                        self.rt_pose_update_event.set()  # 通知位姿更新
                else:
                    logger.warning("[RADAR] Map resolve thread wait timeout")
            except Exception as e:
                import traceback

                logger.error(
                    f"[RADAR] Map resolve thread error: {traceback.format_exc()}"
                )
                time.sleep(0.5)

    def stop_resolve_pose(self):
        """
        停止使用点云图解算位姿
        """
        self._rtpose_flag = False
        self.rt_pose = [0.0, 0.0, 0.0]
        self._rt_pose_inited = [False, False, False]
        self.rt_pose_update_event.clear()

    def stop_resolve_pose_with_two_points(self):
        """
        停止使用两点解算位姿
        """
        self._rtpose_flag_two_point = False
        self.rt_pose = [0.0, 0.0, 0.0]
        self._rt_pose_inited = [False, False, False]
        self.rt_pose_update_event.clear()

    def update_resolve_pose_args(
        self, size: int = 1000, ratio: float = 1, low_pass_ratio: float = 0.5
    ):
        """
        更新参数
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        """
        self._rtpose_size = size
        self._rtpose_scale_ratio = ratio
        self._rtpose_low_pass_ratio = low_pass_ratio

    def start_find_point(
        self,
        timeout: float,
        type: int,
        from_: int,
        to_: int,
        num: int,
        range_limit: int,
    ):
        """
        开始更新目标点
        timeout: 超时时间, 超时后fp_timeout_flag被置位
        type: 0:直接搜索 1:极值搜索
        其余参数与find_nearest一致
        """
        self._fp_update_time = time.time()
        self._fp_timeout = timeout
        self.fp_timeout_flag = False
        self.fp_points = []
        self._fp_flag = True
        self._fp_type = type
        self._fp_arg = (from_, to_, num, range_limit)

    def stop_find_point(self):
        """
        停止更新目标点
        """
        self._fp_flag = False

    def _update_target_point(self, points: list[Point_2D]):
        """
        更新目标点位置
        """
        if self.fp_timeout_flag and len(points) > 0:
            self.fp_timeout_flag = False
        elif len(points) > 0:
            self.fp_points = points
            self._fp_update_time = time.time()

    def check_target_point(self):
        """
        目标点超时判断
        """
        if (
            not self.fp_timeout_flag
            and time.time() - self._fp_update_time > self._fp_timeout
        ):
            self.fp_timeout_flag = True
            logger.warning("[Radar] lost point!")
