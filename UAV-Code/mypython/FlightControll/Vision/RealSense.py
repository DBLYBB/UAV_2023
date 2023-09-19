import datetime
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import pyrealsense2 as rs

    rs.config()
except:
    import pyrealsense2.pyrealsense2 as rs  # for linux

from loguru import logger


def quaternions_to_euler(x, y, z, w):
    # mathod 1
    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * x))
    # y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # mathod 2
    # r = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    # p = -math.asin(2.0 * (x * z - w * y))
    # y = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

    # mathod 3
    # Resolve the gimbal lock problem
    sinp = 2.0 * (w * y - z * x)
    p = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    y = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # convert radians to degrees
    r, p, y = math.degrees(r), math.degrees(p), math.degrees(y)
    return r, p, y


def quaternions_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    """
    将wxyz的四元数转换为3x3的旋转矩阵
    """
    # 构造旋转矩阵
    rotation_matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )

    return rotation_matrix


def rotation_matrix_to_quaternions(rotation_matrix: np.ndarray) -> tuple:
    """
    将3x3的旋转矩阵转换为wxyz的四元数
    """
    # 计算四元数的w分量
    w = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
    # 计算四元数的x, y, z分量
    x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * w)
    y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * w)
    z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * w)

    return x, y, z, w


@dataclass
class T265_Pose_Frame(object):
    """
    T265 姿态数据帧
    """

    @dataclass
    class _XYZ:  # 三维坐标
        x: float
        y: float
        z: float

    @dataclass
    class _WXYZ:  # 四元数
        w: float
        x: float
        y: float
        z: float

    translation: _XYZ  # 位移 / m
    rotation: _WXYZ  # 四元数姿态
    velocity: _XYZ  # 速度 / m/s
    acceleration: _XYZ  # 加速度 / m/s^2
    angular_velocity: _XYZ  # 角速度 / rad/s
    angular_acceleration: _XYZ  # 角加速度 / rad/s^2
    tracker_confidence: int  # 跟踪置信度 0: Failed, 1: Low, 2: Medium, 3: High
    mapper_confidence: int  # 建图置信度 0: Failed, 1: Low, 2: Medium, 3: High


"""
note:
T265 姿态坐标系
            y
         z  ^
          \ |
           \|
   x<---[ (O O)]
pitch-dx, yaw-dy, roll-dz
所有轴向均为右手系
"""


class T265(object):
    """
    Realsense T265 包装类
    """

    def __init__(
        self, log_to_file: bool = False, log_to_console: bool = False, event_skip=0, log_level: str = "info", **args
    ) -> None:
        """
        初始化 T265
        """
        self.pose: T265_Pose_Frame = None  # type: ignore
        self.frame_num: int = 0  # frame number
        self.frame_timestamp: float = 0.0  # timestamp
        self.running = False
        self.update_event = threading.Event()
        self.event_skip = event_skip
        if log_to_file:
            rs.log_to_file(getattr(rs.log_severity, log_level), "rs_t265.log")
        if log_to_console:
            rs.log_to_console(getattr(rs.log_severity, log_level))
        self._connect(**args)
        self._connect_args = args
        self._callbacks: list[Callable] = []
        self.secondary_frame_established = False

    def _connect(self, **args) -> None:
        self._pipe = rs.pipeline()
        self._cfg = rs.config()
        self._cfg.enable_stream(rs.stream.pose)
        self._device = self._cfg.resolve(self._pipe).get_device()
        logger.info(f"[T265] Connected to {self._device}")
        logger.debug(f"[T265] Device sensors: {self._device.query_sensors()}")
        pose_sensor = self._device.first_pose_sensor()
        logger.debug(f"[T265] Pose sensor: {pose_sensor}")
        pose_sensor.set_option(rs.option.enable_auto_exposure, args.get("enable_auto_exposure", 1))
        pose_sensor.set_option(rs.option.enable_mapping, args.get("enable_mapping", 1))
        pose_sensor.set_option(rs.option.enable_map_preservation, args.get("enable_map_preservation", 1))
        pose_sensor.set_option(rs.option.enable_relocalization, args.get("enable_relocalization", 1))
        pose_sensor.set_option(rs.option.enable_pose_jumping, args.get("enable_pose_jumping", 1))
        pose_sensor.set_option(rs.option.enable_dynamic_calibration, args.get("enable_dynamic_calibration", 1))
        logger.debug(f"[T265] Pose sensor options:")
        for opt in pose_sensor.get_supported_options():
            logger.debug(f"[T265]   {opt}: {pose_sensor.get_option(opt)}")

    def _callback(self, frame) -> None:
        pose = frame.as_pose_frame()
        if not pose:
            return
        self.frame_num = pose.frame_number
        self.frame_timestamp = pose.timestamp
        self.pose = pose.get_pose_data()
        if self._print_update:
            self._print_pose()
        self._update_count += 1
        if self._callbacks:
            for callback in self._callbacks:
                callback(self.pose, self.frame_num, self.frame_timestamp)
        if self.event_skip == 0 or self._update_count % self.event_skip == 0:
            self.update_event.set()

    def _print_pose(self, refresh=True) -> None:
        BACK = "\033[F"
        r, p, y = self.eular_rotation
        text = (
            f"T265 Pose Frame #{self.frame_num} at {datetime.datetime.fromtimestamp(self.frame_timestamp / 1000)}\n"
            f"Translation    :{self.pose.translation.x:11.6f},{self.pose.translation.y:11.6f},{self.pose.translation.z:11.6f};\n"
            f"Velocity       :{self.pose.velocity.x:11.6f},{self.pose.velocity.y:11.6f},{self.pose.velocity.z:11.6f};\n"
            f"Acceleration   :{self.pose.acceleration.x:11.6f},{self.pose.acceleration.y:11.6f},{self.pose.acceleration.z:11.6f};\n"
            f"Angular vel    :{self.pose.angular_velocity.x:11.6f},{self.pose.angular_velocity.y:11.6f},{self.pose.angular_velocity.z:11.6f};\n"
            f"Angular accel  :{self.pose.angular_acceleration.x:11.6f},{self.pose.angular_acceleration.y:11.6f},{self.pose.angular_acceleration.z:11.6f};\n"
            f"Rotation       :{self.pose.rotation.w:11.6f},{self.pose.rotation.x:11.6f},{self.pose.rotation.y:11.6f},{self.pose.rotation.z:11.6f};\n"
            f"Roll/Pitch/Yaw :{r:11.6f},{p:11.6f},{y:11.6f};\n"
            f"Tracker conf: {self.pose.tracker_confidence}, Mapper conf: {self.pose.mapper_confidence}"
        )
        if self.secondary_frame_established:
            position, eular = self.get_pose_in_secondary_frame(as_eular=True)
            text += (
                f"\n2nd Translation   : {position[0]:11.6f},{position[1]:11.6f},{position[2]:11.6f};\n"
                f"2nd Roll/Pitch/Yaw: {eular[0]:11.6f},{eular[1]:11.6f},{eular[2]:11.6f}"
            )
            if refresh:
                text += BACK * 3
        if refresh:
            print(f"{text}{BACK* 8}", end="")

    def start(self, async_update: bool = True, print_update: bool = False) -> None:
        """
        开始监听 T265

        async_update: 是否使用异步回调的方式监听, 若为 False, 则需要手动调用 update() 方法
        print_update: 是否在控制台打印更新
        lightweight_update: 是否使用轻量级更新(仅更新位置和四元数姿态数据)
        """
        self._async = async_update
        self._print_update = print_update
        if self._async:
            self._pipe.start(self._cfg, self._callback)
        else:
            self._pipe.start(self._cfg)
        self._update_count = 0
        self.secondary_frame_established = False
        self._start_time = time.perf_counter()
        self.running = True
        logger.info("[T265] Started")

    def update(self):
        """
        更新 T265 状态(阻塞直到有新的数据帧到来)
        """
        if not self.running:
            raise RuntimeError("T265 is not running")
        if self._async:
            raise RuntimeError("Async mode")
        frames = self._pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if not pose:
            return
        self.frame_num = pose.frame_number
        self.frame_timestamp = pose.timestamp
        self.pose = pose.get_pose_data()
        if self._print_update:
            self._print_pose()
        self._update_count += 1
        if self._callbacks:
            for callback in self._callbacks:
                callback(self.pose, self.frame_num, self.frame_timestamp)
        if self.event_skip == 0 or self._update_count % self.event_skip == 0:
            self.update_event.set()

    def register_callback(self, callback: Callable[[T265_Pose_Frame, int, float], None]) -> None:
        """
        注册 T265 更新回调函数
        回调参数: T265_Pose_Frame, 帧编号, 帧时间戳(ms)
        """
        self._callbacks.append(callback)

    def stop(self) -> None:
        """
        停止监听 T265
        """
        self._pipe.stop()
        self.running = False
        logger.info("[T265] Stopped")

    @property
    def fps(self) -> float:
        """
        获取 T265 的平均更新速率
        """
        fps = self._update_count / (time.perf_counter() - self._start_time)
        self._update_count = 0
        self._start_time = time.perf_counter()
        return fps

    def hardware_reset(self) -> None:
        """
        强制重置 T265 并重新连接
        """
        self._device.hardware_reset()
        logger.warning("[T265] Hardware reset, waiting for reconnection...")
        while True:
            try:
                self._connect(**self._connect_args)
                break
            except RuntimeError:
                time.sleep(1)
        if self.running:
            if self._async:
                self._pipe.start(self._cfg, self._callback)
            else:
                self._pipe.start(self._cfg)
            self._update_count = 0
            self._start_time = time.perf_counter()
        self.secondary_frame_established = False

    @property
    def eular_rotation(self) -> Tuple[float, float, float]:
        """
        获取欧拉角姿态
        返回值: roll, pitch, yaw
        """
        # in convert matrices: roll (x), pitch (y), yaw (z)
        # so we swap axis: x, y, z = r_z, r_x, r_y
        # return quaternions_to_euler(
        #     self.pose.rotation.z, self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.w
        # )

        return Rotation.from_quat(
            [self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w]
        ).as_euler("zxy", degrees=True)

    def establish_secondary_origin(
        self,
        force_level: bool = True,
        x_offset: float = 0.0,
        y_offset: float = 0.0,
        z_offset: float = 0.0,
        yaw_offset: float = 0,
    ):
        """
        以当前位置和姿态建立副坐标系原点
        force_level: 强制副坐标系为水平面
        offset: 当前位置相对于副坐标系原点的偏移
        (yaw_offset: 仅当返回eular时有效)
        """
        # 获取当前位置和朝向
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])
        if force_level:
            orientation[0] = 0
            orientation[2] = 0

        # 将当前位置和朝向作为副坐标系的原点和朝向
        # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
        self._secondary_position = position
        self._secondary_orientation = orientation
        self._secondary_rotation = Rotation.from_quat(orientation)  # xyzw
        self._secondary_rotation_matrix = self._secondary_rotation.as_matrix()
        self._offset_position = np.array([x_offset, y_offset, z_offset])
        self._offset_yaw = yaw_offset
        # logger.debug(f"[T265] Secondary origin established: {self._secondary_position}, {self._secondary_orientation}")
        self.secondary_frame_established = True

    def get_pose_in_secondary_frame(self, as_eular=True) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取当前位置和姿态在副坐标系中的表示
        as_eular: 是否返回欧拉角
        return: xyz位置, xyzw四元数/rpy欧拉角
        """
        if not self.secondary_frame_established:
            raise RuntimeError("Secondary frame not established")
        # 获取当前位置和朝向
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])

        # 将当前位置和朝向转换到副坐标系中
        position -= self._secondary_position
        # 反向应用副坐标系的旋转矩阵
        position = np.dot(position, self._secondary_rotation_matrix.T)
        # 反向应用副坐标系的朝向
        # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
        # rotation_matrix = np.dot(rotation_matrix, self._secondary_rotation_matrix.T)
        # orientation = rotation_matrix_to_quaternions(rotation_matrix)
        rotation = Rotation.from_quat(orientation) * self._secondary_rotation.inv()

        position -= self._offset_position
        if as_eular:
            euler = rotation.as_euler("zxy", degrees=True)
            if self._offset_yaw != 0:
                euler[2] = (euler[2] - self._offset_yaw + 180) % 360 - 180
            return position, euler
        return position, rotation.as_quat()


if __name__ == "__main__":
    t265 = T265()
    # t265.hardware_reset()
    t265.start(print_update=True)
    try:
        while True:
            time.sleep(0.1)
            get = input(">>> \n")
            if get == "e":
                t265.establish_secondary_origin()
            elif get == "g":
                print(f"pose: {t265.get_pose_in_secondary_frame()}" + " " * 60)
    finally:
        t265.stop()
