import pyrealsense2 as rs
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math as m
import cv2
import threading
from Lcode.Logger import logger
class t265_class:
    first_flag = 1
    time_start = 0
    base_pos = [0, 0, 0]
    base_yaw = 0
    xy_route = []
    map_size = 1000 # 地图大小
    point_cache_size = 10000 # 地图点缓存大小
    set_time = 3 # 自动标定时间
    t265_pose=[0,0,0]
    update_running=False
    def __init__(self):
        """
        初始化t265
        所有参数会置为0
        """
        # 声明rs流水线，封装实际设备和传感器
        self.pipe = rs.pipeline()
        # 构建配置对象并请求姿势数据
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.pose)
        # 使用请求的配置开始流式传输
        self.pipe.start(self.cfg)
        self.DEBUG = False

    def arr_trans(self, arr):
        """
        x,y,z --> -z,-x,y
        将t265的坐标系转换为匿名坐标系
        """
        x = arr[0]
        y = arr[1]
        z = arr[2]
        arr[0] = -z
        arr[1] = -x
        arr[2] = y
        return arr

    def get_raw_data(self):
        """
        获取t265的原始数据(转换后的匿名坐标系)
        格式: [id,pos,vel,acc]
        id: 帧id
        pos: 位置[x,y,z]
        vel: 速度[x,y,z]
        acc: 加速度[x,y,z]
        quaternion: 四元数[w, x, y, z]
        euler: 欧拉角[pitch, roll, yaw]
        """
        try:
            # 等待相机的下一组帧
            frames = self.pipe.wait_for_frames()
            # 获取姿势帧
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                id = int(pose.frame_number)
                pos = str(data.translation).replace('x: ', '').replace(
                    'y: ', '').replace('z: ', '').split(',')
                pos = [float(x) for x in pos]
                pos = self.arr_trans(pos)
                vel = str(data.velocity).replace('x: ', '').replace(
                    'y: ', '').replace('z: ', '').split(',')
                vel = [float(x) for x in pos]
                vel = self.arr_trans(vel)
                acc = str(data.acceleration).replace('x: ', '').replace(
                    'y: ', '').replace('z: ', '').split(',')
                acc = [float(x) for x in pos]
                acc = self.arr_trans(acc)
                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y
                quaternion = [w, x, y, z]
                pitch = -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi
                roll = m.atan2(2.0 * (w*x + y*z), w*w - x *
                               x - y*y + z*z) * 180.0 / m.pi
                yaw = m.atan2(2.0 * (w*z + x*y), w*w + x *
                              x - y*y - z*z) * 180.0 / m.pi
                euler = [pitch, roll, yaw]
                res = [id, pos, vel, acc, quaternion, euler]
                if (self.DEBUG):
                    print(res)
                return res
        except Exception as e:
            print('t265 data get error:', e)
            return None

    def stop_t265(self):
        """
        停止t265连接
        """
        self.pipe.stop()

    def fps_test(self):
        """
        测试t265的fps(500次)
        """
        ts = time.time()
        for i in range(500):
            self.get_raw_data()
        te = time.time()-ts
        fps = 500/te
        print('fps: ', fps)

    def plot_points(self, arr):
        """
        画出[x,y,z]的三维图像
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        points = np.array(arr)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        ax.scatter(x, y, z, c='r', marker='.')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def show_route_test(self, move_times):
        """
        获取并绘制t265的运动轨迹
        """
        output = []
        for i in range(move_times):
            ret_0 = self.get_raw_data()[0]
            if (ret_0 != None):
                ret_1 = self.get_raw_data()[1]
                output.append(ret_1)
            else:
                break
        self.plot_points(output)

    def low_pass_filter(self, arr):
        """
        低通滤波器
        """
        arr = np.array(arr)
        rows, cols = arr.shape
        # 创建一个与输入数组形状相同的零数组
        filtered_arr = np.zeros((rows, cols))
        # 对每个元素进行低通滤波
        for i in range(rows):
            for j in range(cols):
                # 获取当前元素周围的元素
                neighbors = arr[max(0, i-1):min(rows, i+2),
                                max(0, j-1):min(cols, j+2)]
                # 计算邻居元素的平均值，并将其赋值给过滤后的数组
                filtered_arr[i, j] = np.mean(neighbors)
        return filtered_arr.tolist()

    def kalman_filter(self, arr):
        """
        卡尔曼滤波器
        """
        arr = np.array(arr)
        rows, cols = arr.shape
        # 创建一个与输入数组形状相同的零数组，用于存储滤波后的结果
        filtered_arr = np.zeros((rows, cols))
        # 初始化卡尔曼滤波器参数
        Q = 0.1  # 过程噪声协方差
        R = 1.0  # 测量噪声协方差
        x = arr[0, :]  # 初始状态估计
        P = np.eye(cols)  # 初始状态协方差矩阵
        # 对每个时间步进行卡尔曼滤波
        for i in range(rows):
            # 预测步骤
            x = x
            P = P + Q
            # 更新步骤
            y = arr[i, :] - x
            S = P + R
            K = np.dot(P, np.linalg.inv(S))
            x = x + np.dot(K, y)
            P = np.dot((np.eye(cols) - K), P)
            # 将滤波结果存储到滤波后的数组中
            filtered_arr[i, :] = x
        return filtered_arr.tolist()

    def ekf(self, arr):
        """
        扩展卡尔曼滤波
        支持3维和4维list
        """
        # arr的维数
        deg = len(arr[0])
        # 初始化状态向量和协方差矩阵
        # 初始状态向量 [x, y, z] 或者 [w, x, y, z]
        if (deg == 3):
            x = np.array([0, 0, 0])
        if (deg == 4):
            x = np.array([0, 0, 0, 0])
        P = np.eye(deg)  # 初始协方差矩阵
        # 定义系统模型和测量模型
        F = np.eye(deg)  # 状态转移矩阵
        H = np.eye(deg)  # 观测矩阵
        # 定义过程噪声和测量噪声的协方差矩阵
        Q = np.eye(deg) * 0.1  # 过程噪声协方差矩阵
        R = np.eye(deg) * 0.1  # 测量噪声协方差矩阵
        filtered_arr = []
        for measurement in arr:
            # 预测步骤
            x_pred = np.dot(F, x)
            P_pred = np.dot(np.dot(F, P), F.T) + Q
            # 更新步骤
            y = measurement - np.dot(H, x_pred)
            S = np.dot(np.dot(H, P_pred), H.T) + R
            K = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))
            x = x_pred + np.dot(K, y)
            P = np.dot((np.eye(deg) - np.dot(K, H)), P_pred)
            filtered_arr.append(x.tolist())
        return filtered_arr

    def coordinate_transform(self, quaternion, euler):
        """
        将t265坐标系下的四元数和欧拉角转化为匿名坐标系下的四元数和欧拉角
        坐标系1:t265坐标系;坐标系2:匿名坐标系
        """
        # 坐标系1到坐标系2的旋转矩阵
        rotation_matrix = [[0, -1, 0],
                           [1, 0, 0],
                           [0, 0, 1]]
        # 坐标系1下的四元数
        w1, x1, y1, z1 = quaternion
        # 坐标系1下的欧拉角
        pitch1, roll1, yaw1 = euler
        # 将四元数转换为旋转矩阵
        rotation_matrix1 = [[1 - 2*y1*y1 - 2*z1*z1, 2*x1*y1 - 2*w1*z1, 2*x1*z1 + 2*w1*y1],
                            [2*x1*y1 + 2*w1*z1, 1 - 2*x1*x1 -
                                2*z1*z1, 2*y1*z1 - 2*w1*x1],
                            [2*x1*z1 - 2*w1*y1, 2*y1*z1 + 2*w1*x1, 1 - 2*x1*x1 - 2*y1*y1]]
        # 计算坐标系2下的旋转矩阵
        rotation_matrix2 = np.dot(rotation_matrix, rotation_matrix1)
        # 将旋转矩阵转换为四元数
        w2 = 0.5 * \
            np.sqrt(
                1 + rotation_matrix2[0][0] + rotation_matrix2[1][1] + rotation_matrix2[2][2])
        x2 = 0.5 * np.sign(rotation_matrix2[2][1] - rotation_matrix2[1][2]) * np.sqrt(
            abs(rotation_matrix2[0][0] - rotation_matrix2[1][1] - rotation_matrix2[2][2] + 1))
        y2 = 0.5 * np.sign(rotation_matrix2[0][2] - rotation_matrix2[2][0]) * np.sqrt(
            abs(rotation_matrix2[1][1] - rotation_matrix2[2][2] - rotation_matrix2[0][0] + 1))
        z2 = 0.5 * np.sign(rotation_matrix2[1][0] - rotation_matrix2[0][1]) * np.sqrt(
            abs(rotation_matrix2[2][2] - rotation_matrix2[0][0] - rotation_matrix2[1][1] + 1))
        # 将欧拉角转换为坐标系2下的欧拉角
        pitch2 = -np.arcsin(rotation_matrix2[2][0]) * 180.0 / np.pi
        roll2 = np.arctan2(
            rotation_matrix2[2][1], rotation_matrix2[2][2]) * 180.0 / np.pi
        yaw2 = np.arctan2(
            rotation_matrix2[1][0], rotation_matrix2[0][0]) * 180.0 / np.pi
        # 返回坐标系2下的四元数和欧拉角
        ret_quaternion = [w2, x2, y2, z2]
        ret_euler = [pitch2, roll2, yaw2]
        return [ret_quaternion, ret_euler]

    def complementary_filter(self, quaternion, euler, velocity, acceleration):
        """
        互补滤波:
        四元数: quaternion[w,x,y,z]
        欧拉角: euler[pitch,roll,yaw]
        速度: vel[v_x,v_y,v_z]
        加速度: acc[a_x,a_y,a_z]
        """
        # 互补滤波的权重
        alpha = 0.98
        # 四元数滤波
        quaternion_filtered = [0, 0, 0, 0]
        for i in range(4):
            quaternion_filtered[i] = alpha * \
                quaternion[i] + (1 - alpha) * euler[i]
        # 欧拉角滤波
        euler_filtered = [0, 0, 0]
        for i in range(3):
            euler_filtered[i] = alpha * euler[i] + \
                (1 - alpha) * quaternion[i+1]
        # 速度滤波
        velocity_filtered = [0, 0, 0]
        for i in range(3):
            velocity_filtered[i] = alpha * velocity[i] + \
                (1 - alpha) * acceleration[i]
        # 加速度滤波
        acceleration_filtered = [0, 0, 0]
        for i in range(3):
            acceleration_filtered[i] = alpha * \
                acceleration[i] + (1 - alpha) * velocity[i]
        return [quaternion_filtered, euler_filtered, velocity_filtered, acceleration_filtered]

    def autoset(self):
        """
        自动标定t265
        """
        if(self.first_flag == 1):
            print('t265 autoset start')
            self.first_flag = 0
            self.time_start = time.time()
            while True:
                if(time.time() - self.time_start > self.set_time):
                    raw_data = self.get_raw_data()
                    self.base_pos = raw_data[1]
                    self.base_yaw = self.get_actual_yaw(raw_data)
                    break
            print('t265 autoset end')
        else:
            pass
    
    def reset(self):
        """
        重新标定t265
        """
        self.first_flag = 1
        self.time_start = 0
        self.xy_route = []
        self.autoset()

    def get_actual_pos(self, raw_data):
        """
        获取t265的实际位置,已经自动标定
        返回值: [x,y,z],单位: cm
        """
        self.autoset()
        data_tmp = raw_data[1]
        res_1 = 100*(data_tmp[0] - self.base_pos[0])
        res_2 = 100*(data_tmp[1] - self.base_pos[1])
        res_3 = 100*(data_tmp[2] - self.base_pos[2])
        res = [round(res_1,3), round(res_2,3), round(res_3,3)]
        return res
    
    def get_actual_yaw(self, raw_data):
        """
        获取t265的实际yaw,未标定
        返回值: [x,y,z],单位: cm
        """
        data_euler = raw_data[5]
        data_quaternion = raw_data[4]
        res = self.coordinate_transform(data_quaternion, data_euler)
        ret_quaternion = res[0]
        ret_euler = res[1]
        yaw = ret_euler[2] - self.base_yaw
        return yaw
    
    def draw_xy_route(self, raw_data):
        """
        画平面轨迹
        """
        xyz = self.get_actual_pos(raw_data)
        tmp_x = xyz[0]
        tmp_y = xyz[1]
        tmp_z = xyz[2]
        # xy转化为opencv坐标系
        x = - tmp_y
        y = - tmp_x
        z = tmp_z
        ret_point = [x,y,z]
        self.xy_route.append(ret_point)
        image = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)
        for point in self.xy_route:
            x, y, z = point
            x = int(x)
            y = int(y)
            print('z',z)
            bias = int(self.map_size/2)
            cv2.circle(image, (x + bias, y + bias), 1, (255, 255, 255), -1)
        if(len(self.xy_route)>self.point_cache_size):
            self.xy_route = []
        return image,ret_point
    
    def normalize_angle(self, angle):
        normalized_angle = angle % 360.0
        if normalized_angle < 0:
            normalized_angle += 360.0
        if(normalized_angle > 180):
            normalized_angle = normalized_angle - 360
        return normalized_angle

    def get_pos(self):
        """
        返回自动标定后的t265位置和yaw,yaw是-180到180的角度
        """
        raw_data = self.get_raw_data()
        if(raw_data != None):
            pos = self.get_actual_pos(raw_data)
            yaw = self.get_actual_yaw(raw_data)
            yaw = self.normalize_angle(yaw)
            yaw = round(yaw,3)
            return [pos, yaw]
        else:
            return None

    def get_car_pos(self):
        """
        返回小车xy坐标和yaw轴(已经自动标定),yaw是-180到180的角度
        """
        ret = self.get_pos()
        if(ret != None):
            x = ret[0][0]
            y = ret[0][1]
            yaw = ret[1]
            return [x, y, yaw]
        else:
            return None
    def start_update(self):
        t265_thread=threading.Thread(target=self.t265_update)
        t265_thread.daemon=True
        t265_thread.start()
        self.update_running=True
        logger.info("t265自动更新线程启动")
    def t265_update(self):
        self.autoset()
        while self.update_running==True:
            self.t265_pose=self.get_car_pos()
            time.sleep(0.02)

