import cv2
import numpy as np
from RadarDrivers_reconstruct.RadarMapApplication import radar_map_application


class radar_debug(radar_map_application):
    """
    雷达调试器
    """

    def __init__(self):
        super().__init__()

    def start(self, com_port: str, radar_type: str = "LD06") -> None:
        # 开始监听串口
        self.start_serial_task(com_port=com_port, radar_type=radar_type)
        # 开始地图解算
        self.start_pose_task()

    def init_radar_map(self):
        """
        初始化雷达地图
        """
        self.radar_map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        a = np.sqrt(2) * 600
        b = (a - 600) / 2
        c = a - b
        b = int(b / np.sqrt(2))
        c = int(c / np.sqrt(2))
        cv2.line(self.radar_map_img, (b, b), (c, c), (255, 0, 0), 1)
        cv2.line(self.radar_map_img, (c, b), (b, c), (255, 0, 0), 1)
        cv2.line(self.radar_map_img, (300, 0), (300, 600), (255, 0, 0), 1)
        cv2.line(self.radar_map_img, (0, 300), (600, 300), (255, 0, 0), 1)
        cv2.circle(self.radar_map_img, (300, 300), 100, (255, 0, 0), 1)
        cv2.circle(self.radar_map_img, (300, 300), 200, (255, 0, 0), 1)
        cv2.circle(self.radar_map_img, (300, 300), 300, (255, 0, 0), 1)
        self.radar_map_img_scale = 1
        self.radar_map_info_angle = -1
        cv2.namedWindow("Radar Map", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(
            "Radar Map",
            lambda *args, **kwargs: self.show_radar_map_on_mouse(*args, **kwargs),
        )

    def show_radar_map_on_mouse(self, event, x, y, flags):
        """
        雷达地图鼠标事件回调函数
        """
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.radar_map_img_scale *= 1.1
            else:
                self.radar_map_img_scale *= 0.9
            self.radar_map_img_scale = min(max(0.001, self.radar_map_img_scale), 2)
        elif event == cv2.EVENT_LBUTTONDOWN or (
            event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON
        ):
            self.radar_map_info_angle = (
                90 - np.arctan2(300 - y, x - 300) * 180 / np.pi
            ) % 360
            self.radar_map_info_angle = int(self.radar_map_info_angle)

    def show_radar_map(self):
        """
        显示雷达地图(调试用, 高占用且阻塞)
        """
        self.init_radar_map()
        while True:
            img = self.radar_map_img.copy()
            cv2.putText(
                img,
                f"{100 / self.radar_map_img_scale:.0f}",
                (300, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img,
                f"{200 / self.radar_map_img_scale:.0f}",
                (300, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img,
                f"{300 / self.radar_map_img_scale:.0f}",
                (300, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            add_p = self.find_two_point_with_given_distance(
                from_=-60,
                to_=60,
                distance=110,
                threshold=15,
                range_limit=3000,
            )
            print(add_p)
            if self.radar_map_info_angle != -1:
                cv2.putText(
                    img,
                    f"Angle: {self.radar_map_info_angle}",
                    (10, 540),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                cv2.putText(
                    img,
                    f"Distance: {self.get_distance(self.radar_map_info_angle)}",
                    (10, 560),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                point = self.get_point(self.radar_map_info_angle)
                xy = point.to_xy()
                cv2.putText(
                    img,
                    f"Position: ( {xy[0]:.2f} , {xy[1]:.2f} )",
                    (10, 580),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                add_p = [point] + add_p
                pos = point.to_cv_xy() * self.radar_map_img_scale + np.array([300, 300])
                cv2.line(img, (300, 300), (int(pos[0]), int(pos[1])), (255, 255, 0), 1)
            self.draw_on_cv_image(img, scale=self.radar_map_img_scale, add_points=add_p)
            cv2.imshow("Radar Map", img)
            key = cv2.waitKey(int(1000 / 50))
            if key == ord("q"):
                break
            elif key == ord("w"):
                self.radar_map_img_scale *= 1.1
            elif key == ord("s"):
                self.radar_map_img_scale *= 0.9
            elif key == ord("a"):
                out = self.output_cloud()
                cv2.imwrite(f"radar_map.png", out)
                cv2.imshow("Cloud", out)
