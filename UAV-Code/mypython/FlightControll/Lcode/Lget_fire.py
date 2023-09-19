import cv2 as cv
import numpy as np
# from Lpid import PID3
import time
class cam_test:
    debug = True
    arr = []
    filt_cnt = 3
    filt_dis = 9000
    area_min = 450
    area_max = 1600
    
    def __init__(self) -> None:
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv.CAP_PROP_FPS, 30)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)

    def find_largest_contour_center(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        edges = cv.Canny(gray, 50, 150)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv.contourArea)
            area = cv.contourArea(max_contour)
            if(self.debug == True):
                pass
                #print(area)
            if self.area_max > area > self.area_min:  # 添加判断条件，只返回面积大于self.area的轮廓中点坐标
                M = cv.moments(max_contour)
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                cv.drawContours(image, [max_contour], -1, (0, 255, 0), 2)
                return image, center_x, center_y
        return image, None, None

    def find_laser_point(self, img):
        frame = img
        h_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_red = np.array([0, 20, 210])
        upper_red = np.array([50, 255, 240])
        mask = cv.inRange(h_img, lower_red, upper_red)
        res = cv.bitwise_and(img, h_img, mask=mask)
        kernel = np.ones((9, 9), np.uint8)
        res = cv.dilate(res, kernel)
        kernel = np.ones((3, 3), np.uint8)
        res = cv.erode(res, kernel)
        kernel = np.ones((9, 9), np.uint8)
        res = cv.dilate(res, kernel)
        kernel = np.ones((3, 3), np.uint8)
        res = cv.erode(res, kernel)
        res = self.find_largest_contour_center(res)
        img_res = res[0]
        loc = [res[1], res[2]]
        if(self.debug == True):
            cv.imshow('res', img_res)
        return img_res, loc
    
    def get_fire_loc(self,img):
        img_res, loc = self.find_laser_point(img)
        if loc[0] is not None and loc[1] is not None:
            if(len(self.arr) <5):
                self.arr.append(loc)
            elif(len(self.arr)==5):
                cnt = 0
                for i in range(5):
                    dis = (loc[0] - self.arr[i][0])**2 + (loc[1] - self.arr[i][1])**2
                    if(dis <= self.filt_dis):
                        cnt += 1
                self.arr.append(loc)
                self.arr.pop(0)
                if(cnt >= self.filt_cnt):
                    return loc
        else:
            return None


if __name__ == "__main__":
    cam = cam_test()
    cam.debug = True
#     x_pid=PID3(0,240)
#     y_pid=PID3(0,320)
    while True:
        ret, img = cam.cap.read()
        res = cam.get_fire_loc(img)
        if(res != None):
#             x_speed=x_pid.get_pid(res[1])
#             y_speed=y_pid.get_pid(res[0])
            print(res)
#             print(x_speed)
#             print(y_speed)
            cv.waitKey(1)
        time.sleep(0.02)
