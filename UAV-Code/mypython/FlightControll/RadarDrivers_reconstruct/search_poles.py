# from RadarDrivers_reconstruct.Radar import Radar
# import Radar
import cv2
import numpy as np
from collections import defaultdict
import time


class radar_cv:
    res_cache = []
    hough_line_size = 140  # 霍夫直线宽度
    cut_size = 7    # 边缘黑色区域宽度(例如6代表每边切掉1/6)
    area_size = 20  # 划分格数,例如20代表将图像划分为20*20的格子
    aver_size = 30  # 合并圆心坐标时的阈值,例如30代表两个圆心坐标的x,y坐标差值都小于30时,将两个圆心坐标合并为一个坐标
    debug = False   # 是否开启debug模式

    def __init__(self) -> None:
        pass

    def extract_white(self, image):
        """
        提取图像中的白色部分
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        mask = np.zeros_like(image)
        # 将二值图像中的白色部分复制到掩膜图像中
        mask[binary == 255] = image[binary == 255]
        return mask

    def dilate_erode(self, image):
        """
        对图像进行膨胀和腐蚀操作(7,3)
        """
        # 膨胀
        kernel_dilate = np.ones((7, 7), np.uint8)
        # 腐蚀
        kernel_erode = np.ones((3, 3), np.uint8)
        dilated_image = cv2.dilate(image, kernel_dilate, iterations=2)
        eroded_image = cv2.erode(dilated_image, kernel_erode, iterations=1)
        return eroded_image

    def dilate_erode_res(self, image):
        """
        对图像进行膨胀和腐蚀操作(3,30)
        """
        kernel_dilate = np.ones((3, 3), np.uint8)
        kernel_erode = np.ones((30, 30), np.uint8)
        dilated_image = cv2.dilate(image, kernel_dilate, iterations=2)
        eroded_image = cv2.erode(dilated_image, kernel_erode, iterations=1)
        return eroded_image

    def dilate_erode_circle(self, image):
        """
        对图像进行膨胀和腐蚀操作(3,2)
        """
        kernel_dilate = np.ones((3, 3), np.uint8)
        kernel_erode = np.ones((2, 2), np.uint8)
        dilated_image = cv2.dilate(image, kernel_dilate, iterations=2)
        eroded_image = cv2.erode(dilated_image, kernel_erode, iterations=1)
        return eroded_image

    def get_intersection(self,line1, line2):
        """
        计算两条直线的交点
        """
        x1, y1 = line1[0]
        x2, y2 = line1[1]
        x3, y3 = line2[0]
        x4, y4 = line2[1]
        # 计算直线的斜率
        slope1 = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else float('inf')
        slope2 = (y4 - y3) / (x4 - x3) if x4 - x3 != 0 else float('inf')
        # 计算直线的截距
        intercept1 = y1 - slope1 * x1 if slope1 != float('inf') else x1
        intercept2 = y3 - slope2 * x3 if slope2 != float('inf') else x3
        # 计算交点的横坐标
        if slope1 == slope2:
            x = float('inf')
        elif slope1 == float('inf'):
            x = x1
        elif slope2 == float('inf'):
            x = x3
        else:
            x = (intercept2 - intercept1) / (slope1 - slope2)
        # 计算交点的纵坐标
        if slope1 == float('inf'):
            y = slope2 * x + intercept2
        else:
            y = slope1 * x + intercept1
        x = int(x)
        y = int(y)
        return [x, y]

    def sort_points_clockwise(self,points):
        """
        将点按照顺时针排序
        """
        # 计算每个点相对于中心点的极角
        center_x = sum(point[0] for point in points) / len(points)
        center_y = sum(point[1] for point in points) / len(points)
        angles = []
        for point in points:
            x, y = point
            angle = np.arctan2(y - center_y, x - center_x)
            angles.append(angle)
        # 按照极角对点进行排序
        sorted_points = [point for _, point in sorted(zip(angles, points))]
        return sorted_points


    def remove_lines(self,image):
            """
            去掉图像中的直线,直线围成四边形的外面
            """
            # 将图像转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # 使用Canny边缘检测算法进行边缘检测
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            # 运行Hough直线变换检测直线
            lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=30)
            mask = np.zeros_like(image)
            mask_1 = np.zeros_like(image)
            # 在掩膜图像上绘制检测到的直线
            try:
                point_left = []
                point_right = []
                point_up = []
                point_down = []
                for line in lines:
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    if((x1 < 500 and x2 < 500) and ((y1 < 500 and y2 > 500) or (y1 >500 and y2 <500))):
                        if(y1 > y2):
                            tmp = y1
                            y1 = y2
                            y2 = tmp
                            tmp = x1
                            x1 = x2
                            x2 = tmp
                        point_left.append([x1,y1,x2,y2])
                    if((x1 > 500 and x2 > 500) and ((y1 < 500 and y2 > 500) or (y1 >500 and y2 <500))):
                        if(y1 > y2):
                            tmp = y1
                            y1 = y2
                            y2 = tmp
                            tmp = x1
                            x1 = x2
                            x2 = tmp
                        point_right.append([x1,y1,x2,y2])
                    if((y1 < 500 and y2 < 500) and ((x1 < 500 and x2 > 500) or (x1 >500 and x2 <500))):
                        if(x1 > x2):
                            tmp = x1
                            x1 = x2
                            x2 = tmp
                            tmp = y1
                            y1 = y2
                            y2 = tmp
                        point_up.append([x1,y1,x2,y2])
                    if((y1 > 500 and y2 > 500) and ((x1 < 500 and x2 > 500) or (x1 >500 and x2 <500))):
                        if(x1 > x2):
                            tmp = x1
                            x1 = x2
                            x2 = tmp
                            tmp = y1
                            y1 = y2
                            y2 = tmp
                        point_down.append([x1,y1,x2,y2])
                left_aver = []
                right_aver = []
                up_aver = []
                down_aver = []
                x1 = 0
                y1 = 0
                x2 = 0
                y2 = 0
                for i in range(len(point_left)):
                    x1 += point_left[i][0]
                    y1 += point_left[i][1]
                    x2 += point_left[i][2]
                    y2 += point_left[i][3]
                x1 = int(x1/len(point_left))
                y1 = int(y1/len(point_left))
                x2 = int(x2/len(point_left))
                y2 = int(y2/len(point_left))
                left_aver = [x1,y1,x2,y2]
                x1 = 0
                y1 = 0
                x2 = 0
                y2 = 0
                for i in range(len(point_right)):
                    x1 += point_right[i][0]
                    y1 += point_right[i][1]
                    x2 += point_right[i][2]
                    y2 += point_right[i][3]
                x1 = int(x1/len(point_right))
                y1 = int(y1/len(point_right))
                x2 = int(x2/len(point_right))
                y2 = int(y2/len(point_right))
                right_aver = [x1,y1,x2,y2]
                x1 = 0
                y1 = 0
                x2 = 0
                y2 = 0
                for i in range(len(point_up)):
                    x1 += point_up[i][0]
                    y1 += point_up[i][1]
                    x2 += point_up[i][2]
                    y2 += point_up[i][3]
                x1 = int(x1/len(point_up))
                y1 = int(y1/len(point_up))
                x2 = int(x2/len(point_up))
                y2 = int(y2/len(point_up))
                up_aver = [x1,y1,x2,y2]
                x1 = 0
                y1 = 0
                x2 = 0
                y2 = 0
                for i in range(len(point_down)):
                    x1 += point_down[i][0]
                    y1 += point_down[i][1]
                    x2 += point_down[i][2]
                    y2 += point_down[i][3]
                x1 = int(x1/len(point_down))
                y1 = int(y1/len(point_down))
                x2 = int(x2/len(point_down))
                y2 = int(y2/len(point_down))
                down_aver = [x1,y1,x2,y2]
                lines = [left_aver,right_aver,up_aver,down_aver]
                arr = []
                for i in range(4):
                    line_tmp = lines[i]
                    x1 = line_tmp[0]
                    y1 = line_tmp[1]
                    x2 = line_tmp[2]
                    y2 = line_tmp[3]     
                    arr.append([[x1,y1],[x2,y2]])
                    cv2.line(mask, (x1, y1), (x2, y2), (255, 255, 255), 100)
                rect_point = []
                intersection1 = self.get_intersection(arr[0], arr[2])
                rect_point.append(intersection1)
                intersection2 = self.get_intersection(arr[0], arr[3])
                rect_point.append(intersection2)
                intersection3 = self.get_intersection(arr[1], arr[2])
                rect_point.append(intersection3)
                intersection4 = self.get_intersection(arr[1], arr[3])
                rect_point.append(intersection4)
                rect_point = self.sort_points_clockwise(rect_point)
                # 在掩膜上绘制四边形
                cv2.fillPoly(mask_1, [np.array(rect_point)], (255, 255, 255))
                mask_1 = cv2.bitwise_not(mask_1)
                # 应用掩膜到图像
                # 将直线从原始图像中去除
                result = cv2.subtract(image, mask)
                result = cv2.subtract(result, mask_1)
            except:
                result = image
            return result

    def blacken_edges(self, image):
        """
        去掉图像的边缘
        """
        height, width = image.shape[:2]
        edge_length = min(height, width) // self.cut_size
        result = np.copy(image)
        # 将边缘变成黑色
        result[:edge_length, :] = (0, 0, 0)
        result[-edge_length:, :] = (0, 0, 0)
        result[:, :edge_length] = (0, 0, 0)
        result[:, -edge_length:] = (0, 0, 0)
        return result

    def draw_res(self, image, size):
        """
        去除size*size的区域内重复的白点,只保留一个白点
        """
        height, width = image.shape[:2]
        region_width = width // size
        region_height = height // size
        result = np.zeros_like(image)
        # 遍历每个区域
        for i in range(size):
            for j in range(size):
                # 计算当前区域的横坐标范围和纵坐标范围
                x_start = i * region_width
                x_end = (i + 1) * region_width
                y_start = j * region_height
                y_end = (j + 1) * region_height
                # 在当前区域内查找白点
                white_points = np.where(
                    image[y_start:y_end, x_start:x_end] == 255)
                # 如果有白点，则计算白点的平均坐标
                if len(white_points[0]) > 0:
                    x_avg = int(np.mean(white_points[1]) + x_start)
                    y_avg = int(np.mean(white_points[0]) + y_start)
                    cv2.circle(result, (x_avg, y_avg), 3, (255, 255, 255), -1)
        return result

    def detect_circles(self, image):
        """
        霍夫圆检测
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 对灰度图进行模糊处理，以减少噪声对霍夫圆检测的影响
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # 进行霍夫圆检测
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,
            param2=10,
            minRadius=1,
            maxRadius=10,
        )
        # 如果没有检测到圆，则返回空列表
        if circles is None:
            return []
        # 将检测到的圆的坐标转换为整数
        circles = np.round(circles[0, :]).astype("int")
        # 在图像上标出检测到的圆
        for x, y, r in circles:
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(
                image,
                (x - r - 10, y - r - 10),
                (x + r + 10, y + r + 10),
                (0, 128, 255),
                2,
            )
            # 在图像上显示文本
            text = str([x, y])
            org = (x - r - 10, y - r - 30)  # 文本的起始坐标
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.4
            color = (0, 255, 0)  # 文本颜色为绿色
            thickness = 1
            # 在图像上绘制文本
            cv2.putText(image, text, org, fontFace,
                        fontScale, color, thickness)
        return [circles, image]

    def show_test_res(self, image):
        """
        返回未滤波的圆心坐标
        """
        res = self.extract_white(image)
        mask = res
        res = self.dilate_erode(res)
        res = self.remove_lines(res)
        res = self.blacken_edges(res)
        res = self.dilate_erode(res)
        res = self.dilate_erode(res)
        res = self.dilate_erode_res(res)
        res = self.draw_res(res, self.area_size)
        res = self.dilate_erode_circle(res)
        try:
            res_tmp = self.detect_circles(res)[0]
            try:
                res_cir_edit = []
                for i in range(len(res_tmp)):
                    res_cir_x = int(res_tmp[i][0])
                    res_cir_y = int(res_tmp[i][1])
                    res_cir = [res_cir_x, res_cir_y]
                    res_cir_edit.append(res_cir)
            except:
                res_cir_edit = []
                pass
            self.res_cache.append(res_cir_edit)
        except:
            res_cir_edit = []
        return res_cir_edit, mask

    def filt(self, img):
        """
        滤波圆心坐标
        """
        arr, mask = self.show_test_res(img)
        len_cache = len(self.res_cache)
        if len_cache >= 5:
            arr_tmp = self.res_cache[len_cache - 5: len_cache]
            self.res_cache.pop(0)
            flattened_arr = []
            for i in range(len(arr_tmp)):
                for j in range(len(arr_tmp[i])):
                    flattened_arr.append(arr_tmp[i][j])
            arr = flattened_arr
            point_counts = defaultdict(int)
            merged_points = []
            for i in range(len(arr)):
                x, y = arr[i]
                merged = False
                if i > 0:
                    prev_x, prev_y = arr[i - 1]
                    if abs(x - prev_x) < self.aver_size and abs(y - prev_y) < self.aver_size:
                        merged_x = (x + prev_x) / 2
                        merged_y = (y + prev_y) / 2
                        merged_points[-1] = [merged_x, merged_y]
                        point_counts[(merged_x, merged_y)] += 1
                        merged = True
                if not merged:
                    merged_points.append([x, y])
                    point_counts[(x, y)] += 1
            sorted_points = sorted(
                point_counts.items(), key=lambda x: x[1], reverse=True
            )
            ret = [list(point) for point in sorted_points[:2]]
            ret_edit = []
            for i in range(len(ret)):
                tmp0 = ret[i][0][0]
                tmp1 = ret[i][0][1]
                ret_edit.append([tmp0, tmp1])
            return ret_edit, mask
        else:
            ret_edit = []
            return ret_edit, mask

    def get_points_from_image(self, img):
        """
        返回图像中的白点坐标
        """
        points = []
        height, width = img.shape[:2]
        for i in range(height):
            for j in range(width):
                if np.any(img[i, j] == 255):  # 假设白色表示目标点
                    points.append([i, j])
        return points

    def acc_search(self, ret_img):
        """
        精确查找圆心坐标
        """
        cnt = 0
        while True:
            cnt += 1
            ret_edit, mask = self.filt(ret_img)
            len_ret_edit = len(ret_edit)
            if len_ret_edit == 2:
                point_1 = ret_edit[0]
                point_2 = ret_edit[1]
                pow_point_dis = (point_1[0]-point_2[0]
                                 )**2+(point_1[1]-point_2[1])**2
                point_dis = pow(pow_point_dis, 0.5)
                if point_dis > 80:
                    try:
                        area_1 = mask[point_1[1]-40:point_1[1] +
                                      40, point_1[0]-40:point_1[0]+40]
                        area_2 = mask[point_2[1]-40:point_2[1] +
                                      40, point_2[0]-40:point_2[0]+40]
                        area_1_points = self.get_points_from_image(area_1)
                        area_2_points = self.get_points_from_image(area_2)
                        aver_area_1 = list(np.mean(area_1_points, axis=0))
                        aver_area_1[0] += point_1[0]-40
                        aver_area_1[1] += point_1[1]-40
                        aver_area_2 = list(np.mean(area_2_points, axis=0))
                        aver_area_2[0] += point_2[0]-40
                        aver_area_2[1] += point_2[1]-40
                        aver_area_1 = [int(aver_area_1[0]),
                                       int(aver_area_1[1])]
                        aver_area_2 = [int(aver_area_2[0]),
                                       int(aver_area_2[1])]
                        res = [aver_area_1, aver_area_2]
                        return res, mask
                        break
                    except:
                        continue
                else:
                    if (cnt > 100):
                        return ret_edit, mask
                        break
            else:
                if (cnt > 100):
                    return ret_edit, mask
                    break

    def draw_final_res(self, img):
        """
        绘制最终结果,debug用
        """
        try:
            circles, mask = self.acc_search(img)
            image = np.zeros_like(mask)
            for x, y in circles:
                r = 2
                cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(
                    image,
                    (x - r - 10, y - r - 10),
                    (x + r + 10, y + r + 10),
                    (0, 128, 255),
                    2,
                )
                # 在图像上显示文本
                text = str([x, y])
                org = (x - r - 10, y - r - 30)  # 文本的起始坐标
                fontFace = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.4
                color = (0, 255, 0)  # 文本颜色为绿色
                thickness = 1
                # 在图像上绘制文本
                cv2.putText(image, text, org, fontFace,
                            fontScale, color, thickness)
            return circles, image
        except:
            circles, mask = self.acc_search(img)
            image = np.zeros_like(mask)
            return circles, image

    def get_final_res(self, img):
        if (self.debug):
            res = self.draw_final_res(img)
            pic = res[1]
            cv2.imshow("Result", pic)
            cv2.waitKey(1)
            time.sleep(0.05)
            return res
        else:
            res = self.acc_search(img)[0]
            return res

    def test_case(self, img, debug=False):
        """
        测试例程
        """
        # img = cv2.imread('radar_map.jpg')
        self.debug = debug
        if (self.debug):
            # for i in range(10):
            res = self.get_final_res(img)
            circle = res[0]
            # print(circle)
        else:
            for i in range(10):
                res = self.get_final_res(img)
                print(res)
        # cv2.destroyAllWindows()


# radar = Radar()
# radar.start('COM4')
# while True:
#     img = radar.output_cloud(scale=0.1, size=1000)
#     img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
#     test = radar_cv()
#     test.test_case(img, debug=True)
