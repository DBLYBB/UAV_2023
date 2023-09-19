import os

import cv2
import numpy as np


def nms(dets, nmsThreshold):
    """
    非极大值抑制
    dets: [[x1, y1, x2, y2, score], [x1, y1, x2, y2, score], ...]
    """
    # dets:N*M,N是bbox的个数，M的前4位是对应的（x1,y1,x2,y2），第5位是对应的分数
    # #thresh:0.3,0.5....
    if dets.shape[0] == 0:
        return []
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]
    scores = dets[:, 4]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # 求每个bbox的面积
    order = scores.argsort()[::-1]  # 对分数进行倒排序
    keep = []  # 用来保存最后留下来的bboxx下标
    while order.size > 0:
        i = order[0]  # 无条件保留每次迭代中置信度最高的bbox
        keep.append(i)
        # 计算置信度最高的bbox和其他剩下bbox之间的交叉区域
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        # 求交叉区域的面积占两者（置信度高的bbox和其他bbox）面积和的比例
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        # 保留ovr小于thresh的bbox，进入下一次迭代。
        inds = np.where(ovr <= nmsThreshold)[0]
        # 因为ovr中的索引不包括order[0]所以要向后移动一位
        order = order[inds + 1]
    output = []
    for i in keep:
        output.append(dets[i].tolist())
    return output


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def tanh(x):
    return 2.0 / (1 + np.exp(-2 * x)) - 1


def draw_pred(frame, class_name, conf, left, top, right, bottom):
    """
    绘制预测结果
    """
    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
    label = f"{class_name}: {conf:.2f}"
    labelSize, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    top = max(top - 10, labelSize[1])
    left = max(left, 0)
    cv2.putText(
        frame,
        label,
        (left, top),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        thickness=2,
    )


class QRScanner_Yolo(object):
    def __init__(self, confThreshold=0.6, nmsThreshold=0.5, drawOutput=False):
        """
        YoloV3 二维码识别
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        drawOutput: 是否在图像上画出识别结果
        """
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.inpWidth = 416
        self.inpHeight = 416
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        cfg_path = os.path.join(path, "qrcode-yolov3-tiny.cfg")
        weights_path = os.path.join(path, "qrcode-yolov3-tiny.weights")
        self.net = cv2.dnn.readNet(cfg_path, weights_path)
        self.drawOutput = drawOutput

    def post_process(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        confidences = []
        boxes = []
        centers = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    boxes.append([left, top, width, height])
                    confidences.append(float(confidence))
                    centers.append((center_x, center_y))
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        ret = [(centers[i], confidences[i]) for i in indices]
        if self.drawOutput:
            for i in indices:
                box = boxes[i]
                left = box[0]
                top = box[1]
                width = box[2]
                height = box[3]
                draw_pred(
                    frame,
                    "QRcode",
                    confidences[i],
                    left,
                    top,
                    left + width,
                    top + height,
                )
        return ret

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 置信度)
        """
        blob = cv2.dnn.blobFromImage(
            frame,
            1 / 255.0,
            (self.inpWidth, self.inpHeight),
            [0, 0, 0],
            swapRB=True,
            crop=False,
        )
        # 加载网络
        self.net.setInput(blob)
        # 前向传播
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        return self.__post_process(frame, outs)


class FastestDet:
    def __init__(self, confThreshold=0.5, nmsThreshold=0.4, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "FastestDet.onnx")
        self.classes = list(map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 500
        self.inpHeight = 500
        self.net = cv2.dnn.readNet(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.drawOutput = drawOutput

    def post_process(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        outs = outs.transpose(1, 2, 0)  # 将维度调整为 (H, W, C)
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        feature_height = outs.shape[0]
        feature_width = outs.shape[1]
        preds = []
        confidences = []
        boxes = []
        ret = []
        for h in range(feature_height):
            for w in range(feature_width):
                data = outs[h][w]
                obj_score, cls_score = data[0], data[5:].max()
                score = (obj_score**0.6) * (cls_score**0.4)
                if score > self.confThreshold:
                    classId = np.argmax(data[5:])
                    # 检测框中心点偏移
                    x_offset, y_offset = tanh(data[1]), tanh(data[2])
                    # 检测框归一化后的宽高
                    box_width, box_height = sigmoid(data[3]), sigmoid(data[4])
                    # 检测框归一化后中心点
                    box_cx = (w + x_offset) / feature_width
                    box_cy = (h + y_offset) / feature_height
                    x1, y1 = box_cx - 0.5 * box_width, box_cy - 0.5 * box_height
                    x2, y2 = box_cx + 0.5 * box_width, box_cy + 0.5 * box_height
                    x1, y1, x2, y2 = (
                        int(x1 * frameWidth),
                        int(y1 * frameHeight),
                        int(x2 * frameWidth),
                        int(y2 * frameHeight),
                    )
                    preds.append([x1, y1, x2, y2, score, classId])
                    boxes.append([x1, y1, x2 - x1, y2 - y1])
                    confidences.append(score)
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        for i in indices:
            pred = preds[i]
            score, classId = pred[4], int(pred[5])
            x1, y1, x2, y2 = pred[0], pred[1], pred[2], pred[3]
            center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
            ret.append(((center_x, center_y), self.classes[classId], score))
            if self.drawOutput:
                draw_pred(frame, self.classes[classId], score, x1, y1, x2, y2)
        return ret

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 类型名称, 置信度)
        """
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.inpWidth, self.inpHeight))
        self.net.setInput(blob)
        pred = self.net.forward(self.net.getUnconnectedOutLayersNames())[0][0]
        return self.post_process(frame, pred)


class FastestDetOnnx(FastestDet):
    """
    使用 onnxruntime 运行 FastestDet 目标检测网络
    """

    def __init__(self, confThreshold=0.6, nmsThreshold=0.2, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        import onnxruntime

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "FastestDet.onnx")
        self.classes = list(map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 500
        self.inpHeight = 500
        self.session = onnxruntime.InferenceSession(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.drawOutput = drawOutput

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 类型名称, 置信度)
        """
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.inpWidth, self.inpHeight))
        input_name = self.session.get_inputs()[0].name
        feature_map = self.session.run([], {input_name: blob})[0][0]
        return self.post_process(frame, feature_map)


class HAWP(object):
    """
    使用 onnxruntime 运行 HAWP 线框检测
    """

    def __init__(self, confThreshold=0.95, drawOutput=False):
        """
        HAWP 线框检测网络
        confThreshold: 置信度阈值
        """
        import onnxruntime

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_onnx = os.path.join(path, "HAWP.onnx")

        self.onnx_session = onnxruntime.InferenceSession(path_onnx)

        self.input_name = self.onnx_session.get_inputs()[0].name
        self.output_name = self.onnx_session.get_outputs()[0].name

        self.input_shape = self.onnx_session.get_inputs()[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)

        self.confThreshold = confThreshold
        self.drawOutput = drawOutput

    def pre_process(self, frame):
        """
        图像预处理
        """
        frame = cv2.resize(frame, dsize=(self.input_width, self.input_height))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = (frame.astype(np.float32) / 255.0 - self.mean) / self.std
        frame = frame.transpose(2, 0, 1)
        frame = np.expand_dims(frame, axis=0)
        return frame

    def post_process(self, frame, feature_map):
        """ 
        数据后处理
        """
        lines, scores = feature_map[0], feature_map[1]
        image_width, image_height = frame.shape[1], frame.shape[0]
        for line in lines:
            line[0] = int(line[0] / 128 * image_width)
            line[1] = int(line[1] / 128 * image_height)
            line[2] = int(line[2] / 128 * image_width)
            line[3] = int(line[3] / 128 * image_height)
        output = []
        for n in range(len(lines)):
            if scores[n] > self.confThreshold:
                output.append((lines[n], scores[n]))
        if self.drawOutput:
            for line, score in output:
                x1, y1 = int(line[0]), int(line[1])
                x2, y2 = int(line[2]), int(line[3])
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return output

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: ((x1,y1,x2,y2),score)
        """
        blob = self.pre_process(frame)
        feature_map = self.onnx_session.run(None, {self.input_name: blob})
        return self.post_process(frame, feature_map)
