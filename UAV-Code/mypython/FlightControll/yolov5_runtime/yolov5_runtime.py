import cv2
import onnxruntime
import time
import numpy as np

class Detector():
    names = 'category.names' # names文件
    weights = 'best.onnx' # 模型权重
    threshold = 0.5 # 置信度阈值
    iou_thres = 0.45 # nms阈值
    img_size = 640 # 图片尺寸
    DEBUG = False # 开启debug会实时显示图片
    dml = False # 开启dml加速

    def __init__(self):
        super(Detector, self).__init__()
        self.stride = 1
        self.init_model()
        with open(self.names, 'r') as f:
            class_names = f.read().splitlines()
        self.names = class_names

    def letterbox(self,img, new_shape=(img_size, img_size), auto=False, scaleFill=False, scaleUp=True):
        shape = img.shape[:2]  # current shape[height,width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleUp:
            r = min(r, 1.0)  # 确保不超过1
        ration = r, r  # width,height 缩放比例
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, 64), np.mod(dh, 64)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ration = new_shape[1] / shape[1], new_shape[0] / shape[0]
        # 均分处理
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))  # 添加边界
        return img, ration, (dw, dh)

    def clip_coords(self,boxes, img_shape):
        boxes[:, 0].clip(0, img_shape[1])  # x1
        boxes[:, 1].clip(0, img_shape[0])  # y1
        boxes[:, 2].clip(0, img_shape[1])  # x2
        boxes[:, 3].clip(0, img_shape[0])  # x2

    def scale_coords(self,img1_shape, coords, img0_shape, ratio_pad=None):
        if ratio_pad is None:  # 从img0_shape中计算
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain=old/new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]
        coords[:, [0, 2]] -= pad[0]  # x padding
        coords[:, [1, 3]] -= pad[1]  # y padding
        coords[:, :4] /= gain
        self.clip_coords(coords, img0_shape)
        return coords

    def init_model(self):
        if self.dml:
            sess = onnxruntime.InferenceSession(self.weights,providers=['DmlExecutionProvider','CPUExecutionProvider'])
        else:
            sess = onnxruntime.InferenceSession(self.weights,providers=['CPUExecutionProvider'])  # 加载模型权重
        self.input_name = sess.get_inputs()[0].name  # 获得输入节点
        output_names = []
        for i in range(len(sess.get_outputs())):
            output_names.append(sess.get_outputs()[i].name)  # 所有的输出节点
        self.output_name = sess.get_outputs()[0].name  # 获得输出节点的名称
        self.m = sess

    def preprocess(self, img):
        img0 = img.copy()
        img = self.letterbox(img, new_shape=self.img_size)[0]  # 图片预处理
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img).astype(np.float32)
        img /= 255.0
        img = np.expand_dims(img, axis=0)
        assert len(img.shape) == 4
        return img0, img

    def detect(self, im):
        img0, img = self.preprocess(im)
        pred = self.m.run(None, {self.input_name: img})[0]  # 执行推理
        pred = pred.astype(np.float32)
        pred = np.squeeze(pred, axis=0)
        boxes = []
        classIds = []
        confidences = []
        for detection in pred:
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID] * detection[4]  # 置信度为类别的概率和目标框概率值得乘积
            if confidence > self.threshold:
                box = detection[0:4]
                (centerX, centerY, width, height) = box.astype("int")
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                boxes.append([x, y, int(width), int(height)])
                classIds.append(classID)
                confidences.append(float(confidence))
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.threshold, self.iou_thres)  # 执行nms算法
        pred_boxes = []
        pred_confes = []
        pred_classes = []
        if len(idxs) > 0:
            for i in idxs.flatten():
                confidence = confidences[i]
                if confidence >= self.threshold:
                    pred_boxes.append(boxes[i])
                    pred_confes.append(confidence)
                    pred_classes.append(classIds[i])
        return im, pred_boxes, pred_confes, pred_classes

    def runtime(self,image):
        """
        ret = [class_name,confidence,(loc_x,loc_y)]\n
        没有检测到目标时,返回空列表\n
        开启DEBUG模式时,会显示图片
        """
        shape = (self.img_size, self.img_size)
        img, pred_boxes, pred_confes, pred_classes = self.detect(image)
        ret = []
        if len(pred_boxes) > 0:
            for i, _ in enumerate(pred_boxes):
                box = pred_boxes[i]
                left, top, width, height = box[0], box[1], box[2], box[3]
                box = (left, top, left + width, top + height)
                box = np.squeeze(
                    self.scale_coords(shape, np.expand_dims(box, axis=0).astype("float"), img.shape[:2]).round(), axis=0).astype(
                    "int")  # 进行坐标还原
                x0, y0, x1, y1 = box[0], box[1], box[2], box[3]
                arr_x_tmp = int((x0+x1)/2)
                arr_y_tmp = int((y0+y1)/2)
                pred_classes[i] = self.names[pred_classes[i]]
                ret_tmp = [pred_classes[i],round(pred_confes[i],3),(arr_x_tmp,arr_y_tmp)]
                if(pred_confes[i] >= self.threshold):
                    ret.append(ret_tmp)
                if self.DEBUG:
                    # 执行画图函数
                    cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 255), thickness=2)
                    cv2.putText(image, '{0}  {1:.2f}'.format(pred_classes[i], pred_confes[i]), (x0, y0 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thickness=1)
        if self.DEBUG:
            cv2.imshow("detector", image)
            cv2.waitKey(1)
        return ret
    
    def test_fps(self,image):
        print("test fps start")
        start = time.time()
        for i in range(100):
            self.runtime(image)
        end = time.time()
        fps = round(100/(end-start),3)
        print("fps:",fps)


class camera:
    cap = None
    width = 640
    height = 480
    fps = 60
    cam_id = 1 # 摄像头id

    def __init__(self) -> None:
        try:
            self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except:
            print("摄像头异常")

    def get_frame(self):
        try:
            if not self.cap.isOpened():
                print("无法打开摄像头")
                return None
            ret, frame = self.cap.read()
            if not ret:
                print("无法获取图像帧")
                return None
            return frame
        except:
            print("摄像头异常")
            return None
    
    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    
# test demo
""" if __name__=="__main__":
    test = Detector()
    test.DEBUG = True
    test.dml = False
    cam = camera()
    start_time = time.time()
    frame_count = 0
    while True:
        frame = cam.get_frame()
        res = test.runtime(frame)
        if(res != []):
            print(res)
        frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time > 1:  # 每隔一秒钟输出一次帧数
            fps = frame_count / elapsed_time
            print("FPS:", round(fps, 2))
            frame_count = 0
            start_time = current_time
 """
# img = cv2.imread("img2-67.jpg")
# test.test_fps(img)
