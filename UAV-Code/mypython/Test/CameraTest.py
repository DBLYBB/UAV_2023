import cv2
count = 0       # 定义拍照计数变量
def main():
    global count
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    while cap.isOpened():
        ret, img = cap.read()  # 视频读入
        if not ret:
            continue
        cv2.imshow('show', img)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            count =   count + 1
            cv2.imwrite("./testback" + str(count) + ".jpg", img)
            print("save success!  count =", count)


if __name__ == '__main__':
    main()

