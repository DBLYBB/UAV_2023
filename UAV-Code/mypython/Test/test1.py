
# coding=utf-8
import cv2
import numpy as np
 
img = cv2.imread('/home/pi/Documents/python/line.png')
 
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
gaus = cv2.GaussianBlur(gray,(3,3),0)
 
edges = cv2.Canny(gaus, 50, 150, apertureSize=3)
 
minLineLength = 100
maxLineGap = 10
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength, maxLineGap)
 
for x1, y1, x2, y2 in lines[0]:
    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
 
cv2.imshow("houghline",img)
cv2.waitKey()
cv2.destroyAllWindows()

 