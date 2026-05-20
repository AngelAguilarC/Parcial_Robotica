import cv2
import numpy as np
import os
import yaml
#打开摄像头
 
 
cap =cv2.VideoCapture(0)
 
font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
flag = cap.isOpened()
cap.set(3, 3840)
cap.set(4, 2160)
cap.set(6, cv2.VideoWriter.fourcc(*'MJPG'))
cv2.namedWindow('frame', cv2.WINDOW_FREERATIO)
cv2.namedWindow('frame1', cv2.WINDOW_FREERATIO)
###加载标定参数yaml
###加载文件路径###
file_path = ("./4k标定参数.yaml")
###加载文件路径###
 
with open(file_path, "r") as file:
    parameter = yaml.load(file.read(), Loader=yaml.Loader)
    mtx = parameter['camera_matrix']
    dist = parameter['dist_coeff']
    camera_u = parameter['camera_u']
    camera_v = parameter['camera_v']
    mtx = np.array(mtx)
    dist = np.array(dist)
 
i=0
 
while 1:
    (grabbed, frame) = cap.read()
 
    h1, w1 = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame1 = dst1
 
    cv2.imshow('frame',frame)
    cv2.imshow('frame1',frame1)
 
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        u = str(i)
        firename=str('./photo/img'+u+'.jpg')
        cv2.imwrite(firename, img)
        print('写入：',firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break