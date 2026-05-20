# !/usr/bin/env python
# coding: utf-8
import os
import cv2 as cv
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import math
import logging
from simple_pid import PID

class face_follow:
    def __init__(self):
        self.target_servox=0
        self.target_servoy=-90
        self.a = 0
        self.b = 0
        self.xservo_pid = PID(3.5, 0.1, 0.05)
        self.yservo_pid = PID(2, 0.05, 0.05)
        # Create an instance of OpenCV's joint classifier
        # 创建opencv的联级分类器的实例
        self.faceDetect = cv.CascadeClassifier("haarcascade_frontalface_default.xml")
        
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.mc.send_angles([0, 0, -90, 95, 0, -45], 50)
               # 日志
        self.logger = logging.getLogger()
        self.file_handler = logging.FileHandler('log_file.txt')
        self.logger.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        self.file_handler.setFormatter(formatter)
        self.logger.addHandler(self.file_handler)
        self.clear_log()
        
    def clear_log(self):
        with open('log_file.txt', 'w') as f:
            f.truncate()
        self.logger.info('Log file cleared')

    def face_filter(self, faces):
        '''
        Filter the face
        对人脸进行一个过滤
        '''
        if len(faces) == 0: return None
        # At present, we are looking for the face with the largest area in the pictur
        # 目前找的是画面中面积最大的人脸
        max_face = max(faces, key=lambda face: face[2] * face[3])
        (x, y, w, h) = max_face
        # Set the minimum threshold of face detection
        # 设置人脸检测最小阈值
        if w < 10 or h < 10: return None
        return max_face

    def follow_function(self, img):
        self.a = 0
        self.b = 0
        img = cv.resize(img, (640, 480))
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = img.copy()
        # Convert image to grayscale
        # 将图像转为灰度图
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Face detection
        # 检测人脸
        faces = self.faceDetect.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        if len(faces) != 0:
            face = self.face_filter(faces)
            # Face filtering
            # 人脸过滤
            (x, y, w, h) = face
            # Draw a rectangle on the original color map
            # 在原彩图上绘制矩形
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 4)
            cv.putText(img, 'Person', (280, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (105, 105, 105), 2)
            point_x = x + w / 2
            point_y = y + h / 2
            
            point_x = point_x / 640
            if abs(point_x - 0.5) > 0.02: # 相差范围小于一定值就不用再动了
                self.xservo_pid.setpoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                output = self.xservo_pid(point_x, dt=0.1)
                self.target_servox = min(max(self.target_servox + output, -160), 160)
            else:
                self.xservo_pid.reset() # 如果已经到达中心了就复位一下 pid 控制器

            point_y = point_y / 480
            if abs(point_y - 0.5) > 0.02:
                self.yservo_pid.setpoint = 0.5
                output = self.yservo_pid(point_y, dt=0.1)
                self.target_servoy = min(max(self.target_servoy + output, -140), 0)
            else:
                self.yservo_pid.reset()
            
            joints_0 = [self.target_servox, 0, self.target_servoy, -self.target_servoy+5, 0, -45]
            self.logger.info("joints_0 = {}".format(joints_0))
            self.mc.send_angles(joints_0, 50)
        return img
