# !/usr/bin/env python
# coding: utf-8
import os
import cv2 as cv
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import logging
import math
from simple_pid import PID

class color_follow:
    def __init__(self):
        self.img = None
        self.target_servox=0
        self.target_servoy=-90
        self.xservo_pid = PID(2, 0.05, 0.05)
        self.yservo_pid = PID(2, 0.05, 0.05)
        self.b = 0
        self.a = 0
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

    def follow_function(self, img, HSV_config):
        self.b = 0
        self.a = 0
        (color_lower, color_upper) = HSV_config
        self.img = cv.resize(img, (640, 480), )
        self.img = cv.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (5, 5), 0)
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(cnts) > 0:
            try:
                cnt = max(cnts, key=cv.contourArea)
                (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
                if color_radius > 10:
                    # Mark the detected color with the prototype coil
                    # 将检测到的颜色用原形线圈标记出来``
                    cv.circle(self.img, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 3)
                    color_x = color_x / 640
                    if abs(color_x - 0.5) > 0.03: # 相差范围小于一定值就不用再动了
                        self.xservo_pid.setpoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                        output = self.xservo_pid(color_x, dt=0.1)
                        self.target_servox = min(max(self.target_servox + output, -160), 160)
                    else:
                        self.xservo_pid.reset() # 如果已经到达中心了就复位一下 pid 控制器

                    color_y = color_y / 480
                    if abs(color_y - 0.5) > 0.03:
                        self.yservo_pid.setpoint = 0.5
                        output = self.yservo_pid(color_y, dt=0.1)
                        self.target_servoy = min(max(self.target_servoy + output, -140), 0)
                    else:
                        self.yservo_pid.reset()
                
                    joints_0 = [self.target_servox, 0, self.target_servoy, -self.target_servoy+5, 0, -45]
                    self.logger.info("joints_0 = {}".format(joints_0))
                    self.mc.send_angles(joints_0, 50)
            except Exception as e:
                print("error = ",e)
            
        return self.img

    def get_hsv(self, img):
        '''
        Get the range of HSV in a region
        获取某一区域的HSV的范围
        :param img: 彩色图    color image
        :return: 图像和HSV的范围   Image and HSV range
        '''
        print("okok")
        H = [];S = [];V = []
        img = cv.resize(img, (640, 480), )
        # Convert color image to HSV
        # 将彩色图转成HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # draw a rectangle
        # 画矩形框
        point_init_x=290
        point_init_y=300
        point_end_x=350
        point_end_y=360
        cv.rectangle(img, (point_init_x, point_init_y), (point_end_x, point_end_y), (0, 255, 0), 2)
        # Take out the H, S, V values ​​of each row and column in turn and put them into the container
        # 依次取出每行每列的H,S,V值放入容器中
        for i in range(point_init_x, point_end_x):
            for j in range(point_init_y, point_end_y):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # Calculate the maximum and minimum of H, S, and V respectively
        # 分别计算出H,S,V的最大最小
        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)
        # HSV range adjustment
        # HSV范围调整
        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        S_max = 255;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:cv.putText(img, txt1, (230, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:cv.putText(img, txt2, (270, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

    def learning_follow(self, img, HSV_config):
        img = self.follow_function(img, HSV_config)
        return img
    
    def close_fileHandler(self):
        # 关闭FileHandler
        self.file_handler.close()
