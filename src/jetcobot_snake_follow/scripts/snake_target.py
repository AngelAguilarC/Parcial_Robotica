# !/usr/bin/env python
# coding: utf-8
import sys
import os
import cv2 as cv
import numpy as np
import math
from simple_PID import IncrementalPID
import time
import math
import threading
from jetcobot_utils.grasp_controller import GraspController

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord


class Snake_Target:
    def __init__(self):
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        time.sleep(2)
        self.coords = [80, -60, 300, -95, -44, -85]
        self.mc.send_coords(self.coords, 40, 0)
        self.snake_clamp = False
        self.snake_count = 0
        self.pid = IncrementalPID(0.01, 0, 0.001)
        self.pid.set_target(7000)
        self.pid.set_limit_output(80, 230)
        self.snake_check = 0
        self.grasp_controller = GraspController()

    def Image_Processing(self, img):
        '''
        Morphological transformation to remove small interference factors
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像      Enter the initial image
        :return: 检测的轮廓点集(坐标)  Detected contour point set (coordinates)
        '''
        # Convert image to grayscale
        # 将图像转为灰度图
        gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # Get structuring elements of different shapes
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # morphological closure
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # Image Binarization Operation
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # Get the set of contour points (coordinates)
        # 获取轮廓点集(坐标)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        return contours

    def get_area(self, hsv_name, hsv_range):
        (lowerb, upperb) = hsv_range
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        color_mask = self.image.copy()
        # Convert image to HSV
        # 将图像转换为HSV
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # filter out elements between two arrays
        # 筛选出位于两个数组之间的元素
        color = cv.inRange(hsv_img, lowerb, upperb)
        # Set the non-mask detection part to be all black
        # 设置非掩码检测部分全为黑色
        color_mask[color == 0] = [0, 0, 0]
        # cv.imshow("mask", color_mask)
        contours = self.Image_Processing(color_mask)
        # Contour drawing by polygon approximation
        # 采用多边形逼近的方法绘制轮廓
        for i, cnt in enumerate(contours):
            # Calculate the moment of a polygon
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0: continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # Get the center of the polygon
            # 获取多边形的中心
            (x, y) = (np.int32(cx), np.int32(cy))
            # Calculate the area of ​​the contour
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # Area greater than 800
            # ⾯积⼤于300
            if area > 300:
                # drawing center
                
                # Calculate the smallest rectangular area
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # get box vertices
                # 获取盒⼦顶点
                box = cv.boxPoints(rect)
                # Convert to long type
                # 转成long类型
                box = np.int0(box)
                # 绘制中⼼
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                # draw the smallest rectangle
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, hsv_name, (int(box[1][0] - 15), int(box[1][1]) - 15), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return area
        return None

    def target_run(self, img, color_hsv, color_name=None):
        # self.image = cv.resize(img, (640, 480), )
        self.image = img
        # Traverse the color channels to get recognizable results
        # 遍历颜色通道,获取能够识别的结果
        msg = {}
        if color_name is None:
            for key, value in color_hsv.items():
                area = self.get_area(key, value)
                if area != None: msg[key] = area
        else:
            area = self.get_area(color_name, color_hsv.get(color_name))
            if area != None: msg[color_name] = area
        self.snake_count = self.snake_count + 1
        if self.snake_count >= 300:
            if not self.snake_clamp:
                threading.Thread(target=self.snake_control_grip).start()
            self.snake_count = 0
        return self.image, msg


    def snake_control(self, name, msg):
        for key, area in msg.items():
            if key == name:
                # x = round(self.pid.calculate(math.sqrt(area)), 2)
                x = round(self.pid.calculate(area), 2)
                coords = [230-x+80, -60, 300, -95, -44, -85]
                # coords = [230-x+80, -60, 300, -90, 50, -90]
                print("x=", 230-x+80)
                if not self.snake_clamp:
                    self.snake_check = self.snake_check + 1
                    if 230-x+80 >= 220:
                        if self.snake_check > 5:
                            self.snake_grip_block(name)
                            self.snake_check = 0
                    else:
                        self.mc.send_coords(coords, 100, 1)
                        self.snake_check = 0


    def snake_control_grip(self):
        for i in range(3):
            if not self.snake_clamp:
                self.mc.set_gripper_value(10, 100)
                time.sleep(1)
            if not self.snake_clamp:
                self.mc.set_gripper_value(90, 100)
                time.sleep(1)

    def snake_grip_turn(self):
        self.mc.send_angle(6, -90, 50)
        time.sleep(1)
        self.mc.send_angle(6, 0, 50)
        time.sleep(1)
        self.mc.send_angle(6, -43, 50)
        time.sleep(1.5)
        self.mc.set_gripper_value(100, 100)
        time.sleep(1)
    
    def snake_grip_block(self, name):
        self.snake_clamp = True
        time.sleep(1)
        self.snake_grip_turn()
        self.grasp_controller.close_gripper(2)
        self.grasp_controller.goColorSortingPose(name)
        time.sleep(1)
        self.grasp_controller.open_gripper(1.2)
        self.mc.send_coord(Coord.Z.value, 200, 50)
        time.sleep(0.4)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        time.sleep(2)
        self.mc.send_coords(self.coords, 40, 0)
        time.sleep(2)
        self.snake_clamp = False

    def snake_set_pid_parm(self, P, I, D):
        self.pid.set_pid_param(P, I, D)
        
