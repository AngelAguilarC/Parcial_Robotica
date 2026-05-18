#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np
from time import sleep
import math
import logging

class identify_GetTarget:
    def __init__(self):
        self.image = None
        self.color_name = None
        self.color_status = True

        # 日志
        self.logger = logging.getLogger()
        self.file_handler = logging.FileHandler('log_file.txt')
        self.logger.setLevel(logging.INFO)
        formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - %(message)s')
        self.file_handler.setFormatter(formatter)
        self.logger.addHandler(self.file_handler)
        self.clear_log()
    
    def clear_log(self):
        with open('log_file.txt', 'w') as f:
            f.truncate()
        self.logger.info('Log file cleared')

    def select_color(self, image, color_hsv, color_list):
        '''
        Choose a recognition color
        选择识别颜色
        :param image:输入图像  input image
        :param color_hsv: HSV的范围阈值  Range threshold for HSV
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]  Color sequence: ['0': None '1': Red '2': Green '3': Blue '4': Yellow]
        :return: 输出处理后的图像,(颜色,位置)  Output the processed image, (color, position)
        '''
        # canonical input image size
        # 规范输入图像大小
        self.image = cv.resize(image, (640, 480))
        msg = {}
        if len(color_list) == 0: return self.image, msg
        if '4' in color_list:
            self.color_name = color_list['4']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '3' in color_list:
            self.color_name = color_list['3']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '2' in color_list:
            self.color_name = color_list['2']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '1' in color_list:
            self.color_name = color_list['1']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        return self.image, msg


    def get_Sqaure(self, color_hsv):
        '''
        Color recognition, get the coordinates of the square
        颜色识别,获得方块的坐标
        '''
        try:
            (lowerb, upperb) = color_hsv
            # Copy the original image to avoid interference during processing
            # 复制原始图像,避免处理过程中干扰
            mask = self.image.copy()
            # Convert image to HSV
            # 将图像转换为HSV
            HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
            # filter out elements between two arrays
            # 筛选出位于两个数组之间的元素
            img = cv.inRange(HSV_img, lowerb, upperb)
            # Set the non-mask detection part to be all black
            # 设置非掩码检测部分全为黑色
            mask[img == 0] = [0, 0, 0]
            # Get structuring elements of different shapes
            # 获取不同形状的结构元素
            kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
            # morphological closure
            # 形态学闭操作
            dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
            # Convert image to grayscale
            # 将图像转为灰度图
            dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
            # Image Binarization Operation
            # 图像二值化操作
            ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
            # Get the set of contour points (coordinates)
            # 获取轮廓点集(坐标)
            find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if len(find_contours) == 3: contours = find_contours[1]
            else: contours = find_contours[0]

            #---------------------add---------------------------
            # 找出最大轮廓
            c = max(contours, key = cv.contourArea)
            # 计算轮廓面积
            area = math.fabs(cv.contourArea(c))
            # 根据轮廓大小判断是否进行下一步处理
            rect = cv.minAreaRect(c)  # 获取最小外接矩形
            corners = np.int64(cv.boxPoints(rect))  # 获取最小外接矩形的四个角点
            # 在原始图像上绘制最小外接矩形
            cv.drawContours(self.image, [corners], 0, (255, 0, 0), 3)
            # 打印旋转角度
            yaw = rect[2]
            # self.logger.info("yaw = {}".format(yaw))
            #---------------------add---------------------------

            for i, cnt in enumerate(contours):
                x, y, w, h = cv.boundingRect(cnt)
                area = cv.contourArea(cnt)
                if area > 1000:
                    point_x = float(x + w / 2)
                    point_y = float(y + h / 2)
                    # self.logger.info("point_x = {}, point_y = {}".format(point_x, point_y))
                    # cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
                    cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                            cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    # Calculate the position of the block in the image
                    # 计算方块在图像中的位置
                    (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000)*0.7+0.15, 5))
                    # self.logger.info("a = {}, b = {}, yaw = {}".format(a, b, yaw))
                    return (a, b, yaw)
        except Exception as e:
            self.logger.info("get_Sqaure error ={} ".format(e))
            return None