#!/usr/bin/python3
#coding=utf8
import os
import cv2
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import SetBool
from pymycobot.mycobot import MyCobot
import fps
import queue


class DepthImageNode:
    def __init__(self):
        rospy.init_node('depth_pseudocolor', anonymous=True)
        self.fps = fps.FPS()

        # self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        # self.mc.send_angles([0, 0, -90, 90, 0, -45], 50)
        
        rospy.wait_for_service('/camera/set_ldp')
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', RosImage, queue_size=1)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', RosImage, queue_size=1)
        rospy.ServiceProxy('/camera/set_ldp', SetBool)(False)
    
        # 同步时间戳, 时间允许有误差在0.02s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 2, 0.02)
        sync.registerCallback(self.multi_callback) 
        self.queue = queue.Queue(maxsize=1)
        rospy.loginfo('start depth_pseudocolor...')

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image))

    def image_proc(self):
        ros_rgb_image, ros_depth_image = self.queue.get(block=True)
        try:
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            # depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # 对彩色图像进行高斯滤波，以去除噪声
            color = cv2.GaussianBlur(bgr_image, (5, 5), 0)

            # 对深度图像进行中值滤波，以去除噪声
            depth = cv2.medianBlur(depth_image, 5)

            # 对彩色图像进行灰度化，以便于分割
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            # 对灰度图像进行二值化，以提取方块的轮廓
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # 对二值图像进行开运算，以去除小的噪点
            kernel = np.ones((3, 3), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            # 对二值图像进行轮廓检测，以提取方块的轮廓
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 假设只有一个方块，取第一个轮廓为方块的轮廓
            cnt = contours[0]

            # 对方块的轮廓进行多边形拟合，以得到方块的四个角点
            epsilon = 0.015 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            print(approx.shape)

            # 对方块的四个角点进行排序，以得到左上、右上、右下、左下的顺序
            approx = approx.reshape(4, 2)
            rect = np.zeros((4, 2), dtype="float32")
            s = approx.sum(axis=1)
            rect[0] = approx[np.argmin(s)]
            rect[2] = approx[np.argmax(s)]
            diff = np.diff(approx, axis=1)
            rect[1] = approx[np.argmin(diff)]
            rect[3] = approx[np.argmax(diff)]

            # 计算方块的长和宽（像素值）
            (tl, tr, br, bl) = rect
            widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
            widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
            heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
            heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
            maxWidth = max(int(widthA), int(widthB))
            maxHeight = max(int(heightA), int(heightB))

            # 计算方块的正视图的四个角点
            dst = np.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]], dtype="float32")

            # 计算方块的透视变换矩阵
            M = cv2.getPerspectiveTransform(rect, dst)

            # 对彩色图像进行透视变换，以得到方块的正视图
            warped = cv2.warpPerspective(color, M, (maxWidth, maxHeight))

            # 对深度图像进行透视变换，以得到方块的正视图
            warped_depth = cv2.warpPerspective(depth, M, (maxWidth, maxHeight))

            # 计算方块的平均深度值（像素值）
            mean_depth = cv2.mean(warped_depth)[0]

            # 假设深度相机的内参已知，根据像素值计算方块的实际尺寸（毫米值）
            # 这里使用的是Intel Realsense D435i的内参，您需要根据您的深度相机的内参进行修改
            fx = 616.344 # 焦距
            cx = 321.886 # 主点横坐标
            cy = 238.183 # 主点纵坐标
            # 计算方块的实际长和宽（毫米值）
            real_width = maxWidth * mean_depth / fx
            real_height = maxHeight * mean_depth / fx
            # 计算方块的实际深度（毫米值）
            real_depth = mean_depth
            # 计算方块的体积（立方厘米值）
            volume = real_width * real_height * real_depth / 1000

            # 显示结果
            print("The width of the cube is {:.2f} mm".format(real_width))
            print("The height of the cube is {:.2f} mm".format(real_height))
            print("The depth of the cube is {:.2f} mm".format(real_depth))
            print("The volume of the cube is {:.2f} cm^3".format(volume))
            
            cv2.imshow("depth_image", depth_image)
            key = cv2.waitKey(1)
            if key != -1:
                rospy.signal_shutdown('KeyboardInterrupt')

        except Exception as e:
            rospy.logerr('error:', str(e))
            

if __name__ == "__main__":
    try:
        node = DepthImageNode()
        while not rospy.is_shutdown():
            node.image_proc()
    except KeyboardInterrupt as e:
        rospy.loginfo('KeyboardInterrupt')

