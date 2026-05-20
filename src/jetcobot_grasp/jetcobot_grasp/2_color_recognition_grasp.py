#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import rclpy
from rclpy.node import Node
import threading
from jetcobot_grasp.grasp_controller import GraspController
from jetcobot_utils.color_recognition import ColorRecognition
from jetcobot_utils.jetcobot_config import *

import sys
import time
import signal


class Color_Grasp(Node):
    def __init__(self):
        super().__init__('color_grasp_node')
        self.event = threading.Event()
        self.event.set()
        self.pTime = 0
        self.colorRecognition = ColorRecognition()
        self.graspController = GraspController()
        self.color_hsv = {"red": ((0, 103, 172), (2, 255, 255)),
                          "green": ((54, 109, 78), (77, 255, 255)),
                          "blue": ((92, 100, 62), (121, 251, 255)),
                          "yellow": ((26, 100, 91), (32, 255, 255))}
        # HSV参数路径  HSV Parameter path
        self.HSV_path = "/home/jetson/jetcobot_ws/src/jetcobot_color_identify/scripts/HSV_config.txt"

        # 定义抓取方块的状态
        self.yellow_grabbed = 0
        self.red_grabbed = 0
        self.green_grabbed = 0
        self.blue_grabbed = 0
        self.num = 0
        self.status = 'waiting'
        self.last_color = None

        read_HSV(self.HSV_path, self.color_hsv)
        self.graspController.init_pose()


    def grasp_run(self, color_name):
        self.graspController.ctrl_nod()
        if color_name == 'yellow':
            self.graspController.goYellowfixedPose()
        elif color_name == 'red':
            self.graspController.goRedfixedPose()
        elif color_name == 'green':
            self.graspController.goGreenfixedPose()
        elif color_name == 'blue':
            self.graspController.goBluefixedPose()
        else:
            return
        self.graspController.drop_gripper(1.5)
        self.graspController.close_gripper(1.5)
        self.graspController.rise_gripper(1.5)
        self.graspController.goColorOverPose()
        self.graspController.goBoxCenterlayer1Pose()
        self.graspController.open_gripper(1)
        self.graspController.rise_gripper(1)
        self.graspController.init_pose()
        self.status = 'waiting'


    def process(self, frame):
        if self.status == 'waiting':
            frame, color = self.colorRecognition.get_all_color(frame, self.color_hsv)
            if len(color) == 0:
                color_name = None
            else:
                color_name = color[0]
            if len(color) == 1 and color_name == self.last_color:
                self.num += 1
                self.get_logger().info("color = {} , num = {}".format(color_name,self.num))
                if self.num > 10 and self.status == 'waiting':
                    self.get_logger().info("grap color = {} ".format(color_name))
                    self.status = "running"
                    self.grasp_run(color_name)
                    self.num = 0
            else:
                self.num = 0
                self.last_color = color_name

        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame


def quit(signum, frame):
    print("sys.exit")
    sys.exit()


def main(args=None):
    rclpy.init(args=args)
    color_grasp = Color_Grasp()
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    try:
        while capture.isOpened() and rclpy.ok():
            ret, frame = capture.read()
            action = cv.waitKey(1) & 0xFF
            frame = color_grasp.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
    finally:
        capture.release()
        cv.destroyAllWindows()
        color_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
