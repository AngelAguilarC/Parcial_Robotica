#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import rclpy
from rclpy.node import Node
from jetcobot_grasp.grasp_controller import GraspController
from jetcobot_utils.color_recognition import ColorRecognition
from jetcobot_utils.apriltag_identify import ApriltagIdentify
from jetcobot_utils.jetcobot_config import *
import sys
import time
import signal


class Aprtag_Grasp(Node):
    def __init__(self):
        super().__init__('Aprtag_Grasp_node')
        self.pTime = 0
        self.graspController = GraspController()
        self.apriltagIdentify = ApriltagIdentify()

        # 定义抓取方块的状态
        self.apriltag1_grabbed = 0
        self.apriltag2_grabbed = 0
        self.apriltag3_grabbed = 0
        self.apriltag4_grabbed = 0
        self.num = 0
        self.status = 'waiting'

        self.graspController.init_watch_pose()

    def grasp_run(self,tagId):
        self.graspController.goBoxCenterlayer1Pose(1)
        self.graspController.close_gripper(1.5)
        self.graspController.goColorOverPose()
        if tagId == '1':
            self.graspController.goApriltag1fixedPose(2)
        elif tagId == '2':
            self.graspController.goApriltag2fixedPose(2)
        elif tagId == '3':
            self.graspController.goApriltag3fixedPose(2)
        elif tagId == '4':
            self.graspController.goApriltag4fixedPose(2)
        else:
            self.graspController.init_watch_pose()
            self.status = 'waiting' 
            return
        time.sleep(1)
        self.graspController.drop_gripper(1.5)
        self.graspController.open_gripper(1)
        self.graspController.rise_gripper(1)
        self.graspController.init_watch_pose()
        self.status = 'waiting'     

    def process(self, frame):
        if self.status == 'waiting':
            frame, tagId = self.apriltagIdentify.getSingleApriltagID(frame)
            if len(tagId) > 0:
                self.num += 1
                if self.num % 10 == 0:
                    self.status = "running"
                    self.grasp_run(tagId)
                    self.num = 0

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
    aprtag_grasp = Aprtag_Grasp()
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
            frame = aprtag_grasp.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
    finally:
        capture.release()
        cv.destroyAllWindows()
        aprtag_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()