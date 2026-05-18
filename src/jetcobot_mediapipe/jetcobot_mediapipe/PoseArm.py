#!/usr/bin/env python3
# encoding: utf-8
import os
import threading
import numpy as np
from time import sleep, time
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import rclpy
from rclpy.node import Node
from jetcobot_mediapipe.media_library import *

class PoseCtrlArm(Node):
    def __init__(self):
        super().__init__('pose_ctrl_arm')
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        self.media_ros = Media_ROS()
        self.reset_pose()
        self.event = threading.Event()
        self.event.set()
    
    def reset_pose(self):
        self.mc.send_angles([0, 0, 0, 0, 0, -45], 50)
        sleep(1.5)

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, pointArray, lhandptArray, rhandptArray = self.pose_detector.findHolistic(frame)
        threading.Thread(target=self.arm_ctrl_threading, args=(pointArray, lhandptArray, rhandptArray)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame
    
    def get_angle(self, v1, v2):
        angle = np.dot(v1, v2) / (np.sqrt(np.sum(v1 * v1)) * np.sqrt(np.sum(v2 * v2)))
        angle = np.arccos(angle) / 3.14 * 180
        cross = v2[0] * v1[1] - v2[1] * v1[0]
        if cross < 0:
            angle = - angle
        return angle

    def get_pos(self, keypoints):
        str_pose = ""
        # 计算左臂与水平方向的夹角 Calculate the angle between the left arm and the horizontal
        keypoints = np.array(keypoints)
        v1 = keypoints[12] - keypoints[11]
        v2 = keypoints[13] - keypoints[11]
        angle_left_arm = self.get_angle(v1, v2)
        # 计算右臂与水平方向的夹角 Calculate the angle between the right arm and the horizontal
        v1 = keypoints[11] - keypoints[12]
        v2 = keypoints[14] - keypoints[12]
        angle_right_arm = self.get_angle(v1, v2)
        # 计算左肘的夹角 Calculate the angle of the left elbow
        v1 = keypoints[11] - keypoints[13]
        v2 = keypoints[15] - keypoints[13]
        angle_left_elow = self.get_angle(v1, v2)
        # 计算右肘的夹角 Calculate the angle of the right elbow
        v1 = keypoints[12] - keypoints[14]
        v2 = keypoints[16] - keypoints[14]
        angle_right_elow = self.get_angle(v1, v2)
        
        if 90<angle_left_arm<120 and -120<angle_right_arm<-90:
            str_pose = "NORMAL"
        elif 90<angle_left_arm<120 and 90<angle_right_arm<120:
            # 左手放下，举起右手 Put your left hand down and raise your right hand
            str_pose = "RIGHT_UP"
        elif -120<angle_left_arm<-90 and -120<angle_right_arm<-90:
            # 右手放下，举起左手 Put your right hand down and raise your left hand
            str_pose = "LEFT_UP"
        elif -120<angle_left_arm<-90 and 90<angle_right_arm<120:
            # 手上向上 Hands up
            str_pose = "ALL_HANDS_UP"
        elif 130<angle_left_arm<150 and -150<angle_right_arm<-130 and 90<angle_left_elow<120 and -120<angle_right_elow<90:
            # 双手叉腰 Hands on hips
            str_pose = "AKIMBO"
        elif -150<angle_left_arm<-120 and 120<angle_right_arm<150 and -85<angle_left_elow<-55 and 55<angle_right_elow<85:
            # 双手合成三角形 Make a triangle with both hands
            str_pose = "TRIANGLE"
        # print("str_pose = ",str_pose)
        # print("angle_left_arm = ",angle_left_arm,"\tangle_right_arm = ",angle_right_arm)
        # print("angle_left_elow = ",angle_left_elow,"\tangle_right_elow = ",angle_right_elow)
        return str_pose

    def arm_ctrl_threading(self, pointArray, lhandptArray, rhandptArray):
        keypoints = ['' for i in range(33)]
        if self.event.is_set():
            self.event.clear()
            if len(pointArray) != 0:
                for i in range(len(pointArray)):
                    keypoints[i] = (pointArray[i][1],pointArray[i][2])
                
                str_pose = self.get_pos(keypoints)
                if str_pose:
                    print("str_pose = ",str_pose)
                if str_pose=="RIGHT_UP":
                    self.RIGHT_UP()
                elif str_pose=="LEFT_UP":
                    self.LEFT_UP()
                elif str_pose=="ALL_HANDS_UP":
                    self.ALL_HANDS_UP()
                elif str_pose=="TRIANGLE":
                    self.TRIANGLE()
                elif str_pose=="AKIMBO":
                    self.AKIMBO()
                self.event.set()
            else:
                self.event.set()
                
    def RIGHT_UP(self):
        self.mc.send_angles([90, 80, 0, -90, -90, -45], 50)
        sleep(3)
        self.reset_pose()
        
    def LEFT_UP(self):
        self.mc.send_angles([-90, 80, 0, -90, 90, -45], 50)
        sleep(3)
        self.reset_pose()
        
    def ALL_HANDS_UP(self):
        self.mc.send_angles([0, 0, 0, 80, 0, -45], 50)
        sleep(3)
        self.reset_pose()
        
    def TRIANGLE(self):
        self.mc.send_angles([0, 90, -120, 50, 0, -45], 50)
        sleep(3)
        self.reset_pose()

    def AKIMBO(self):
        self.mc.send_angles([0, 90, -120, -20, 0, -45], 50)
        sleep(3)
        self.reset_pose()   

# Define the main function
def main(args=None):
    rclpy.init(args=args)
    pose_ctrl_arm = PoseCtrlArm()
    capture = cv.VideoCapture("/dev/video0")
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    try:
        while capture.isOpened() and rclpy.ok():
            ret, frame = capture.read()
            if ret:
                frame = pose_ctrl_arm.process(frame)
                cv.imshow('frame', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        capture.release()
        cv.destroyAllWindows()
        pose_ctrl_arm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()