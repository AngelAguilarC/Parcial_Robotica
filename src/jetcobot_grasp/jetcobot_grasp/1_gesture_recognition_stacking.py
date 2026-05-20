#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import rclpy
from rclpy.node import Node
import time
from jetcobot_grasp.GestureRecognition import handDetector
from jetcobot_grasp.grasp_controller import GraspController
import sys                                                                  
import signal

class GestureRecognitionStacking(Node):
    def __init__(self):
        super().__init__('gesture_recognition_stacking')
        self.hand_detector = handDetector(detectorCon=0.75)
        self.pTime = 0
        self.graspController = GraspController()
        self.graspController.init_pose()
        self.graspController.open_gripper()

        # 定义抓取方块的状态
        self.one_grabbed = 0
        self.two_grabbed = 0
        self.three_grabbed = 0
        self.four_grabbed = 0

        self.block_num = 0

        # 定义手势识别次数
        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Five = 0

    def ctrl_arm_move(self, index):
        if index >= 5:
            self.graspController.ctrl_nod()
            return
        self.graspController.goColorOverPose()
        if index == 1:
            self.graspController.goApriltag1fixedPose(2)
        elif index == 2:
            self.graspController.goApriltag2fixedPose(2)
        elif index == 3:
            self.graspController.goApriltag3fixedPose(2)
        elif index == 4:
            self.graspController.goApriltag4fixedPose(2)
        time.sleep(1)
        self.graspController.drop_gripper(1)
        self.graspController.close_gripper(1)
        self.graspController.ctrl_gripper_height(200, 1.5)
        # self.graspController.goColorOverPose()
        # time.sleep(1)
        self.graspController.goStackingOverPose()
        time.sleep(1)
        self.block_num = self.block_num + 1
        self.graspController.goStackingPose(str(self.block_num))
        time.sleep(1)
        self.graspController.open_gripper(1)
        self.graspController.ctrl_gripper_height(150+self.block_num*30, 1.5)
        self.graspController.init_pose()
        time.sleep(.5)
        

    def process(self, frame):
        try:
            frame, lmList = self.hand_detector.findHands(frame, draw=False)
            if len(lmList) != 0:
                gesture = self.hand_detector.get_gesture()
                self.get_logger().info("gesture = {}".format(gesture))
                cv.putText(frame, gesture, (250, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                if gesture == 'One':
                    self.Count_One = self.Count_One + 1
                    self.Count_Two = 0
                    self.Count_Three = 0
                    self.Count_Four = 0
                    self.Count_Five = 0
                    if self.Count_One >= 10 and self.one_grabbed == 0:
                        self.arm_ctrl_threading(gesture)
                        self.one_grabbed = 1
                elif gesture == 'Two':
                    self.Count_Two = self.Count_Two + 1
                    self.Count_Three = 0
                    self.Count_Four = 0
                    self.Count_Five = 0
                    if self.Count_Two >= 10 and self.two_grabbed == 0:
                        self.arm_ctrl_threading(gesture)
                        self.two_grabbed = 1
                elif gesture == 'Three':
                    self.Count_Three = self.Count_Three + 1
                    self.Count_Two = 0
                    self.Count_Four = 0
                    self.Count_Five = 0
                    if self.Count_Three >= 10 and self.three_grabbed == 0:
                        self.arm_ctrl_threading(gesture)
                        self.three_grabbed = 1
                elif gesture == 'Four':
                    self.Count_Four = self.Count_Four + 1
                    self.Count_Two = 0
                    self.Count_Three = 0
                    self.Count_Five = 0
                    if self.Count_Four >= 10 and self.four_grabbed == 0:
                        self.arm_ctrl_threading(gesture)
                        self.four_grabbed = 1
                elif gesture == 'Five':
                    self.Count_Five = self.Count_Five + 1
                    self.Count_One = 0
                    self.Count_Two = 0
                    self.Count_Three = 0
                    self.Count_Four = 0
                    if self.Count_Five >= 10:
                        self.arm_ctrl_threading(gesture)
                        self.one_grabbed = 0
                        self.two_grabbed = 0
                        self.three_grabbed = 0
                        self.four_grabbed = 0
                        self.Count_Five = 0
                        self.block_num = 0

            self.cTime = time.time()
            fps = 1 / (self.cTime - self.pTime)
            self.pTime = self.cTime
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (20, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

            return frame
        except BaseException as e:
            self.get_logger().info("process error = {}".format(e))
            return None


    def arm_ctrl_threading(self, gesture):
        self.get_logger().info("arm_ctrl_threading gesture = {}".format(gesture))
        if gesture == 'One':
            self.ctrl_arm_move(1)
        elif gesture == 'Two':
            self.ctrl_arm_move(2)
        elif gesture == 'Three':
            self.ctrl_arm_move(3)
        elif gesture == 'Four':
            self.ctrl_arm_move(4)
        elif gesture == 'Five':
            self.ctrl_arm_move(5)


def quit(signum, frame):
    print("sys.exit")
    sys.exit()

def main(args=None):
    rclpy.init(args=args)
    gestureRecognitionStacking = GestureRecognitionStacking()
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
            frame = gestureRecognitionStacking.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
    finally:
        capture.release()
        cv.destroyAllWindows()
        gestureRecognitionStacking.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()