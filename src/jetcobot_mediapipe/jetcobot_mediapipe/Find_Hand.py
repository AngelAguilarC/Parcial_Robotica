#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
from time import sleep, time
from pymycobot.mycobot import MyCobot
from jetcobot_mediapipe.media_library import *
import rclpy
from rclpy.node import Node

class HandCtrlArm(Node):
    def __init__(self):
        super().__init__('hand_ctrl_arm')
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.hand_detector = HandDetector()
        self.reset_pose()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.Joy_active = True
        self.event = threading.Event()
        self.event.set()
        self.Joy_active = True

    def reset_pose(self):
        self.mc.send_angles([0, 0, -90, 95, 0, -45], 50)
        sleep(1.5)

    def process(self, frame):
        frame = cv.flip(frame, 1)
        if self.Joy_active:
            frame, lmList, bbox = self.hand_detector.findHands(frame)
            if len(lmList) != 0 and self.Joy_active:
                threading.Thread(target=self.find_hand_threading, args=(lmList, bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame

    def find_hand_threading(self, lmList, bbox):
        fingers = self.hand_detector.fingersUp(lmList)
        self.hand_detector.draw = True
        angle = self.hand_detector.ThumbTOforefinger(lmList)
        value = np.interp(angle, [0, 70], [185, 20])
        indexX = (bbox[0] + bbox[2]) / 2
        indexY = (bbox[1] + bbox[3]) / 2
        print("index X: %.1f, Y: %.1f" % (indexX, indexY))

def main(args=None):
    rclpy.init(args=args)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    
    ctrl_arm = HandCtrlArm()
    
    try:
        while capture.isOpened():
            ret, frame = capture.read()
            action = cv.waitKey(1) & 0xFF
            frame = ctrl_arm.process(frame)
            if action == ord('q'):
                ctrl_arm.media_ros.cancel()
                break
            cv.imshow('frame', frame)
            rclpy.spin_once(ctrl_arm, timeout_sec=0.001)
    finally:
        capture.release()
        cv.destroyAllWindows()
        ctrl_arm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

