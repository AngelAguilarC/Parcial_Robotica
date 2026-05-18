#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from dt_apriltags import Detector
from pymycobot.mycobot import MyCobot
from simple_pid import PID
from jetcobot_advance.fps import FPS
from jetcobot_advance.vutils import draw_tags

class TagTrackingNode(Node):
    def __init__(self):
        super().__init__('apriltag_tracking')

        self.target_servox = 0
        self.target_servoy = -90
        self.xservo_pid = PID(3.5, 0.1, 0.05)
        self.yservo_pid = PID(2, 0.05, 0.05)

        self.tracker = None
        self.enable_select = False

        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.fps = FPS() 
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.mc.send_angles([0, 0, -90, 90, 0, -45], 50)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera")
            return
 
        self.process_frames_loop()

    def process_frames_loop(self):
        while rclpy.ok() and self.cap.isOpened():
            self.process_frame()

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Unable to read frame")
            return

        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result_image = np.copy(rgb_image)

        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id)
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        self.get_logger().info(f"len(tags)={len(tags)}")

        if len(tags) == 1:
            center_x, center_y = tags[0].center
            center_x = center_x / rgb_image.shape[1]
            if abs(center_x - 0.5) > 0.02:
                self.xservo_pid.setpoint = 0.5
                output = self.xservo_pid(center_x, dt=0.1)
                self.target_servox = min(max(self.target_servox + output, -160), 160)
            else:
                self.yservo_pid.reset()

            center_y = center_y / rgb_image.shape[0]
            if abs(center_y - 0.5) > 0.02:
                self.yservo_pid.setpoint = 0.5
                output = self.yservo_pid(center_y, dt=0.1)
                self.target_servoy = min(max(self.target_servoy + output, -140), 0)
            else:
                self.yservo_pid.reset()

            joints_0 = [self.target_servox, 0, self.target_servoy, -self.target_servoy, 0, -45]
            self.get_logger().info(f"joints_0 = {joints_0}")
            self.mc.send_angles(joints_0, 50)

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("result_image", result_image)
        key = cv2.waitKey(1)
        if key != -1:
            self.destroy_node()
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tag_tracking = TagTrackingNode()
    rclpy.spin(tag_tracking)
    tag_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
