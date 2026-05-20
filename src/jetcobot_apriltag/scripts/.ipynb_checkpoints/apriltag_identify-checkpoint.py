#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import time
from time import sleep
from dt_apriltags import Detector
from jetcobot_utils.vutils import draw_tags
import logging
import jetcobot_utils.logger_config as logger_config
from jetcobot_advance.fps import FPS
from pymycobot.mycobot import MyCobot

class ApriltagIdentify:
    def __init__(self):
        logger_config.setup_logger()
        self.fps = FPS() 
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.image = None
        self.reset_pose()
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

    def reset_pose(self):
        self.mc.send_angles([0, 0, -90, 95, 0, -45], 50)
        sleep(1.5)

    def getApriltagPosMsg(self, image):
        self.image = cv.resize(image, (640, 480))
        msg = {}
        try:
            tags = self.at_detector.detect(cv.cvtColor(
                self.image, cv.COLOR_RGB2GRAY), False, None, 0.025)
            tags = sorted(tags, key=lambda tag: tag.tag_id)
            if len(tags) > 0:
                for tag in tags:
                    point_x = tag.center[0]
                    point_y = tag.center[1]
                    (a, b) = (round(((point_x - 320) / 4000), 5),
                              round(((480 - point_y) / 3000) * 0.7 + 0.15, 5))
                    msg[tag.tag_id] = (a, b)

                self.image = draw_tags(self.image, tags, corners_color=(
                    0, 0, 255), center_color=(0, 255, 0))
        except Exception as e:
            logging.info('getApriltagPosMsg e = {}'.format(e))

        return self.image, msg

    def getSingleApriltagID(self, image):
        self.image = cv.resize(image, (640, 480))
        tagId = ""
        try:
            tags = self.at_detector.detect(cv.cvtColor(
                self.image, cv.COLOR_RGB2GRAY), False, None, 0.025)
            tags = sorted(tags, key=lambda tag: tag.tag_id)
            if len(tags) == 1:
                tagId = str(tags[0].tag_id)
                self.image = draw_tags(self.image, tags, corners_color=(
                    0, 0, 255), center_color=(0, 255, 0))
        except Exception as e:
            logging.info('getSingleApriltagID e = {}'.format(e))

        return self.image, tagId


if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    tag_identify = ApriltagIdentify()

    prev_time = 0
    fps_list = []
    num_frames = 10
    try:
        while capture.isOpened():
            action = cv.waitKey(10) & 0xFF
            if action == ord('q'):
                break
            ret, img = capture.read()
            img, data = tag_identify.getApriltagPosMsg(img)
            tag_identify.fps.update()
            tag_identify.fps.show_fps(img)
            cv.imshow('img', img)
    except:
        pass
    capture.release()
    cv.destroyAllWindows()