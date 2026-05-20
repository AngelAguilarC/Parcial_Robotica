#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
from dt_apriltags import Detector
from jetcobot_utils.vutils import draw_tags
import logging
import jetcobot_utils.logger_config as logger_config

class ApriltagIdentify:
    def __init__(self):
        logger_config.setup_logger()
        self.image = None
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)



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
                            round(((480 - point_y) / 3000) * 0.8+0.18, 5))
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
                tagId =str(tags[0].tag_id)
                self.image = draw_tags(self.image, tags, corners_color=(
                    0, 0, 255), center_color=(0, 255, 0))
        except Exception as e:
            logging.info('getSingleApriltagID e = {}'.format(e))
        
        return self.image, tagId