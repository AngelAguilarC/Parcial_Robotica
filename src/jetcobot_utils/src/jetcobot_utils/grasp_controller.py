#!/usr/bin/env python3
# encoding: utf-8
import time
import os
from time import sleep
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot.genre import Coord
import logging
import jetcobot_utils.logger_config as logger_config

class GraspController:
    def __init__(self):
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        # 日志
        logger_config.setup_logger()
        self.func_start = False
        self.garbage_num = [0, 0, 0, 0]

    # 机械臂初始位置
    def init_pose(self):
        self.mc.send_angles([0, 0, 0, 0, 0, -45], 50)
        sleep(2)

    # 机械臂初始位置2
    def init_pose2(self):
        self.mc.send_angles([0, 0, -90, 95, 0, -45], 50)
        sleep(2)

    # 运动到watch位姿,打开夹爪
    def init_watch_pose(self):
        self.go_watch_position(2)
        logging.info("go_watch_position!")
        self.open_gripper(2)

    # 打开夹爪
    def open_gripper(self, delay=0):
        self.mc.set_gripper_value(100, 50)
        if delay > 0:
            sleep(delay)
    
    # 夹紧积木块
    def close_gripper(self, delay=0):
        self.mc.set_gripper_value(20, 50)
        if delay > 0:
            sleep(delay)
    
    def rotate_gripper(self, yaw, delay=0):
        self.mc.send_angle(Angle.J6.value, yaw, 50)
        sleep(2)

    def go_watch_position(self, delay=0):
        # self.mc.send_angles([42, 0, 0, -76, -7, 0], 50)
        self.mc.send_angles([42, 0, 0, -85, -7, -3], 50)
        if delay > 0:
            sleep(delay)

    def go_calibration_angles(self, angles):
        self.mc.send_angles(angles, 70)

    def go_angles(self, angles, delay=0):
        self.mc.send_angles(angles, 40)
        if delay > 0:
            sleep(delay)
    
    def go_radians(self, radians, delay=2):
        self.mc.send_radians(radians, 40)
        if delay > 0:
            sleep(delay)

    def go_coords(self, coords, delay=0):
        self.mc.send_coords(coords, 40, 0)
        if delay > 0:
            sleep(delay)

    def grasp_color(self, msg):
        if len(msg) != 1:
            return
        speed = 40
        self.go_coords(coords_init, 3)
        
        self.mc.send_coord(Coord.Z.value, 115, speed)
        sleep(1)
        # 合拢夹爪
        self.close_gripper(1.5)
        
        # 上升到upper位
        self.mc.send_coord(Coord.Z.value, 180, speed)
        sleep(2)

        # 过度位
        self.goOverPose(task, type)
        
        # 去物体位
        self.goTargetPose("sorting", "color", 1, name)
        
        # 打开夹爪
        self.open_gripper(1)
        
        # 提起夹爪
        self.lift_gripper("sorting", "color", 1, name)
        self.mc.send_coord(Coord.Z.value, 180, speed)

        # 过度位
        self.goOverPose(task, type) 

        self.init_watch_pose()
        
    
    def grasp_state(self):
        return self.func_start

    # 根据任务类型获取XY偏移量, 
    def grasp_get_offset_xy(self, task, type, originX, originY):
        offset_x = -0.012
        offset_y = 0.0005
        if type == "garbage":
            offset_x = -0.012
            offset_y = 0.0002
        elif type == "apriltag":
            offset_x = -0.012
            offset_y = 0.0005
        elif type == "color":
            offset_x = -0.012
            offset_y = 0.0005
        return offset_x, offset_y
    
    def grasp_run(self, task, type, msg, joint1456=None):
        if len(msg) == 0:
            logging.info("未识别到物块！")
            return
        move_num = 1
        self.func_start = True
        self.garbage_num = [0, 0, 0, 0]
        for name, pos in msg.items():
            try:
                yaw = 0
                # 只有color有yaw
                if type == "color":
                    yaw = pos[2]
                logging.info("name = {}, pos[0] = {}, pos[1] = {} , yaw = {}".format(name, pos[0], pos[1], yaw))
                if pos[1] > 0.27652:
                    logging.info("目标位姿不可达！")
                    continue

                # 偏移位。夹爪偏移夹不住时可调节
                offsetx, offsety = self.grasp_get_offset_xy(task, type, pos[1], -pos[0])
                x = (pos[1] + offsetx)*1000
                y = (-pos[0] + offsety)*1000
                self.grasp(task, type, str(move_num), str(name), yaw, x, y)
                move_num += 1
            except Exception as e:
                logging.info("grasp_run error = {}".format(e))

        joints_angles = [joint1456[0], 0, 0, joint1456[1], joint1456[2], joint1456[3]]
        self.go_angles(joints_angles, 2)
        logging.info("回到标定完之后的watch pose")
        self.func_start = False

    """
    执行抓取任务
    task: 任务呢类型:sorting(分拣) stacking(堆叠)
    type: 抓取的物品类型:garbage, color, apriltag
    move_num: 堆叠使用，表示第几个方块
    name: 垃圾名称，或者颜色名称，或者二维码识别数字
    yaw: 姿态旋转角度
    rx=-175.05, ry=-1.73, rz=-43.63: 初始姿态
    speed: 运动速度
    """
    def grasp(self, task, type, move_num, name, yaw, x, y, z=170, rx=-175, ry=0, rz=-45, speed=40):
        logging.info(str(name))
        coords_init = [x, y, z, rx, ry, rz]
        # upper位
        sleep(1)
        logging.info("1 upper位")
        self.go_coords(coords_init, 3)
        
        # 旋转角,在能获取到第6关节初始角度的时候进行
        # sleep(1)
        # init_angles = self.mc.get_angles()
        # if len(init_angles)==6:
        #     init_yaw = init_angles[5]
        #     logging.info("init_yaw = {}".format(init_yaw))
        #     self.rotate_gripper(yaw+init_yaw)
        #     logging.info("2 转角度 = {}".format(yaw+init_yaw))
        # 抓取位
        logging.info("3 下降到抓取位")
        self.mc.send_coord(Coord.Z.value, 115, speed)
        sleep(1)

        # 合拢夹爪
        logging.info("4 合拢夹爪")
        self.close_gripper(1.5)
        
        # 上升到upper位
        logging.info("5 上升到upper位")
        self.mc.send_coord(Coord.Z.value, 170, speed)
        sleep(2)

        # 过度位
        logging.info("6 到过度位")
        self.goOverPose(task, type)
        
        # 去物体位
        logging.info("7 去物体位")
        self.goTargetPose(task, type, move_num, name)
        
        # # 打开夹爪
        logging.info("8 打开夹爪 放置物品")
        self.open_gripper(1)
        
        # 提起夹爪
        logging.info("9 提起夹爪")
        self.lift_gripper(task, type, move_num, name)
        # self.mc.send_coord(Coord.Z.value, 200, speed-10)

        # 过度位
        logging.info("10 到过度位")
        self.goOverPose(task, type) 
        return True

    """
    过度位
    task: sorting(分拣) stacking(堆叠)
    type: garbage, color, apriltag
    """
    def goOverPose(self, task, type):
        if task == "sorting":
            if type == "garbage":
                self.goGarbageOverPose()
            elif type == "color" or type == "apriltag":
                self.goColorOverPose()
        elif task == "stacking":
            self.goStackingOverPose()
    
    def goStackingOverPose(self):
        over_radians = [-0.7266840000000001, -0.0005739999999998524, -0.2904439999999999, -0.6905220000000001, -7.670890735462309e-05, -0.7722599999999997]
        self.go_radians(over_radians, 2)
    
    def goStackingUpperPose(self):
        over_radians = [-0.835744, 0.14522199999999996, -1.27141, -0.3633420000000003, 0.0, -0.07746999999999993]
        self.go_radians(over_radians, 2)

    def goColorOverPose(self):
        over_radians = [0.82, -0.2, -0.35, -0.69, 0, -0.77]
        self.go_radians(over_radians, 2)

    def goGarbageOverPose(self):
        over_radians = [-0.82, 0, -0.29, -0.69, 0, -0.77]
        self.go_radians(over_radians)

    # 提起夹爪
    def lift_gripper(self, task, type, move_num, name):
        if task == "sorting":
            self.mc.send_coord(Coord.Z.value, 180, 40)
            sleep(1.5)
        elif task == "stacking":
            # self.mc.send_coord(Coord.Z.value, 200, 40)
            # sleep(1.5)
            self.ctrl_gripper_height(150+int(move_num)*30, 1.5)

    # 控制夹爪高度
    def ctrl_gripper_height(self, value_Z, delay=0):
        self.mc.send_coord(Coord.Z.value, value_Z, 40)
        if delay > 0:
            sleep(delay)

    # 夹爪上升
    def rise_gripper(self, delay=0):
        self.mc.send_coord(Coord.Z.value, 180, 40)
        if delay > 0:
            sleep(delay)
    
    # 夹爪下降
    def drop_gripper(self, delay=0):
        self.mc.send_coord(Coord.Z.value, 110, 40)
        if delay > 0:
            sleep(delay)
    
    # 点头
    def ctrl_nod(self):
        for i in range(2):
            self.mc.send_angle(Angle.J4.value, -30, 50)
            sleep(1)
            self.mc.send_angle(Angle.J4.value, 10, 50)
            sleep(1)
        self.mc.send_angle(Angle.J4.value, 0, 50)
        sleep(1)
        
    # task: sorting(分拣) stacking(堆叠)
    # type：garbage, color, apriltag
    def goTargetPose(self, task, type, move_num, name):
        if task == "sorting":
            if type == "garbage":
                self.goGarbageSortingPose(name)
            elif type == "color":
                self.goColorSortingPose(name)
            elif type == "apriltag": 
                self.goApriltagSortingPose(name)
        elif task == "stacking":
            if type == "garbage":
                self.goGarbageStackingPose(name, self.garbage_num)
            else:
                self.goStackingPose(move_num)
                sleep(1)

    def goStackingPose(self, move_num):
        if move_num == '1':
            self.goStackingNum1Pose()
        elif move_num == '2':
            self.goStackingNum2Pose()
        elif move_num == '3':
            self.goStackingNum3Pose()
        elif move_num == '4':
            self.goStackingNum4Pose()
    
    def goColorSortingPose(self, name):
        if name == "red":
            self.goRedPose()
        if name == "blue":
            self.goBluePose()
        if name == "green":
            self.goGreenPose()
        if name == "yellow":
            self.goYellowPose()

    def goApriltagSortingPose(self, name):
        if name == '1':
            self.goApriltag1fixedPose()
        elif name == '2':
            self.goApriltag2fixedPose()
        elif name == '3':
            self.goApriltag3fixedPose()
        elif name == '4':
            self.goApriltag4fixedPose()
        else:
            self.goApriltag1fixedPose()
        sleep(1)

    def goGarbageStackingPose(self, name, num):
        if name == "Zip_top_can" or name == "Newspaper" or name == "Old_school_bag" or name == "Book":
            num[0] = int(num[0])+1
            self.goRecyclablePose(num[0]) # 可回收垃圾
        elif name == "Syringe" or name == "Used_batteries" or name == "Expired_cosmetics" or name == "Expired_tablets":
            num[1] = int(num[1])+1
            self.goHazardousWastePose(num[1]) # 有害垃圾
        elif name == "Fish_bone" or name == "Watermelon_rind" or name == "Apple_core" or name == "Egg_shell":
            num[2] = int(num[2])+1
            self.goFoodWastePose(num[2]) # 厨余垃圾
        elif name == "Cigarette_butts" or name == "Toilet_paper" or name == "Peach_pit" or name == "Disposable_chopsticks":
            num[3] = int(num[3])+1
            self.goResidualWastePose(num[3]) # 其他垃圾
    
    def goGarbageSortingPose(self, name):
        if name == "Zip_top_can" or name == "Newspaper" or name == "Old_school_bag" or name == "Book":
            self.goRecyclablePose() # 可回收垃圾
        elif name == "Syringe" or name == "Used_batteries" or name == "Expired_cosmetics" or name == "Expired_tablets":
            self.goHazardousWastePose() # 有害垃圾
        elif name == "Fish_bone" or name == "Watermelon_rind" or name == "Apple_core" or name == "Egg_shell":
            self.goFoodWastePose() # 厨余垃圾
        elif name == "Cigarette_butts" or name == "Toilet_paper" or name == "Peach_pit" or name == "Disposable_chopsticks":
            self.goResidualWastePose() # 其他垃圾

    def limit_garbage_layer(self, layer):
        if int(layer) < 1 or int(layer) > 2:
            return 2
        else:
            return layer

    # 其他垃圾位置
    def goResidualWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        coords = [130, -225, 120 + int(layer-1)*40, -180, -10, -45]
        self.go_coords(coords, 3)
    
    # 厨余垃圾位置
    def goFoodWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        coords = [65, -225, 110 + int(layer-1)*40, -180, -10, -45]
        self.go_coords(coords, 3)
    
    # 有害垃圾位置
    def goHazardousWastePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        coords = [5, -225, 110 + int(layer-1)*40, -180, -10, -45]
        self.go_coords(coords, 3)
    
    # 可回收垃圾位置
    def goRecyclablePose(self, layer=1):
        layer = self.limit_garbage_layer(layer)
        coords = [-70, -225, 110 + int(layer-1)*40, -180, -10, -45]
        self.go_coords(coords, 3)

    
    # stacking 区域位置
    def goStackingNum1Pose(self):
        coords = [135, -150, 115, -180, -10, -45]
        self.go_coords(coords, 3)
    
    def goStackingNum2Pose(self):
        coords = [132, -150, 145, -180, -10, -45]
        self.go_coords(coords, 3)
    
    def goStackingNum3Pose(self):
        coords = [132, -150, 175, -180, -10, -45]
        self.go_coords(coords, 3)

    def goStackingNum4Pose(self):
        coords = [131, -150, 205, -180, -10, -45]
        self.go_coords(coords, 3)
        
    # 方框中心十字架位置
    def goBoxCenterlayer1Pose(self, grasp=0):
        if grasp == 0:
            coords = [210, -5, 120, -175, 0, -45]
        else:
            coords = [200, -5, 120, -175, 0, -45]
        self.go_coords(coords, 3)

    # 颜色放置区域位置
    def goYellowPose(self):
        coords = [140, 215, 115, -175, 0, -45]
        self.go_coords(coords, 3)
    
    def goRedPose(self):
        coords = [75, 215, 115, -175, 0, -45]
        self.go_coords(coords, 3)
    
    def goGreenPose(self):
        coords = [10, 215, 115, -175, 0, -45]
        self.go_coords(coords, 3)
    
    def goBluePose(self):
        coords = [-70, 215, 115, -175, 0, -45]
        self.go_coords(coords, 3)


    # 颜色识别抓取位
    def goYellowfixedPose(self):
        coords = [135, 230, 160, -170, 3, -45]
        self.go_coords(coords, 3)

    def goRedfixedPose(self):
        coords = [70, 230, 160, -170, 3, -45]
        self.go_coords(coords, 3)

    def goGreenfixedPose(self):
        coords = [0, 230, 160, -170, 3, -45]
        self.go_coords(coords, 3)

    def goBluefixedPose(self):
        coords = [-70, 230, 160, -170, 3, -45]
        self.go_coords(coords, 3)

    
    # Apriltag标签位置
    def goApriltag4fixedPose(self, layer=1):
        coords = [142, 140, 110+50*(layer-1), -170, 3, -45]
        self.go_coords(coords, 2)

    def goApriltag3fixedPose(self, layer=1):
        coords = [77, 140, 110+50*(layer-1), -170, 3, -45]
        self.go_coords(coords, 2)

    def goApriltag2fixedPose(self, layer=1):
        coords = [2, 140, 110+50*(layer-1), -170, 3, -45]
        self.go_coords(coords, 2)

    def goApriltag1fixedPose(self, layer=1):
        coords = [-68, 140, 110+50*(layer-1), -170, 3, -45]
        self.go_coords(coords, 2)


