#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
import threading
import time
from sensor_msgs.msg import Joy
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle


class Jetcobot_Joystick(Node):
    def __init__(self):
        super().__init__('joy_ctrl')
        self.reset_value()
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        self.ANGLE_MIN = [-168, -135, -150, -145, -165, -180, 0]
        self.ANGLE_MAX = [168, 90, 150, 145, 165, 180, 100]
        self.GRIPPER_ID_J7 = 7
        self.sub_Joy = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)
        self.init_pose()

        self.Joy_Button_Index = {
            # BUTTON
            "KEY_A": 0,
            "KEY_B": 1,
            "KEY_X": 3,
            "KEY_Y": 4,
            "KEY_L1": 6,
            "KEY_R1": 7,
            "KEY_L2": 8,
            "KEY_R2": 9,
            "KEY_SELECT": 10,
            "KEY_START": 11,
            "KEY_RK1": 12,
            "KEY_RK2": 13
        }

        self.Joy_Axis_Index = {
            # AXIS
            "RK1_LEFT_RIGHT": 0,
            "RK1_UP_DOWN": 1,
            "RK2_LEFT_RIGHT": 2,
            "RK2_UP_DOWN": 3,
            "AXIS_R2": 4,
            "AXIS_L2": 5,
            "WSAD_LEFT_RIGHT": 6,
            "WSAD_UP_DOWN": 7
        }

    # 初始化数值
    # Initialize the value
    def reset_value(self):
        self.joins_active = [False for i in range(7)]
        self.arm_joints = [0, 0, 0, 0, 0, -45, 100]
        self.ctrl_step = 2
        self.speed = 50
        self.press_state = 0

    # 机器人初始状态的位姿
    # The initial position of the robot
    def init_pose(self):
        self.reset_value()
        self.mc.send_angles(self.arm_joints, self.speed)
        time.sleep(0.5) 
        self.mc.set_gripper_value(100, self.speed)

    # 控制夹爪，state=0为松开，state=1为夹紧
    # Control the gripper, state=0 means release, state=1 means clamp
    def ctrl_gripper(self, state):
        if state:
            self.joins_active[6] = False
            self.arm_joints[6] = 20
            self.mc.set_gripper_value(20, self.speed)
        else:
            self.joins_active[6] = False
            self.arm_joints[6] = 100
            self.mc.set_gripper_value(100, self.speed)

    # 控制机器人运动
    # Control the robot movement
    def ctrl_machine(self, id, step):
        while True:
            if self.joins_active[id - 1]:
                self.arm_joints[id - 1] += step
                if self.arm_joints[id - 1] > self.ANGLE_MAX[id - 1]:
                    self.arm_joints[id - 1] = self.ANGLE_MAX[id - 1]
                elif self.arm_joints[id - 1] < self.ANGLE_MIN[id - 1]:
                    self.arm_joints[id - 1] = self.ANGLE_MIN[id - 1]
                self.get_logger().info("joints:%d = %d" % (id, self.arm_joints[id - 1]))
                if id == self.GRIPPER_ID_J7:
                    self.mc.set_gripper_value(self.arm_joints[id - 1], self.speed)
                else:
                    self.mc.send_angle(id, self.arm_joints[id - 1], self.speed)
                time.sleep(0.05)
            else:
                break

    # 根据舵机ID和一步的差值来控制舵机
    # Control the servo according to the servo ID and the difference of one step
    def update_joints(self, id, step):
        if step == 0:
            self.joins_active[id - 1] = False
            return
        if self.joins_active[id - 1]:
            return
        self.joins_active[id - 1] = True
        arm_thread = threading.Thread(target=self.ctrl_machine, args=(id, step))
        arm_thread.setDaemon(True)
        arm_thread.start()

    # 根据手柄的按键状态，获取当前关节对应一步的差值
    #According to the button status of the handle, get the difference of the current joint corresponding to one step
    def key_to_step(self, joy_data, key_increase, key_decrease):
        value_inc = int(joy_data.buttons[self.Joy_Button_Index.get(key_increase)])
        value_dec = int(joy_data.buttons[self.Joy_Button_Index.get(key_decrease)])
        result = 0
        if value_inc != 0:
            result = self.ctrl_step
        elif value_dec != 0:
            result = -self.ctrl_step
        return result

    # 根据手柄的轴状态，获取当前关节对应一步的差值
    #According to the axis state of the handle, get the difference of the current joint corresponding to one step
    def axis_to_step(self, joy_data, axis):
        result = self.ctrl_step * int(joy_data.axes[self.Joy_Axis_Index.get(axis)])
        return result

    # 根据手柄的按键状态，更新一步的差距
    # Update the gap of one step according to the button status of the handle
    def update_step(self, joy_data, key_increase, key_decrease):
        step = self.ctrl_step
        if joy_data.buttons[self.Joy_Button_Index.get(key_increase)]:
            if self.press_state == 0:
                self.press_state = 1
                step = self.ctrl_step + 1
                if step > 3:
                    step = 3
            self.get_logger().info("step:%d" % step)
        else:
            if self.press_state == 1:
                self.press_state = 0
        if joy_data.buttons[self.Joy_Button_Index.get(key_decrease)]:
            if self.press_state == 0:
                self.press_state = -1
                step = self.ctrl_step - 1
                if step < 1:
                    step = 1
            self.get_logger().info("step:%d" % step)
        else:
            if self.press_state == -1:
                self.press_state = 0
        self.ctrl_step = step
        return self.ctrl_step

    # 手柄数据更新的回调函数
    # Controller data update callback function
    def joystick_callback(self, joy_data):
        self.update_step(joy_data, "KEY_L1", "KEY_L2")
        self.update_joints(Angle.J1.value, self.axis_to_step(joy_data, "RK1_LEFT_RIGHT"))
        self.update_joints(Angle.J2.value, self.axis_to_step(joy_data, "RK1_UP_DOWN"))
        self.update_joints(Angle.J3.value, self.axis_to_step(joy_data, "WSAD_UP_DOWN"))
        self.update_joints(Angle.J4.value, self.axis_to_step(joy_data, "RK2_UP_DOWN"))
        self.update_joints(Angle.J5.value, self.axis_to_step(joy_data, "RK2_LEFT_RIGHT"))
        self.update_joints(Angle.J6.value, self.key_to_step(joy_data, "KEY_B", "KEY_X"))
        self.update_joints(self.GRIPPER_ID_J7, self.key_to_step(joy_data, "KEY_A", "KEY_Y"))
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_R1")]:
            self.ctrl_gripper(1)
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_R2")]:
            self.ctrl_gripper(0)
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_SELECT")]:
            self.init_pose()


def main(args=None):
    rclpy.init(args=args)
    node = Jetcobot_Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()