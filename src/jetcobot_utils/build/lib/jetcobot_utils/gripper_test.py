import os
from pymycobot import MyCobot
from time import sleep

mc = MyCobot('/dev/ttyUSB0', 1000000)

def handclap():
    # mc.set_gripper_calibration()
    sleep(3)
    for i in range(5):
        print("----",i)
        # 1 表示夹爪合拢状态，0 表示夹爪打开状态
        mc.set_gripper_value(100,50)
        sleep(3)
        mc.set_gripper_value(40,50)
        sleep(3)

if __name__ == '__main__':
    # mc.send_angles([-90, 0, 0, 0, 0, -45], 50)
    # sleep(1.5)
    handclap()
    # 读取夹爪的版本号
    # v = mc.get_servo_data(7,3)
    # print("v = ",v)
    
    # mc.set_gripper_state(0, 50)
    # sleep(3)
    # gripper_value = mc.get_gripper_value()
    # print("gripper_value = {}".format(gripper_value))
    # print("get_system_version() = ",mc.get_system_version())

    # angles = mc.get_angles()
    # print("angles = ",angles)




