import os
from pymycobot import MyCobot
import time

# 需要将串口号更改为电脑实际的串口号，将夹爪张开到最大，再运行此脚本
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time

mc = MyCobot('/dev/ttyUSB0', 1000000)
mc.send_angles([0, 0, 0, 0, 0, -45], 50)

print("零位矫正前当前所在位置: ",mc.get_gripper_value())
time.sleep(0.1)
mc.set_gripper_calibration()
time.sleep(0.1)
print("零位矫正后当前所在位置（矫正成功夹爪会锁住，并且位置接近100）: ",mc.get_gripper_value())
time.sleep(0.1)
print("开始夹爪参数更新...")
# datas = [10, 0, 1, 150]
datas = [25, 25, 0, 140]
address = [21, 22, 23, 16]
current_datas = []
new_datas = []
for addr in address:
    current_datas.append(mc.get_servo_data(7, addr))
    time.sleep(0.1)
print("当前夹爪参数为: ",current_datas)
for addr in range(len(address)):
    mc.set_servo_data(7, address[addr], datas[addr])
    time.sleep(0.1)
for addr in address:
    new_datas.append(mc.get_servo_data(7, addr))
    time.sleep(0.1)
print("更新后夹爪参数为: ",new_datas)




