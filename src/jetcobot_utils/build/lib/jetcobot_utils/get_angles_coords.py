import os
from pymycobot import MyCobot
from time import sleep
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from geometry_msgs.msg import Pose
import math

mc = MyCobot('/dev/ttyUSB0', 1000000)

# 输入：欧拉角的角度
# roll = -140.0
# pitch = 0.0
# yaw = 0.0
# 输出：四元素
def euler_to_quaternion(roll, pitch, yaw):
    # 角度转弧度
    DE2RA = math.pi / 180
    # RPY转四元素
    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    orientation_x = q[0]
    orientation_y = q[1]
    orientation_z = q[2]
    orientation_w = q[3]
    return orientation_x, orientation_y, orientation_z, orientation_w


if __name__ == '__main__':
    angles = mc.get_angles()
    coords = mc.get_coords()
    print("angles = ", angles)
    print("coords = ", coords)
    x = coords[0]
    y = coords[1]
    z = coords[2]
    rx = coords[3]
    ry = coords[4]
    rz = coords[5]
    orientation_x, orientation_y, orientation_z, orientation_w = euler_to_quaternion(rx, ry, rz)
    print("x = {}, y = {}, z = {}, orientation_x = {}, orientation_y = {}, orientation_z = {}, orientation_w = {}".
          format(x, y, z, orientation_x, orientation_y, orientation_z, orientation_w))

    (r, p, y) = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w])
    print("_"*100)
    print("roll = {}, pitch = {}, yaw = {}".format(math.degrees(r), math.degrees(p), math.degrees(y)))

