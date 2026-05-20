from time import sleep
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle

mc = MyCobot('/dev/ttyUSB0', 1000000)

if __name__ == '__main__':
    try:
        # coords =  [303.7, 7.9, 131.5, -175.05, -1.73, -43.63]
        # coords2 =  [253.7, 7.9, 131.5, -175.05, -1.73, -43.63]
        # flag1 = mc.is_in_position(coords,1)
        # flag2 = mc.is_in_position(coords2,1)
        # print("flag1 = ",flag1)
        # print("flag2 = ",flag2)
        # mc.send_coords(coords2, 40, 0)

        coords =  [141.1, -8.7, 107.6, 176.2, -0.34, -42.47]
        mc.send_coords(coords, 40, 0)
    except Exception as e:
        print("e = ",e)

    sleep(3)
    coords_result = mc.get_coords()
    print("coords_result = ", coords_result)
