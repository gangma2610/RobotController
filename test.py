#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2018/10/8 15:51
# @Author  : Lynn
# @Site    : 
# @File    : test.py
# @Software: PyCharm
from robot_controller import *
import time
import numpy as np
import os

def delete_image(delDir = '.'):
    # delList = []
    # delLis = os.listdir(delDir)
    for root, dirs, files in os.walk(delDir):
        for name in files:
            if(name.endswith('.jpg')):
                os.remove(os.path.join(root, name))

def main():
    # delete_image()
    # tlen = 128
    init_pos = [400, 100, 510, 180, 0, 0]
    test_pos = [500, -200, 360, 180, 0, 0]
    # test_pos = [510, 260, 175, 180, 0, 0]
    # init_pos_str = "1,530,0,390,180,0,0"
    # pos_tmp = [507.83, 92.01, round(359.32 - tlen, 2), 180, 0, 0]
    # pos_tmp = [548.97 + 41, 1, 169, 180, 0, 0]
    prea = np.array([0, 56, -17, 0, 24, 0])

    robot_instance = RobotController()
    # robot_instance.move_car_by_offset(offset_C = 180)
    # robot_instance.move_axis_by_offset(offset_a2 = 30)

    # for i in range(3):
    #     robot_instance.control_paw(5, 100)
    #     time.sleep(1)

    # robot_instance.control_paw(5)

    # robot_instance.control_paw(4)
    # robot_instance.set_speed(1)
    # time.sleep(1)
    # # while True:
    # robot_instance.move_car(test_pos)
    # time.sleep(2)
    robot_instance.move_car(init_pos)
        # time.sleep(2)
        # robot_instance.set_speed(4)
        # time.sleep(1)
    # print(robot_instance.get_current_car_pos())
    # print(robot_instance.get_current_axis_pos())
    # time.sleep(3)
    # robot_instance.move_car_by_offset(offset_z=-10)
    # robot_instance.move_car_by_offset(offset_x=-10)
    # print(robot_instance.get_current_axis_pos())
    # print(robot_instance.get_current_car_pos())
    robot_instance.close()

if __name__ == '__main__':
    main()