#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2018/9/27 上午9:22
# @Author  : Lynn
# @Site    : 
# @File    : RobotControler.py
# @Software: PyCharm

import socket
import time
import sixAxesInverse as sai
import sys
############################################################################################

class RobotController():
    def __init__(self, pid = "192.168.39.220", port = 9876):
        """
        构造函数，设置对象的pid和port属性。
        :param pid:     机械臂IP地址
        :param port:    机械臂端口
        """
        self.ip_port = (pid, port)
        self.sk = socket.socket()
        print('connect...')
        self.sk.connect((self.ip_port))



    def send_OK(self):
        """
        向机械臂发送OK的确认信息。
        :return:    None
        """
        print('SEND:OK')
        self.sk.sendall(bytes("OK\0",encoding="utf8"))
        data_recv = self.sk.recv(2)
        data = str(data_recv, encoding='utf8')
        if data == 'OK':
            print('RECV:' + data)
            print('connect successfully.')
        else:
            print('fail to connect to the robot!')
            sys.exit()


    def move_car_init(self):
        """
        将机器人移动到自定义的初始位置[530, 0, 510, 180, 0, 0]
        :return:        None
        """
        self.sk.sendall(bytes("1,530,0,510,180,0,0,\0", encoding='utf8'))


    def move_car(self, carPos):
        """
        通过笛卡尔坐标移动机械臂。
        :param carPos:  笛卡尔坐标
        :return:        None
        """
        strData = '1,{0},{1},{2},{3},{4},{5},\0'.format(carPos[0], carPos[1], carPos[2], carPos[3], carPos[4], carPos[5])
        self.sk.sendall(bytes(strData, encoding='utf8'))



    def move_car_by_offset(self, offset_x = 0, offset_y = 0, offset_z = 0, offset_A = 0, offset_B = 0, offset_C = 0):
        """
        传入笛卡尔坐标的偏移量并移动。
        :param self: 
        :param offset_x:    x轴偏移量
        :param offset_y:    y轴偏移量
        :param offset_z:    z轴偏移量
        :param offset_A:    A的偏移量
        :param offset_B:    B的偏移量
        :param offset_C:    C的偏移量
        :return:            None
        """
        strData = '3,{0},{1},{2},{3},{4},{5},\0'.format(offset_x, offset_y, offset_z, offset_A, offset_B, offset_C)
        # strDirection = '3,' + str(offset_x) + ',' + str(offset_y) + ',' + str(offset_z) + ',' + \
        #     str(offset_A) + ',' + str(offset_B) + ',' + str(offset_C) + '\0'
        self.sk.sendall(bytes(strData, encoding='utf8'))



    def move_axis_by_offset(self, offset_a1 = 0, offset_a2 = 0, offset_a3 = 0, offset_a4 = 0, offset_a5 = 0, offset_a6 = 0):
        """
        传入关节坐标偏移量控制机械臂移动。
        :param offset_a1:   关节a1
        :param offset_a2:   关节a2
        :param offset_a3:   关节a3
        :param offset_a4:   关节a4
        :param offset_a5:   关节a5
        :param offset_a6:   关节a6
        :return:            None
        """
        strdata = '2,{0},{1},{2},{3},{4},{5},\0'.format(offset_a1,offset_a2,offset_a3,offset_a4,offset_a5,offset_a6)
        self.sk.sendall(bytes(strdata, encoding = 'utf8'))



    def move_axis(self, carPos, prea):
        """
        将笛卡尔坐标系转换成关节坐标系后，获取发送字符串。
        :param carPos:  笛卡尔坐标
        :param prea:
        :return:        None
        """
        self.sk.sendall(bytes(self.car_to_axis_str(carPos, prea), encoding='utf8'))



    def get_current_car_pos(self):
        '''
        向机械臂发送7，机械臂发送六个浮点数，即笛卡尔坐标值。
        :return:    笛卡尔坐标值
        '''
        print('SEND:7')
        self.sk.sendall(bytes(str(7) + ',\0', encoding = 'utf8'))
        carpos = []
        for i in range(6):
            data = self.sk.recv(7)
            data.decode('utf-8')
            # print(data)
            data = str(data, encoding='utf8')
            # print(data)
            # carpos.append(round(float(data.split('\\')[0]), 2))
            carpos.append(float(data))
            # print(carpos)

        return carpos


    def get_current_axis_pos(self):
        '''
        向机械臂发送6，机械臂发送六个浮点数，即关节坐标值。
        :return:    关节坐标值
        '''
        print('SEND:6')
        self.sk.sendall(bytes(str(6) + ',\0', encoding = 'utf8'))
        carpos = []
        for i in range(6):
            data = self.sk.recv(7)
            data.decode('utf-8')
            # print(data)
            data = str(data, encoding='utf8')
            # print(data)
            # carpos.append(round(float(data.split('\\')[0]), 2))
            carpos.append(float(data))
            # carpos.append(round(float(data), 2))
            # print(carpos)

        return carpos



    def car_to_axis_str(self, carPos, prea):
        """调用sixAxesInverse将世界坐标系转换成关节坐标系，并返回发送字符串。
            :param carPos:      世界坐标系
            :param  prea:
            :return :           返回值为关节坐标字符串
        """
        inv = sai.Inverse(carPos[0], carPos[1], carPos[2], carPos[3], carPos[4], carPos[5], prea)
        solution, iloc = inv.inverse()
        t = solution[iloc]
        strPos = "0," + str(round(t[0], 2)) + "," + str(round(t[1], 2)) + "," + \
                       str(round(t[2], 2)) + "," + str(round(t[3], 2)) + "," + str(round(t[4], 5)) + "," + str(
            round(t[5], 2)) + ",\0"
        return strPos



    def control_paw(self, flag = 4):
        """
        控制机器人手爪的闭合与张开。
        :param flag:    4-张开手爪， 5-闭合手爪
        :return:        None
        """
        # print('SEND:4')
        strSend = str(flag) + ',\0'
        self.sk.sendall(bytes(strSend, encoding='utf8'))



    def close(self):
        """
        断开和机械臂的连接。
        :return:        None
        """
        self.sk.close()