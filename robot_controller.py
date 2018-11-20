#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2018/9/27 9:22
# @Author  : Lynn
# @Site    : 
# @File    : robot_controller.py
# @Software: PyCharm

import socket
import time
import sys
############################################################################################

class RobotController():
    def __init__(self, pid="192.168.39.220", port=9876):
        """
        构造函数，设置对象的pid和port属性。

        参数：
        ----------
        :param pid:     机械臂IP地址
        :param port:    机械臂端口
        """
        self.ip_port = (pid, port)
        self.sk = socket.socket()
        print('connect...')
        self.sk.connect((self.ip_port))
        self.send_OK()



    def send_OK(self):
        """
        向机械臂发送OK的确认信息。

        返回值：
        ----------
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



    def move_car(self, carPos):
        """
        通过笛卡尔坐标移动机械臂。

        参数：
        ----------
        :param carPos:  list
                笛卡尔坐标([x,y,z,A,B,C], 精度：两位小数)
        返回值：
        ----------
        :return:    None
        """
        strData = '1,{0},{1},{2},{3},{4},{5},\0'.format(carPos[0], carPos[1], carPos[2], carPos[3], carPos[4], carPos[5])
        self.sk.sendall(bytes(strData, encoding='utf8'))
        self.delay()


    def move_car_by_offset(self, offset_x=0, offset_y=0, offset_z=0, offset_A=0, offset_B=0, offset_C=0):
        """
        传入笛卡尔坐标的偏移量并移动。

        参数：
        ----------
        :param offset_x: float
                x轴偏移量(精度：两位小数)
        :param offset_y: float
                y轴偏移量(精度：两位小数)
        :param offset_z: float
                z轴偏移量(精度：两位小数)
        :param offset_A: float
                A的偏移量(精度：两位小数)
        :param offset_B: float
                B的偏移量(精度：两位小数)
        :param offset_C: float
                C的偏移量(精度：两位小数)

        返回值：
        ----------
        :return:    None
        """
        strData = '3,{0},{1},{2},{3},{4},{5},\0'.format(offset_x, offset_y, offset_z, offset_A, offset_B, offset_C)
        self.sk.sendall(bytes(strData, encoding='utf8'))
        self.delay()



    def get_current_car_pos(self):
        '''
        向机械臂发送7，机械臂发送六个浮点数，即笛卡尔坐标值。

        返回值：
        ----------
        :return:  list
                笛卡尔坐标值([x,y,z,A,B,C])
        '''
        print('SEND:7')
        self.sk.sendall(bytes(str(7) + ',\0', encoding='utf8'))
        carpos = []
        for i in range(6):
            data = self.sk.recv(7)
            data.decode('utf8')
            # print(data)
            data = str(data, encoding='utf8')
            # print(data)
            # carpos.append(round(float(data.split('\\')[0]), 2))
            carpos.append(float(data))
            # print(carpos)

        return carpos



    def move_axis(self, axisPos, prea):
        """
        将笛卡尔坐标系转换成关节坐标系后，获取发送字符串。

        参数：
        ----------
        :param carPos: list
                笛卡尔坐标([x,y,z,A,B,C], 精度：两位小数)
        :param prea:

        返回值：
        ----------
        :return:    None
        """
        strData = '0,{0},{1},{2},{3},{4},{5},\0'.format(axisPos[0], axisPos[1], axisPos[2], axisPos[3], axisPos[4], axisPos[5])
        self.sk.sendall(bytes(strData, encoding='utf8'))
        self.delay()



    def move_axis_by_offset(self, offset_a1=0, offset_a2=0, offset_a3=0, offset_a4=0, offset_a5=0, offset_a6=0):
        """
        传入关节坐标偏移量控制机械臂移动。

        参数：
        ----------
        :param offset_a1: float
                关节a1(精度：两位小数)
        :param offset_a2: float
                关节a2(精度：两位小数)
        :param offset_a3: float
                关节a3(精度：两位小数)
        :param offset_a4: float
                关节a4(精度：两位小数)
        :param offset_a5: float
                关节a5(精度：两位小数)
        :param offset_a6: float
                关节a6(精度：两位小数)

        返回值：
        ----------
        :return:    None
        """
        strdata = '2,{0},{1},{2},{3},{4},{5},\0'.format(offset_a1,offset_a2,offset_a3,offset_a4,offset_a5,offset_a6)
        self.sk.sendall(bytes(strdata, encoding='utf8'))
        self.delay()



    def get_current_axis_pos(self):
        '''
        向机械臂发送6，机械臂回复发送六个浮点数，即关节坐标值。

        返回值：
        ----------
        :return:  list
                关节坐标值([a1,a2,a3,a4,a5,a6])
        '''
        print('SEND:6')
        self.sk.sendall(bytes(str(6) + ',\0', encoding='utf8'))
        axispos = []
        for i in range(6):
            data = self.sk.recv(7)
            data.decode('utf-8')
            # print(data)
            data = str(data, encoding='utf8')
            # print(data)
            # carpos.append(round(float(data.split('\\')[0]), 2))
            axispos.append(float(data))
            # carpos.append(round(float(data), 2))
            # print(carpos)

        return axispos



    def control_paw(self, flag=4, pulse_cnt=300):
        """
        控制机器人手爪的闭合与张开。

        参数：
        ----------
        :param flag:    4-张开手爪， 5-闭合手爪

        返回值：
        ----------
        :return:        None
        """
        strSend = str(flag) + ',' + str(pulse_cnt) + ',\0'
        self.sk.sendall(bytes(strSend, encoding='utf8'))



    def set_speed(self, rate=10):
        '''
        控制机器人运动的速度，速度为原速度的 speed_rate/10。

        参数：
        ----------
        :param speed_rate: int
                速度分为10个等级，即设置速度为原速度的speed_rate/10。

        返回值：
        ----------
        :return:        None
        '''
        strSend = str(8) + ',' + str(rate) + ',\0'
        self.sk.sendall(bytes(strSend, encoding='utf8'))
        self.delay()



    def delay(self):
        """
        机械臂移动以及设置速度时使用的延迟函数。

        """
        recvData = self.sk.recv(5)
        print(recvData)



    def close(self):
        """
        断开和机械臂的连接。

        返回值：
        ----------
        :return:        None
        """
        self.sk.close()
