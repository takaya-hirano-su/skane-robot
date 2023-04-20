#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

# Editted by Dai Owaki, 2019.3.12.
# ten motors controlled by CPG
import sys
from pathlib import Path
DYNAMIXEL_PATH=str("/home/pi/hirano-dev/DynamixelSDK/python/src/")
sys.path.append(DYNAMIXEL_PATH)

import os
from tokenize import group

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

import time
import math
import threading
import signal
import sys
import random
import struct
import numpy as np

import socket
import argparse
# parser = argparse.ArgumentParser(description='このプログラムの説明（なくてもよい）')
# parser.add_argument('--ipaddr',type=str,default = '192.168.1.202')
# #parser.add_argument('--ipaddr',type=str,default = '192.168.0.111')
# parser.add_argument('--port',type=int,default = 8080)
# args = parser.parse_args()
# ipaddr = args.ipaddr
# sockport = args.port

# Control table address
ADDR_XH430_TORQUE_ENABLE   = 64

ADDR_XH430_POSITION_D_GAIN = 80
ADDR_XH430_POSITION_I_GAIN = 82
ADDR_XH430_POSITION_P_GAIN = 84

ADDR_XH430_GOAL_CURRENT       = 102
ADDR_XH430_GOAL_VELOCITY      = 104
ADDR_XH430_GOAL_POSITION      = 116

ADDR_XH430_PRESENT_CURRENT    = 126
ADDR_XH430_PRESENT_VELOCITY   = 128
ADDR_XH430_PRESENT_POSITION   = 132

ADDR_XH430_MAX_POSITION_LIMIT = 48
ADDR_XH430_MIN_POSITION_LIMIT = 52

# Data Byte Length
LEN_XH430_GOAL_POSITION      = 4
LEN_XH430_GOAL_VELOCITY      = 4
LEN_XH430_GOAL_CURRENT       = 2

LEN_XH430_PRESENT_POSITION   = 4
LEN_XH430_PRESENT_VELOCITY   = 4
LEN_XH430_PRESENT_CURRENT    = 2

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1  ID : 1
DXL2_ID                     = 2                 # Dynamixel#2  ID : 2
DXL3_ID                     = 3                 # Dynamixel#3  ID : 3
DXL4_ID                     = 4                 # Dynamixel#4  ID : 4
DXL5_ID                     = 5                 # Dynamixel#5  ID : 5
DXL6_ID                     = 6                 # Dynamixel#6  ID : 6
DXL7_ID                     = 7                 # Dynamixel#7  ID : 7
DXL8_ID                     = 8                 # Dynamixel#8  ID : 8
DXL9_ID                     = 9                 # Dynamixel#9  ID : 9
DXL10_ID                    = 10                # Dynamixel#10 ID : 10
DXL11_ID                    = 11                # Dynamixel#10 ID : 11
DXL12_ID                    = 12                # Dynamixel#10 ID : 12
DXL_IDs = [1,2,3,4,5,6,7,8,9,10,11,12]
DXL_IDs_t = [1,2,3,4,5,6,7,8,9,10,11,12]
# DXL_IDs_t = [1,2,3,4]


DXLALL_ID                   = 254               # Send Packet for All motors

# BAUDRATE                    = 2000000
BAUDRATE                    = 3000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 1600                       # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2500                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

ESC_ASCII_VALUE             = 0x1b
s_ASCII_VALUE               = 0x73
q_ASCII_VALUE               = 0x71

middle_position       = 2047#0~4095

CDC_P_GAIN = 300

pi = 3.1415926

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows

portHandler = PortHandler(DEVICENAME)#DEVICENAME = '/dev/ttyUSB0'

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler

packetHandler = PacketHandler(PROTOCOL_VERSION)#2

# Initialize GroupSyncWrite instance

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_XH430_GOAL_POSITION, LEN_XH430_GOAL_POSITION)
ADDR_XH430_GOAL_POSITION = 116 #レジスタの位置
LEN_XH430_GOAL_POSITION = 4 #レジスタの大きさ

# Initialize GroupSyncRead instace for Present Position

groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_XH430_PRESENT_POSITION, LEN_XH430_PRESENT_POSITION)
# groupSyncRead_vel = GroupSyncRead(portHandler, packetHandler, ADDR_XH430_PRESENT_VELOCITY, LEN_XH430_PRESENT_VELOCITY)
groupSyncRead_cur = GroupSyncRead(portHandler, packetHandler, ADDR_XH430_PRESENT_CURRENT, LEN_XH430_PRESENT_CURRENT)
# groupSyncRead_err = GroupSyncRead(portHandler, packetHandler, 70, 1)

# Open port

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# #opnesocket with arged ipaddr
# sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((ipaddr,sockport))

time.sleep(2.0)


while True:
    print("Press [s] to start!")
    if getch() == chr(s_ASCII_VALUE):
        break

print("set p gain of position control\n")
print(packetHandler.write2ByteTxRx(portHandler, DXLALL_ID, ADDR_XH430_POSITION_P_GAIN, CDC_P_GAIN))#ADDR_XH430_POSITION_P_GAIN = 84,CDC_P_GAIN = 300

# print("set degree range of position control\n")
print("==================================================")
print(packetHandler.write4ByteTxRx(portHandler, DXLALL_ID, ADDR_XH430_MAX_POSITION_LIMIT,DXL_MAXIMUM_POSITION_VALUE))#+40 degree
print(packetHandler.write4ByteTxRx(portHandler, DXLALL_ID, ADDR_XH430_MIN_POSITION_LIMIT,DXL_MINIMUM_POSITION_VALUE))#-40 degree


##これで1番が動かせる??
for _ in range(10):
    packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_goal_position1 = DXL_MAXIMUM_POSITION_VALUE
    param_goal_position1 = [
        DXL_LOBYTE(DXL_LOWORD(dxl_goal_position1)),
        DXL_HIBYTE(DXL_LOWORD(dxl_goal_position1)),
        DXL_LOBYTE(DXL_HIWORD(dxl_goal_position1)),
        DXL_HIBYTE(DXL_HIWORD(dxl_goal_position1))
        ]
    print(param_goal_position1)
    groupSyncWrite.addParam(DXL1_ID,param_goal_position1)
    dxl_comm_result = groupSyncWrite.txPacket()

    if not dxl_comm_result==COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

    dxl_present_position1 = groupSyncRead.getData(DXL1_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION)
    print(dxl_present_position1)
    time.sleep(0.1)
##
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()

exit(1)



timer_flag = 0 #GLOBAL
torque = False #GLOBAL
control = False #GLOBAL
time_step_c = 0.1
max_value = 2047.0
action = [] #GLOBAL
send_data = [] #GLOBAL

def get():
    
    recv_data = sock.recv(1024)
    a = list(struct.unpack_from("13B",recv_data ,0))
    
    command = a[0]
    for i in range(len(a)):
        if i != 0:
            theta = a[i]
            a[i] = (-1*(((theta-0x01)^0xFF)) if (theta>>7)==1  else (theta & 0b01111111))*0.01
    
    return command,a[1:]

def send(target):

#waiting(バッファの確認、送信できるかの確認)処理を入れてない。将来的にthread等で実装求む
    theta = target[0:12]
    
    torque = target[12:]
    theta = [(1.25 if i >1.25 else i) for i in theta]
    theta = [(-1.25 if i <-1.25 else i) for i in theta]
    
    torque = [(0.3125 if i >0.3125 else i) for i in torque]
    torque = [(-0.3125 if i <-0.3125 else i) for i in torque]
    torque = [i*4.0 for i in torque]
    theta.extend(torque)
    theta = [int(i*100) & 0xFF for i in theta]
    # print(target)
    send_binary =bytes(theta) 
    # print(target)
    sock.sendall(send_binary)


def control_dyn():
    # time_now= time.time()
    
    global ac
    dxl_goal_position1 = middle_position + int(max_value*action[0]/pi)
    dxl_goal_position2 = middle_position + int(max_value*action[1]/pi)
    dxl_goal_position3 = middle_position + int(max_value*action[2]/pi)
    dxl_goal_position4 = middle_position + int(max_value*action[3]/pi)
    dxl_goal_position5 = middle_position + int(max_value*action[4]/pi)
    dxl_goal_position6 = middle_position + int(max_value*action[5]/pi)
    dxl_goal_position7 = middle_position + int(max_value*action[6]/pi)
    dxl_goal_position8 = middle_position + int(max_value*action[7]/pi)
    dxl_goal_position9 = middle_position + int(max_value*action[8]/pi)
    dxl_goal_position10 = middle_position + int(max_value*action[9]/pi)
    dxl_goal_position11 = middle_position + int(max_value*action[10]/pi)
    dxl_goal_position12 = middle_position + int(max_value*action[11]/pi)

    # print(dxl_goal_position2)
    param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position1)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position1)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position1)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position1))]
    param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position2)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position2)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position2)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position2))]
    param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position3)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position3)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position3)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position3))]
    param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position4)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position4)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position4)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position4))]
    param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position5)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position5)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position5)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position5))]
    param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position6)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position6)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position6)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position6))]
    param_goal_position7 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position7)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position7)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position7)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position7))]
    param_goal_position8 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position8)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position8)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position8)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position8))]
    param_goal_position9 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position9)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position9)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position9)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position9))]
    param_goal_position10= [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position10)),DXL_HIBYTE(DXL_LOWORD(dxl_goal_position10)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position10)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position10))]
    param_goal_position11= [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position11)),DXL_HIBYTE(DXL_LOWORD(dxl_goal_position11)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position11)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position11))]
    param_goal_position12= [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position12)),DXL_HIBYTE(DXL_LOWORD(dxl_goal_position12)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position12)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position12))]

    groupSyncWrite.addParam(DXL1_ID,param_goal_position1)
    groupSyncWrite.addParam(DXL2_ID,param_goal_position2)
    groupSyncWrite.addParam(DXL3_ID,param_goal_position3)
    groupSyncWrite.addParam(DXL4_ID,param_goal_position4)
    groupSyncWrite.addParam(DXL5_ID,param_goal_position5)
    groupSyncWrite.addParam(DXL6_ID,param_goal_position6)
    groupSyncWrite.addParam(DXL7_ID,param_goal_position7)
    groupSyncWrite.addParam(DXL8_ID,param_goal_position8)
    groupSyncWrite.addParam(DXL9_ID,param_goal_position9)
    groupSyncWrite.addParam(DXL10_ID,param_goal_position10)
    groupSyncWrite.addParam(DXL11_ID,param_goal_position11)
    groupSyncWrite.addParam(DXL12_ID,param_goal_position12)
    
    # global action
    # dxl_goal_positions = [middle_position + int(max_value*i/pi) for i in action]
    # #piが少数なので結局intのままでは計算できない

    # # print(dxl_goal_position2)
    # param_goal_positions = [[DXL_LOBYTE(DXL_LOWORD(i)), DXL_HIBYTE(DXL_LOWORD(i)), DXL_LOBYTE(DXL_HIWORD(i)), DXL_HIBYTE(DXL_HIWORD(i))] for i in dxl_goal_positions]

    # for dxl_id,param_goal_position in zip(DXL_IDs,param_goal_positions):
    #     groupSyncWrite.addParam(dxl_id,param_goal_position)

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()

    if dxl_comm_result != COMM_SUCCESS:
        print("koko_cd")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # sys.exit()

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    
    # print("control:")
    # print(time.time()-time_now)
    # time_now= time.time()

    get_servo()


def get_servo():
    # time_now= time.time()
    
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL4_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL5_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL6_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL7_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL8_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL9_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL10_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL11_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL12_ID)

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("koko")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    dxl_present_position1 = ((groupSyncRead.getData(DXL1_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position2 = ((groupSyncRead.getData(DXL2_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position3 = ((groupSyncRead.getData(DXL3_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position4 = ((groupSyncRead.getData(DXL4_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position5 = ((groupSyncRead.getData(DXL5_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position6 = ((groupSyncRead.getData(DXL6_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position7 = ((groupSyncRead.getData(DXL7_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position8 = ((groupSyncRead.getData(DXL8_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position9 = ((groupSyncRead.getData(DXL9_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position10 = ((groupSyncRead.getData(DXL10_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position11 = ((groupSyncRead.getData(DXL11_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position12 = ((groupSyncRead.getData(DXL12_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi

    groupSyncRead.clearParam()
    
    # # get_positions
    # dxl_addparam_results = []
    # for i in range(len(DXL_IDs)):
    #     dxl_addparam_results.append((groupSyncRead.addParam(DXL_IDs[i])))

    # dxl_comm_result = groupSyncRead.txRxPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("koko")
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     # sys.exit()

    # dxl_present_positions = [((groupSyncRead.getData(i,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi for i in DXL_IDs]

    # # print(dxl_present_current)
    # groupSyncRead.clearParam()
    
    
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL1_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL2_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL3_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL4_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL5_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL6_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL7_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL8_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL9_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL10_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL11_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL12_ID)

    dxl_comm_result = groupSyncRead_cur.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("koko")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    dxl_present_curent1 = s16(groupSyncRead_cur.getData(DXL1_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent2 = s16(groupSyncRead_cur.getData(DXL2_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent3 = s16(groupSyncRead_cur.getData(DXL3_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent4 = s16(groupSyncRead_cur.getData(DXL4_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent5 = s16(groupSyncRead_cur.getData(DXL5_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent6 = s16(groupSyncRead_cur.getData(DXL6_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent7 = s16(groupSyncRead_cur.getData(DXL7_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent8 = s16(groupSyncRead_cur.getData(DXL8_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent9 = s16(groupSyncRead_cur.getData(DXL9_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent10 = s16(groupSyncRead_cur.getData(DXL10_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent11 = s16(groupSyncRead_cur.getData(DXL11_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent12 = s16(groupSyncRead_cur.getData(DXL12_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001

    groupSyncRead_cur.clearParam()
    
    # dxl_addparam_results = []
    # for i in range(len(DXL_IDs_t)):
    #     dxl_addparam_results.append((groupSyncRead_cur.addParam(DXL_IDs_t[i])))
    
    # dxl_comm_result = groupSyncRead_cur.txRxPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("koko")
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
    # dxl_present_current = [s16(groupSyncRead_cur.getData(i,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001 for i in DXL_IDs_t]
    
    # groupSyncRead_cur.clearParam()

    global send_data
    # send_data = dxl_present_positions
    # send_data.extend(dxl_present_current)
    send_data =[
    dxl_present_position1
    ,dxl_present_position2
    ,dxl_present_position3
    ,dxl_present_position4
    ,dxl_present_position5
    ,dxl_present_position6
    ,dxl_present_position7
    ,dxl_present_position8
    ,dxl_present_position9
    ,dxl_present_position10
    ,dxl_present_position11
    ,dxl_present_position12
    ,dxl_present_curent1
    ,dxl_present_curent2
    ,dxl_present_curent3
    ,dxl_present_curent4
    ,dxl_present_curent5
    ,dxl_present_curent6
    ,dxl_present_curent7
    ,dxl_present_curent8
    ,dxl_present_curent9
    ,dxl_present_curent10
    ,dxl_present_curent11
    ,dxl_present_curent12
    ]
    # print("get_srvo:")
    # print(time.time()-time_now)
    send(send_data)
    print("data")
    print(send_data)


# def communication(arg1,arg2):
#     global timer_flag
#     timer_flag = 0

def senddata():
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL4_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL5_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL6_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL7_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL8_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL9_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL10_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL11_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL12_ID)

    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("koko")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    dxl_present_position1 = ((groupSyncRead.getData(DXL1_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position2 = ((groupSyncRead.getData(DXL2_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position3 = ((groupSyncRead.getData(DXL3_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position4 = ((groupSyncRead.getData(DXL4_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position5 = ((groupSyncRead.getData(DXL5_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position6 = ((groupSyncRead.getData(DXL6_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position7 = ((groupSyncRead.getData(DXL7_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position8 = ((groupSyncRead.getData(DXL8_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position9 = ((groupSyncRead.getData(DXL9_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position10 = ((groupSyncRead.getData(DXL10_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position11 = ((groupSyncRead.getData(DXL11_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi
    dxl_present_position12 = ((groupSyncRead.getData(DXL12_ID,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi

    groupSyncRead.clearParam()
    
    # # get_positions
    # dxl_addparam_results = []
    # for i in range(len(DXL_IDs)):
    #     dxl_addparam_results.append((groupSyncRead.addParam(DXL_IDs[i])))

    # dxl_comm_result = groupSyncRead.txRxPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("koko")
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     # sys.exit()

    # dxl_present_positions = [((groupSyncRead.getData(i,ADDR_XH430_PRESENT_POSITION,LEN_XH430_PRESENT_POSITION) - middle_position)/max_value)*pi for i in DXL_IDs]

    # # print(dxl_present_current)
    # groupSyncRead.clearParam()
    
    
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL1_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL2_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL3_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL4_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL5_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL6_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL7_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL8_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL9_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL10_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL11_ID)
    dxl_addparam_result = groupSyncRead_cur.addParam(DXL12_ID)

    dxl_comm_result = groupSyncRead_cur.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("koko")
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    dxl_present_curent1 = s16(groupSyncRead_cur.getData(DXL1_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent2 = s16(groupSyncRead_cur.getData(DXL2_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent3 = s16(groupSyncRead_cur.getData(DXL3_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent4 = s16(groupSyncRead_cur.getData(DXL4_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent5 = s16(groupSyncRead_cur.getData(DXL5_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent6 = s16(groupSyncRead_cur.getData(DXL6_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent7 = s16(groupSyncRead_cur.getData(DXL7_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent8 = s16(groupSyncRead_cur.getData(DXL8_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent9 = s16(groupSyncRead_cur.getData(DXL9_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent10 = s16(groupSyncRead_cur.getData(DXL10_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent11 = s16(groupSyncRead_cur.getData(DXL11_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001
    dxl_present_curent12 = s16(groupSyncRead_cur.getData(DXL12_ID,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001

    groupSyncRead_cur.clearParam()
    
    # dxl_addparam_results = []
    # for i in range(len(DXL_IDs_t)):
    #     dxl_addparam_results.append((groupSyncRead_cur.addParam(DXL_IDs_t[i])))
    
    # dxl_comm_result = groupSyncRead_cur.txRxPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("koko")
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
    # dxl_present_current = [s16(groupSyncRead_cur.getData(i,ADDR_XH430_PRESENT_CURRENT,LEN_XH430_PRESENT_CURRENT))*2.69*0.001 for i in DXL_IDs_t]
    
    # groupSyncRead_cur.clearParam()

    global send_data
    # send_data = dxl_present_positions
    # send_data.extend(dxl_present_current)
    send_data =[
    dxl_present_position1
    ,dxl_present_position2
    ,dxl_present_position3
    ,dxl_present_position4
    ,dxl_present_position5
    ,dxl_present_position6
    ,dxl_present_position7
    ,dxl_present_position8
    ,dxl_present_position9
    ,dxl_present_position10
    ,dxl_present_position11
    ,dxl_present_position12
    ,dxl_present_curent1
    ,dxl_present_curent2
    ,dxl_present_curent3
    ,dxl_present_curent4
    ,dxl_present_curent5
    ,dxl_present_curent6
    ,dxl_present_curent7
    ,dxl_present_curent8
    ,dxl_present_curent9
    ,dxl_present_curent10
    ,dxl_present_curent11
    ,dxl_present_curent12
    ]
    # print("get_srvo:")
    # print(time.time()-time_now)
    send(send_data)
    print("data")
    print(send_data)
    # global timer_flag
    # timer_flag = 0

def enable_control():
    print("enable_control")
    global control
    control = True
    global timer_flag
    timer_flag = 1
    control_dyn()
    timer_flag = 0

# def enable_control():
#     print("enable_control")
#     global control
#     control = True
#     global timer_flag
#     timer_flag = 1
#     signal.setitimer(signal.ITIMER_REAL, time_step_c, 1.0)
#     control_dyn()
#     while True:
#         # global timer_flag
#         if timer_flag == 0:
#             break
#     signal.setitimer(signal.ITIMER_REAL, time_step_c, 0)

def disable_control():
    print("disable_control")
    global control
    control = False

def enable_torque():
    # Enable Dynamixel#1-#4 Torque
    print("enable_torque\n")
    packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)#ADDR_XH430_TORQUE_ENABLE=64 TORQUE_ENABLE=1
    packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL10_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL11_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL12_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_ENABLE)
    global torque
    torque = True

def disable_torque():

    print("disable_torque\n")
    packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL10_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL11_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, DXL12_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
    global torque
    torque = False

# def finderror():
#     print("error status:")
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL1_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL2_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL3_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL4_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL5_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL6_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL7_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL8_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL9_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL10_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL11_ID)
#     dxl_addparam_result = groupSyncRead_err.addParam(DXL12_ID)

#     dxl_comm_result = groupSyncRead_err.txRxPacket()
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

#     print(groupSyncRead_err.getData(DXL1_ID,70,1))
#     print(groupSyncRead_err.getData(DXL2_ID,70,1))
#     print(groupSyncRead_err.getData(DXL3_ID,70,1))
#     print(groupSyncRead_err.getData(DXL4_ID,70,1))
#     print(groupSyncRead_err.getData(DXL5_ID,70,1))
#     print(groupSyncRead_err.getData(DXL6_ID,70,1))
#     print(groupSyncRead_err.getData(DXL7_ID,70,1))
#     print(groupSyncRead_err.getData(DXL8_ID,70,1))
#     print(groupSyncRead_err.getData(DXL9_ID,70,1))
#     print(groupSyncRead_err.getData(DXL10_ID,70,1))
#     print(groupSyncRead_err.getData(DXL11_ID,70,1))
#     print(groupSyncRead_err.getData(DXL12_ID,70,1))

#     groupSyncRead_err.clearParam()

def s32(value):
    return -(value & 0b10000000000000000000000000000000) | (value & 0b01111111111111111111111111111111)

def s16(value):
    return -(value & 0b1000000000000000) | (value & 0b0111111111111111)


print("send initialize observation data\n")
senddata()
# signal.signal(signal.SIGALRM, communication)
# time.sleep(5.0)

try:
    while True:
        # base_time = time.time()
        command,data = get()
        print("receive data")
        action = data
        # finderror()

        # print("command:"+str(command))
        # print("action:"+str(action))
        if command == 0 and torque == False and control == False:
            #innactive 2 innactive
            print("this command execute innactive to innactive")
        elif command == 0 and torque == True and control == False:
            #active 2 innactive
            disable_torque()
        elif command == 0 and torque == True and control == True:
            #move 2 innactive
            print("accept PC's stop command")
            disable_control()
            # finderror()
            disable_torque()
        elif command == 1 and torque == False and control == False:
            #innactive 2 active
            print("accept PC's reset command")
            enable_torque()
            senddata()
            senddata()
        elif command == 1 and torque == True and control == False:
            #active to active
            print("error:maybe you code reset() twice")
        elif command == 1 and torque == True and control == True:
            #move 2 active
            disable_control()
        elif command == 2 and torque == False and control == False:
            #[error] innactive 2 move
            print("error:trying to go to move phase by innactive ph")
        elif command == 2 and torque == True and control == False:
            #active to move
            print("start rollout")
            enable_control()
        elif command == 2 and torque == True and control == True:
            #move to move
            print("accept PC's step command")
            print(action)
            enable_control()
        else :
            print("error of error")
            # print(command)
            # print(torque)
            # print(control)
        # print(time.time() - base_time)


except KeyboardInterrupt:

    if torque == True:
        disable_torque()

    # Close port
    portHandler.closePort()
    # close socket
    sock.close()

    print("Well done!")
    os.kill(os.getpid(),signal.SIGKILL)
    pass


# Clear syncread parameter storage
groupSyncRead.clearParam()
groupSyncRead_cur.clearParam()

# Disable Dynamixel#1-#10 Torque
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL10_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL11_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL12_ID,ADDR_XH430_TORQUE_ENABLE, TORQUE_DISABLE)



# Close port
portHandler.closePort()
sock.close()