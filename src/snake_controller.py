#Dynamixelのライブラリのパスを通す
import sys
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

from dynamixel_sdk import *   # Uses Dynamixel SDK library

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
from math import pi,sin,cos
from copy import deepcopy

ADDR_XH430_TORQUE_ENABLE   = 64

ADDR_XH430_POSITION_D_GAIN = 120
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

DXL_MOVING_STATUS_THRESHOLD = 30                # Dynamixel moving status threshold, 20だと小さすぎ


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

ESC_ASCII_VALUE             = 0x1b
s_ASCII_VALUE               = 0x73
q_ASCII_VALUE               = 0x71

middle_position       = 2047#0~4095

CDC_P_GAIN = 300

ADDR_XH430_GOAL_POSITION = 116 #レジスタの位置
LEN_XH430_GOAL_POSITION = 4 #レジスタの大きさ


class SnakeController():
    """
    蛇ロボットをコントロールするクラス
    """

    def __init__(self):

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows

        self.port_handler = PortHandler(DEVICENAME)#DEVICENAME = '/dev/ttyUSB0'

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler

        self.packet_handler = PacketHandler(PROTOCOL_VERSION)#2

        # Initialize GroupSyncWrite instance
        self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_XH430_GOAL_POSITION, LEN_XH430_GOAL_POSITION)
        
        # Initialize GroupSyncRead instace for Present Position
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_XH430_PRESENT_POSITION, LEN_XH430_PRESENT_POSITION)
        self.group_sync_read_cur = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_XH430_PRESENT_CURRENT, LEN_XH430_PRESENT_CURRENT)

        # self.goal_positions=[middle_position for _ in range(len(DXL_IDs))] #それぞれのDynamixelのgoal角度

        # Open port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        time.sleep(1.0)

        while True:
            print("Press [s] to start!")
            if getch() == chr(s_ASCII_VALUE):
                break

        print("set p gain of position control\n")
        print(self.packet_handler.write2ByteTxRx(self.port_handler, DXLALL_ID, ADDR_XH430_POSITION_P_GAIN, CDC_P_GAIN))#ADDR_XH430_POSITION_P_GAIN = 84,CDC_P_GAIN = 300

        # print("set degree range of position control\n")
        print("==================================================")
        print(self.packet_handler.write4ByteTxRx(self.port_handler, DXLALL_ID, ADDR_XH430_MAX_POSITION_LIMIT,DXL_MAXIMUM_POSITION_VALUE))#+40 degree
        print(self.packet_handler.write4ByteTxRx(self.port_handler, DXLALL_ID, ADDR_XH430_MIN_POSITION_LIMIT,DXL_MINIMUM_POSITION_VALUE))#-40 degree


    def torque_enable(self):
        """
        Dynamixelを制御可能にする関数\n
        コントロールする前に必ずこれを行う
        """
        for ID in DXL_IDs:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,ID,
                ADDR_XH430_TORQUE_ENABLE,TORQUE_ENABLE
            )

    def torque_disable(self):
        """
        Dynamixelの制御を終わりにする関数\n
        コントロールが終わったら必ずやる\n
        やらないと蛇が硬いままになる(モータにトルクがかかった状態になる)
        """
        for ID in DXL_IDs:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,ID,
                ADDR_XH430_TORQUE_ENABLE,TORQUE_DISABLE
            )

    def set_initial_position(self):
        """
        蛇をまっすぐにする関数
        """
        self.torque_enable()

        goal_positions=[middle_position for _ in range(len(DXL_IDs))]
        self.control(goal_positions=goal_positions)

        self.torque_disable()


    def control(self,goal_positions):
        """
        goal角度までサーボを動かす関数
        """

        # previos_positions=self.read_present_position() #動かす前の角度
        present_position=self.read_present_position()#動かした後の角度
        self.write_goal_position(goal_positions=goal_positions)

        is_goal=False #目標角度に達したか？
        count=0
        # time.sleep(0.03)
        while not is_goal:
            count+=1

            previos_positions=present_position#動かす前の角度
            # self.write_goal_position(goal_positions=goal_positions)
            present_position=self.read_present_position() #動かした後の角度
            #角度が変化したかどうか
            is_update=True if np.sum(np.abs(np.array(previos_positions)-np.array(present_position)))>=20 else False
            if not is_update: #変化してないならおしまい
                # print("positions are not changed")
                break
            # print("==================")
            for i in range(len(DXL_IDs)): #変化してるなら,ちゃんとgoalに到達してるかチェック
                # print(goal_positions[i],present_position[i])
                #1つでもまだゴールに達してないやつがいたら,さらにサーボを動かす
                if abs(goal_positions[i]-present_position[i])>=DXL_MOVING_STATUS_THRESHOLD:
                    is_goal=False
                    break
                #
                is_goal=True #全部goalに達してたらOK
            time.sleep(0.01)
            if count>1:
                break

        self.group_sync_write.clearParam() #パラメータのリセット
        print("loop count:",count)
        
    def end_control(self):
        """
        コントロールが終わったら必ずやる関数
        """
        self.group_sync_write.clearParam() #パラメータのリセット
        self.group_sync_read.clearParam() #パラメータのリセット
        self.torque_disable()
        self.port_handler.closePort()
        
        

    def write_goal_position(self,goal_positions:list):
        """
        Dynamixelにgoal角度を書き込む関数\n
        書き込むだけだから,本当にgoal角度に到達したかはわからない

        :param goal_positions: goal角度
        :type goal_positions: list [12] (numpyも可)
        """

        for i,ID in enumerate(DXL_IDs):
            param_goal_position=[
                DXL_LOBYTE(DXL_LOWORD(goal_positions[i])),
                DXL_HIBYTE(DXL_LOWORD(goal_positions[i])),
                DXL_LOBYTE(DXL_HIWORD(goal_positions[i])),
                DXL_HIBYTE(DXL_HIWORD(goal_positions[i]))
            ] #goal角度の準備
            self.group_sync_write.addParam(ID,param_goal_position) #goal角度をパラメータとして追加
            # break #一旦1つのサーボで実験する
        dxl_comm_result=self.group_sync_write.txPacket() #Dynamixelを動かす

        if not dxl_comm_result==COMM_SUCCESS: #上手くいかなかったらエラー出力
            print(self.packet_handler.getTxRxResult(dxl_comm_result))

        # self.group_sync_write.clearParam() #パラメータのリセット

    
    def read_present_position(self):
        """
        Dynamixelの今の角度を読み取る関数\n
        これを使ってちゃんとgoal角度に達しているか確認する

        :return present_positions: 今のDynamixelの角度
        """
        for ID in DXL_IDs:
            self.group_sync_read.addParam(ID)
        self.group_sync_read.txRxPacket()

        present_positions=[
            self.group_sync_read.getData(
                    ID,ADDR_XH430_PRESENT_POSITION,
                    LEN_XH430_PRESENT_POSITION
                )
            for ID in DXL_IDs
        ]

        return present_positions


    def update_goal_positions(
        self,t,goal_positions,
        theta_base=middle_position,
        theta_amp=(DXL_MAXIMUM_POSITION_VALUE-middle_position)*0.7,
        omega=2*pi/(1.7*10.0**6)
        ):
        """
        goal角度をアップデートする関数\n
        以下の式で先頭のDynamixelのgoal角度を計算する\n
        残りの角度はシフトすることで決定する

        θ_goal=θ_base + θ_amp*sin(ωt)

        :param t: 現在時刻 [μs]

        """

        theta_head_goal=int(theta_base+theta_amp*sin(omega*t))
        tmp=deepcopy(goal_positions)
        goal_positions[0]=theta_head_goal #先頭のgoal角度をupdate
        goal_positions[1:]=tmp[0:-1] #残りの角度はシフトする

        return goal_positions


    