#!/usr/bin/env python3
import time
from enum import Enum
from threading import Thread
from typing import NamedTuple

import rclpy
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand
from rclpy.node import Node
from rclpy.task import Future
# from YoloDetector import YoloDetectorActionClient
import tkinter as tk
import ordertest
import threading

# 點餐界面

order_count=0
water_count=1
DEFAULT_VELOCITY = 100
DEFAULT_ACCELERATION =100
LINE_VELOCITY = 100
LINE_ACCELERATION = 100
VACUUM_PIN1 = 1
VACUUM_PIN2 = 2
PUMP = 3

cup_pose = [427.14, 361.8, 210.656, 179.372, 0.664, 89.557]               #飲料杯取料點
cup_work_pose = [264.386, 376.426, 202.488, 179.373, 0.668, 89.551]       #飲料杯工作區
# cup_work_up_pose=[356.652,189.788,105.152,179.794,0.594,90.307]
cup_lid_pose = [52.648, 219.663, 99.106, 179.372, 0.663, 89.557]          #飲料蓋取料點
cup_lid_work_pose = [264.635, 376.659, 232.367, 179.373, 0.663, 89.547]   #飲料杯工作區

#cuppushtool8
# cup_push_pose1 = [265.791, 423.304, 51.264, 179.373, 0.668, 89.551]
# cup_push_pose2 = [265.791, 423.304, 51.264, 179.336, 5.373, 89.549]
# cup_push_pose3 = [265.791, 423.304, 41.522, 179.336, 5.373, 89.549]
# cup_push_pose4 =[265.791, 423.304, 40.429, 179.368, 1.32, 89.551]
# cup_push_pose5 = [265.791, 423.304, 40.429, 179.378, 0.023, 89.551]
# cup_push_pose6 = [267.635, 390.659, 232.367, 179.373, 0.663, 89.547]
cup_push_pose1 = [265.915, 429.589, 48.21, 179.378, 0.033, 89.551]        #壓杯蓋步驟
cup_push_pose2 = [265.915, 429.589, 48.21, 179.336, 5.335, 89.549]
cup_push_pose3 = [265.915, 429.589, 38.192, 179.336, 5.335, 89.549]
cup_push_pose4 = [265.915, 421.589, 38.192, 179.368, 5.335, 89.551]
cup_push_pose5 = [265.915, 421.589, 38.192, 179.378, 0.036, 89.551]
cup_push_pose6 = [265.915, 421.589, 34.398, 179.378, 0.036, 89.551]
cup_push_pose7 = [265.915, 421.589, 39.998, 179.378, 0.035, 89.551]
cup_push_pose8 = [265.915, 421.589, 33.234, 179.378, 0.035, 89.551]

sause_pose = [421.728, 181.31, 181.324, 179.372, 0.664, 89.557]           #醬料疊取料點
sause_work_pose = [264.386, 373.426, 168.511, 179.378, 0.035, 89.551]     #醬料疊工作區
sause_lid_pose = [230.97, 215.082, 115.753, 179.373, 0.666, 89.557]       #醬料蓋取料點
sause_lid_work_pose = [264.386, 373.426, 140.117, 179.378, 0.035, 89.551] #醬料蓋工作區
sause_push_pose = [264.386, 376.426, 130.994, 179.378, 0.035, 89.551]     #壓醬料蓋

home_joint = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint                #home點joint
home_pose = [314.951,216.448,414.945,179.794,0.594,90.307]                #home點pose
# OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572] #pose
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint

# water_up_pose=[277.54,199.47,178.13,180.00,0.00,90.00]
sause_work_up_pose=[203.557,189.788,105.152,179.794,0.594,90.307]
# water_pose1=[263.977, 378.515, 249.824, 179.373, 0.664, 89.548]
water_pose1=[241.689, 196.261, 272.64, 179.373, 0.664, 89.548]            #倒水步驟
water_pose2=[43.476, 210.261, 198.852, 179.373, 0.664, 89.548]
water_pose3 = [13.473, 281.922, 198.856, 179.374, 0.663, 89.547]
water_cup_going_pose=[356.652,189.788,105.152,-120.734,0.594,90.307]
water_sause_going_pose=[203.557,189.788,105.152,-120.734,0.594,90.307]

cupfinish = [624.337, 316.785, 170.025, 179.373, 0.664, 89.548]           #飲料出餐區
sausefinish = [639.569, 318.52, 112.867, 179.372, 0.662, 89.547]          #醬料出餐區

cup_finish_pose_1=[595.791,-466.855,172.666,179.794,0.594,90.307]
cup_finish_pose_2=[731.533,-466.855,172.666,179.794,0.594,90.307]
cup_finish_pose_3=[865.939,-466.855,172.666,179.794,0.594,90.307]
sause_finish_pose_1=[595.791,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_2=[731.533,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_3=[865.939,-466.855,140.359,179.794,0.594,90.307]
# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times
NUM_OBJECTS = 5


print(cup_pose,cup_lid_pose,sause_pose,sause_lid_pose)

class States(Enum):
    INIT = 0
    FINISH = 1
    gohome = 2
    water_up = 3
    MOVE_TO_OPJECT_TOP = 4
    PICK_OBJECT = 5
    MOVE_TO_PLACE_POSE = 6
    CHECK_POSE = 7
    CLOSE_ROBOT = 8
    CUP_WATER_23 = 9
    suck_water=10
    suck_water_up=11
    water_cup_up=12
    water_return=13
    back_water_up=14
    back_water_up_up=15
    CUP_WATER_4 = 16
    water_pump=17
    CUP_WATER_5 = 18
    CUP_WATER_1 = 19
    SUCK_CUP_TO_WORK = 20
    SUCK_CUP_LID_TO_WORK = 21
    SUCK_SAUSE_TO_WORK = 22
    SUCK_SAUSE_LID_TO_WORK = 23
    water_sause_up = 24
    SAUSE_WATER_1 = 25
    SAUSE_WATER_23 = 26
    SAUSE_WATER_4 = 27
    SAUSE_WATER_5 = 28
    SAUSE_FINISH = 29
    CUP_FINISH = 30
class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')
        self.object_pose = None
        self.object_cnt = 0
    
    def move(self,PLJ,six_num,velocity=DEFAULT_VELOCITY,acceleration=DEFAULT_ACCELERATION,tool=1,base=30,holding=True,velocity_L=LINE_VELOCITY,acceleration_L=LINE_ACCELERATION):
        self.mode=PLJ
        self.six_number=six_num
        self.v=velocity
        self.a=acceleration
        self.toolnum=tool
        self.basenum=base
        self.hold=holding
        self.vl=velocity_L
        self.al=acceleration_L
        if 'P' in self.mode:
            self.object_pose = self.six_number
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = self.object_pose[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = self.object_pose[3:6]
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP
                ,velocity=self.v,acceleration=self.a,
                pose=pose,
                tool=self.toolnum,
                base=self.basenum,
                holding=self.hold)
            res = self.call_hiwin(req)
        elif 'L' in self.mode:
            self.object_pose = self.six_number
            pose = Twist()
            [pose.linear.x, pose.linear.y, pose.linear.z] = self.object_pose[0:3]
            [pose.angular.x, pose.angular.y, pose.angular.z] = self.object_pose[3:6]
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                velocity=self.vl,
                acceleration=self.al,
                pose=pose,
                tool=self.toolnum,
                base=self.basenum,
                holding=self.hold)
            res = self.call_hiwin(req)
        elif'J' in self.mode:
            req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=self.six_number
                ,velocity=self.v,acceleration=self.a,
                tool=self.toolnum,
                base=self.basenum,
                holding=self.hold
                )
            res = self.call_hiwin(req)
        else:
            print("--------------fuckyou")
            time.sleep(50000)
        return res
    def robot_wait(self):
        req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.WAITING
            )
        res = self.call_hiwin(req)
        return res
    def vacuum_control(self,number,OF,holding=True, do_timer = 0):
        self.num=number
        self.mode=OF
        self.hold=holding
        self.do_timer=do_timer
        if self.num==VACUUM_PIN1:
            if self.mode=='ON':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_ON,
                        digital_output_pin=VACUUM_PIN1,
                        holding=self.hold
                        # pose=pose
                    )
                res = self.call_hiwin(req)
            elif self.mode=='OFF':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
                        digital_output_pin=VACUUM_PIN1,
                        holding=self.hold
                        # pose=pose
                    )
                    
                res = self.call_hiwin(req)
        elif self.num==VACUUM_PIN2:
            if self.mode=='ON':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_ON,
                        digital_output_pin=VACUUM_PIN2,
                        holding=self.hold
                        # pose=pose
                    )
                    
                res = self.call_hiwin(req)
            elif self.mode=='OFF':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
                        digital_output_pin=VACUUM_PIN2,
                        holding=self.hold
                        # pose=pose
                    )
                    
                res = self.call_hiwin(req)
        elif self.num==PUMP:
            if self.mode=='ON':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_ON,
                        digital_output_pin=PUMP,
                        holding=self.hold,
                        do_timer=self.do_timer
                        # pose=pose
                    )
                    
                res = self.call_hiwin(req)
            elif self.mode=='OFF':
                req = self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.DIGITAL_OUTPUT,
                        digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
                        digital_output_pin=PUMP,
                        holding=self.hold
                        # pose=pose
                    )
                    
                res = self.call_hiwin(req)
        return res
    def jaw(self,mode):
        self.mode=mode
        if self.mode=='open':
            res=self.vacuum_control(VACUUM_PIN1,'OFF',holding=False)
            res=self.vacuum_control(VACUUM_PIN2,'ON',holding=False)
        elif self.mode=='close':
            res=self.vacuum_control(VACUUM_PIN2,'OFF',holding=False)
            res=self.vacuum_control(VACUUM_PIN1,'ON',holding=False)
        return res
    def fillwater(self):
        res=self.vacuum_control(VACUUM_PIN1,'OFF',holding=False)
        res=self.vacuum_control(VACUUM_PIN2,'OFF',holding=False)
        res=self.vacuum_control(PUMP,'ON',holding=False)
        time.sleep(2.5)
        res=self.vacuum_control(PUMP,'OFF',holding=False)
    def _state_machine(self, state: States) -> States:
        global order_count
        global water_count
        if state == States.INIT:
            self.get_logger().info('INIT')
            # res=self.vacuum_control(VACUUM_PIN1,'ON')
            # res=self.vacuum_control(VACUUM_PIN1,'OFF')
            # res=self.vacuum_control(VACUUM_PIN2,'ON')
            # res=self.vacuum_control(VACUUM_PIN2,'OFF')
            # res=self.vacuum_control(PUMP,'OFF')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.READ_DI,
                digital_input_pin=2,
                holding=False
                # pose=pose
            )
            res = self.call_hiwin(req)
            while res.digital_state!=True:
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.READ_DI,
                    digital_input_pin=2,
                    holding=False
                    # pose=pose
                )
                res = self.call_hiwin(req)
                if res.digital_state==True:
                    nest_state = States.gohome
                else:
                    nest_state=States.INIT
                    pass
            nest_state = States.gohome
        elif state == States.gohome:
            self.get_logger().info('gohome')
            # res=self.move('J',home_joint,holding=False)
            res=self.move('J',home_joint,holding=True)
            # res=self.move('P',home_pose,holding=False)
            
            if order[order_count][0]+order[order_count][1]>0:
                if order[order_count][1]>0:
                    nest_state = States.SUCK_SAUSE_TO_WORK
                else:
                    nest_state = States.SUCK_CUP_TO_WORK
            elif order_count<4:
                order_count=order_count+1
                cupfinish[1]-=150
                sausefinish[1]-=150
                # cup_finish_pose_1[1]+=125.00
                # cup_finish_pose_2[1]+=125.00
                # cup_finish_pose_3[1]+=125.00
                # sause_finish_pose_1[1]+=125.00
                # sause_finish_pose_2[1]+=125.00
                # sause_finish_pose_3[1]+=125.00
                nest_state = States.gohome
            else:
                nest_state = States.CLOSE_ROBOT
        elif state == States.SUCK_CUP_TO_WORK:
            self.get_logger().info('SUCK_CUP_TO_WORK')
            res=self.jaw('open')
            cup_pose[2]+=30
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=30
            cup_pose[2]+=20
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=20
            cup_pose[2]+=10
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=10
            res=self.move('P',cup_pose)
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            cup_pose[2]+=2
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=2
            cup_pose[2]+=5
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=5
            cup_pose[2]+=10
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=10
            cup_pose[2]+=20
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=20
            cup_pose[2]+=30
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=30
            cup_pose[2]+=40
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=40
            cup_pose[2]+=50
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=50
            cup_pose[2]+=60
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=60
            cup_pose[2]+=80
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=80
            cup_pose[2]+=100
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=100
            cup_pose[2]+=120
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=120
            cup_pose[2]+=140
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=140
            cup_pose[2]+=160
            res=self.move('P',cup_pose,holding=False)
            cup_pose[2]-=160

            cup_work_pose[2]+=160
            res=self.move('P',cup_work_pose,holding=False)
            cup_work_pose[2]-=160
            cup_work_pose[2]+=80
            res=self.move('P',cup_work_pose,holding=False)
            cup_work_pose[2]-=80
            res=self.move('P',cup_work_pose)
            # res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            # cup_work_pose[2]+=30
            # res=self.move('P',cup_work_pose,holding=True)
            # cup_work_pose[2]-=30
            # cup_work_pose[2]+=120
            # res=self.move('P',cup_work_pose,holding=False)
            # cup_work_pose[2]-=120
            nest_state = States.water_up
        elif state == States.SUCK_SAUSE_TO_WORK:
            self.get_logger().info('SUCK_SAUSE_TO_WORK')
            res=self.jaw('open')
            sause_pose[2]+=30
            res=self.move('P',sause_pose,holding=False)
            sause_pose[2]-=30
            res=self.move('L',sause_pose)
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            sause_pose[2]+=50
            res=self.move('L',sause_pose,holding=False)
            sause_pose[2]-=50
            sause_pose[2]+=140
            res=self.move('P',sause_pose,holding=False)
            sause_pose[2]-=140
            sause_work_pose[2]+=140
            res=self.move('P',sause_work_pose,holding=False)
            sause_work_pose[2]-=140
            sause_work_pose[2]+=110
            res=self.move('P',sause_work_pose,holding=False)
            sause_work_pose[2]-=110
            res=self.move('L',sause_work_pose)
            # res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            sause_work_pose[2]+=140
            res=self.move('L',sause_work_pose,holding=True)
            sause_work_pose[2]-=140
            # sause_work_pose[2]+=200
            # res=self.move('P',sause_work_pose,holding=False)
            # sause_work_pose[2]-=200
            nest_state = States.water_up
        elif state == States.water_up:
            self.get_logger().info('water_up')
            res=self.jaw('close')
            res=self.move('P',water_pose1,holding=False)
            nest_state = States.suck_water   
        elif state == States.suck_water:
            self.get_logger().info('suck_water')
            res=self.move('P',water_pose2,holding=False)    
            nest_state = States.suck_water_up
        elif state == States.suck_water_up:
            self.get_logger().info('suck_water_up')
            res=self.move('P',water_pose3,holding=False)
            if order[order_count][1]>0:
                nest_state = States.water_sause_up
            else:
                nest_state = States.water_cup_up
        elif state == States.water_pump:
            self.get_logger().info('water_pump')
            
            
            nest_state = States.suck_water_up
        elif state == States.water_cup_up:
            self.get_logger().info('water_cup_up')
            if water_count<2:
                nest_state = States.CUP_WATER_1
            elif water_count<4:
                nest_state = States.CUP_WATER_23
            elif water_count<5:
                nest_state = States.CUP_WATER_4
            elif water_count<6:
                nest_state = States.CUP_WATER_5
        elif state == States.water_sause_up:
            self.get_logger().info('water_sause_up')
            if water_count<2:
                nest_state = States.SAUSE_WATER_1
            elif water_count<4:
                nest_state = States.SAUSE_WATER_23
            elif water_count<5:
                nest_state = States.SAUSE_WATER_4
            elif water_count<6:
                nest_state = States.SAUSE_WATER_5
        elif state == States.CUP_WATER_1:
            self.get_logger().info('CUP_WATER_1')
            water_pose3[0]+=80
            water_pose3[2]+=155
            res=self.move('L',water_pose3,holding=True)
            water_pose3[2]-=155
            water_pose3[0]-=80
            time.sleep(0.5)
            res=self.move('L',water_pose3,holding=False)
            nest_state = States.water_return
        elif state == States.CUP_WATER_23:
            self.get_logger().info('CUP_WATER_23')
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            # water_cup_going_pose[3]-=1
            # res=self.move('P',water_cup_going_pose,velocity_L=10,acceleration_L=10)
            # req = self.generate_robot_request(
            #     cmd_mode=RobotCommand.Request.WAITING
            # )
            # res = self.call_hiwin(req)
            water_cup_going_pose[3]+=2
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            water_cup_going_pose[3]-=4
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(1.4)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.CUP_WATER_4:
            self.get_logger().info('CUP_WATER_4')
            water_cup_going_pose[3]+=6
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            # water_cup_going_pose[3]-=2
            # res=self.move('P',water_cup_going_pose,velocity_L=10,acceleration_L=10)
            water_cup_going_pose[3]-=8
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(0.6)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.CUP_WATER_5:
            self.get_logger().info('CUP_WATER_5')
            water_cup_going_pose[3]+=12.3
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_cup_going_pose[3]+=1
            res=self.move('L',water_cup_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            # # req = self.generate_robot_request(
            # #     cmd_mode=RobotCommand.Request.WAITING
            # # )
            # # res = self.call_hiwin(req)
            # water_cup_going_pose[3]-=2
            # res=self.move('P',water_cup_going_pose,velocity_L=10,acceleration_L=10)
            water_cup_going_pose[3]-=14.3
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(1)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.SAUSE_WATER_1:
            self.get_logger().info('SAUSE_WATER_1')
            water_pose3[2]+=60
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[2]-=60
            water_pose3[0]+=35
            water_pose3[2]+=120
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=35
            water_pose3[2]-=120
            water_pose3[2]+=140
            water_pose3[0]+=50
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=50
            water_pose3[2]-=140
            water_pose3[0]+=60
            water_pose3[2]+=155
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=60
            water_pose3[2]-=155
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            nest_state = States.water_return
        elif state == States.SAUSE_WATER_23:
            self.get_logger().info('SAUSE_WATER_23')
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            # water_sause_going_pose[3]-=1
            # res=self.move('P',water_sause_going_pose,velocity_L=10,acceleration_L=10)
            # req = self.generate_robot_request(
            #     cmd_mode=RobotCommand.Request.WAITING
            # )
            # res = self.call_hiwin(req)
            water_sause_going_pose[3]+=2
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            water_sause_going_pose[3]-=4
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(1.4)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.SAUSE_WATER_4:
            self.get_logger().info('SAUSE_WATER_4')
            water_sause_going_pose[3]+=6
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            # water_sause_going_pose[3]-=2
            # res=self.move('P',water_sause_going_pose,velocity_L=10,acceleration_L=10)
            water_sause_going_pose[3]-=8
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(0.6)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.SAUSE_WATER_5:
            self.get_logger().info('SAUSE_WATER_5')
            water_sause_going_pose[3]+=12.3
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            res=self.robot_wait()
            water_sause_going_pose[3]+=1
            res=self.move('L',water_sause_going_pose,velocity_L=5,acceleration_L=5,tool=3)
            # # req = self.generate_robot_request(
            # #     cmd_mode=RobotCommand.Request.WAITING
            # # )
            # # res = self.call_hiwin(req)
            # water_sause_going_pose[3]-=2
            # res=self.move('P',water_sause_going_pose,velocity_L=10,acceleration_L=10)
            water_sause_going_pose[3]-=14.3
            res=self.robot_wait()
            if res.arm_state == RobotCommand.Response.IDLE:
                time.sleep(1)
                nest_state = States.water_return
            else:
                nest_state = None
        elif state == States.water_return:
            self.get_logger().info('water_return')
            cup_lid_pose[2]+=80
            res=self.move('P',cup_lid_pose,holding=True)
            cup_lid_pose[2]-=80
            # res=self.move('P',water_pose1,holding=False)
            # water_count=water_count+1
            nest_state = States.back_water_up_up
        elif state == States.back_water_up_up:
            self.get_logger().info('back_water_up_up')
            nest_state = States.back_water_up
        elif state == States.back_water_up:
            self.get_logger().info('back_water_up')
            if order[order_count][1]>0:
                nest_state = States.SUCK_SAUSE_LID_TO_WORK
            else:
                nest_state = States.SUCK_CUP_LID_TO_WORK
        elif state == States.SUCK_CUP_LID_TO_WORK:
            self.get_logger().info('SUCK_CUP_LID_TO_WORK')
            # res=self.robot_wait()
            res=self.jaw('open')
            # cup_lid_pose[2]+=200
            # res=self.move('P',cup_lid_pose,holding=False)
            # cup_lid_pose[2]-=200
            cup_lid_pose[2]+=50
            res=self.move('P',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=50
            cup_lid_pose[2]+=30
            res=self.move('P',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=30
            cup_lid_pose[2]+=20
            res=self.move('P',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=20
            cup_lid_pose[2]+=10
            res=self.move('P',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=10

            res=self.move('P',cup_lid_pose)
            res=self.vacuum_control(PUMP,'ON',holding=False,do_timer=20)
            # if water_count>1:
            #     t = threading.Thread(target = self.fillwater)
            #     t.start()
            #     water_count=1
            # else:
            #     pass
            res=self.jaw('close')
            # res=self.robot_wait()
            cup_lid_pose[2]+=80
            res=self.move('L',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=80
            cup_lid_pose[2]+=160
            res=self.move('P',cup_lid_pose,holding=False)
            cup_lid_pose[2]-=160
            cup_lid_work_pose[2]+=160
            res=self.move('P',cup_lid_work_pose,holding=False)
            cup_lid_work_pose[2]-=160
            cup_lid_work_pose[2]+=30
            res=self.move('P',cup_lid_work_pose,holding=False)
            cup_lid_work_pose[2]-=30
            res=self.move('L',cup_lid_work_pose,holding=False)
            # res=self.robot_wait()
            res=self.move('L',cup_push_pose1,holding=False,tool=8)
            res=self.move('L',cup_push_pose2,holding=False,tool=8)
            res=self.move('L',cup_push_pose3,holding=False,tool=8)
            res=self.move('L',cup_push_pose4,holding=True,tool=8)
            # res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            res=self.move('L',cup_push_pose5,holding=False,tool=8)
            res=self.move('L',cup_push_pose6,holding=True,tool=8)
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            res=self.move('L',cup_push_pose7,holding=True,tool=8)
            res=self.jaw('open')
            res=self.robot_wait()
            res=self.move('L',cup_push_pose8,holding=False,tool=8)
            res=self.jaw('close')
            res=self.robot_wait()
            cup_lid_work_pose[2]+=50
            res=self.move('L',cup_lid_work_pose,holding=False)
            cup_lid_work_pose[2]-=50
            cup_lid_work_pose[2]+=200
            res=self.move('P',cup_lid_work_pose,holding=False)
            cup_lid_work_pose[2]-=200
            nest_state = States.CUP_FINISH
        elif state == States.SUCK_SAUSE_LID_TO_WORK:
            self.get_logger().info('SUCK_SAUSE_LID_TO_WORK')
            # res=self.robot_wait()
            res=self.jaw('open')
            sause_lid_pose[2]+=50
            res=self.move('P',sause_lid_pose,holding=False)
            sause_lid_pose[2]-=50
            
            res=self.move('L',sause_lid_pose)
            res=self.vacuum_control(PUMP,'ON',holding=False,do_timer=20)
            # res=self.robot_wait()
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            sause_lid_pose[2]+=30
            res=self.move('L',sause_lid_pose,holding=False)
            sause_lid_pose[2]-=30
            sause_lid_pose[2]+=150
            res=self.move('P',sause_lid_pose,holding=False)
            sause_lid_pose[2]-=150
            sause_lid_work_pose[2]+=150
            res=self.move('P',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=150
            sause_lid_work_pose[2]+=50
            res=self.move('P',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=50
            res=self.move('L',sause_lid_work_pose)
            # res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            res=self.move('L',sause_push_pose)
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            sause_lid_work_pose[2]+=50
            res=self.move('L',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=50
            sause_lid_work_pose[2]+=150
            res=self.move('P',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=150
            nest_state = States.SAUSE_FINISH
        elif state == States.CUP_FINISH:
            self.get_logger().info('CUP_FINISH')
            if finish_order[order_count][1]==0:
                if finish_order[order_count][0]==0:
                    cupfinish[2]+=250
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=250
                    res=self.move('P',cupfinish)
                    # res=self.robot_wait()
                    res=self.jaw('open')
                    res=self.robot_wait()
                    # time.sleep(0.5)
                    cupfinish[2]+=100
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=100
                    # cup_finish_pose_1[2]+=100
                    # res=self.move('P',cup_finish_pose_1,holding=False)
                    # cup_finish_pose_1[2]-=100
                    # res=self.move('P',cup_finish_pose_1)
                    # res=self.robot_wait()
                    # res=self.jaw('open')
                    # res=self.robot_wait()
                    # time.sleep(0.5)
                elif finish_order[order_count][0]==1:
                    cupfinish[2]+=250
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=250
                    res=self.move('P',cupfinish)
                    # res=self.robot_wait()
                    res=self.jaw('open')
                    res=self.robot_wait()
                    # time.sleep(0.5)
                    cupfinish[2]+=100
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=100
                    # cup_finish_pose_2[2]+=100
                    # res=self.move('P',cup_finish_pose_2,holding=False)
                    # cup_finish_pose_2[2]-=100
                    # res=self.move('P',cup_finish_pose_2)
                    # res=self.robot_wait()
                    # res=self.jaw('open')
                    # res=self.robot_wait()
                    # time.sleep(0.5)
                elif finish_order[order_count][0]==2:
                    cupfinish[2]+=250
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=250
                    res=self.move('P',cupfinish)
                    # res=self.robot_wait()
                    res=self.jaw('open')
                    res=self.robot_wait()
                    # time.sleep(0.5)
                    cupfinish[2]+=100
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=100
                    # cup_finish_pose_3[2]+=100
                    # res=self.move('P',cup_finish_pose_3,holding=False)
                    # cup_finish_pose_3[2]-=100
                    # res=self.move('P',cup_finish_pose_3)
                    # res=self.robot_wait()
                    # res=self.jaw('open')
                    # res=self.robot_wait()
                    # time.sleep(0.5)
            elif finish_order[order_count][1]==1:
                if finish_order[order_count][0]==0:
                    cupfinish[2]+=250
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=250
                    res=self.move('P',cupfinish)
                    # res=self.robot_wait()
                    res=self.jaw('open')
                    res=self.robot_wait()
                    # time.sleep(0.5)
                    cupfinish[2]+=100
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=100
                    # cup_finish_pose_2[2]+=100
                    # res=self.move('P',cup_finish_pose_2,holding=False)
                    # cup_finish_pose_2[2]-=100
                    # res=self.move('P',cup_finish_pose_2)
                    # res=self.robot_wait()
                    # res=self.jaw('open')
                    # res=self.robot_wait()
                    # time.sleep(0.5)
                elif finish_order[order_count][0]==1:
                    cupfinish[2]+=250
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=250
                    res=self.move('P',cupfinish)
                    # res=self.robot_wait()
                    res=self.jaw('open')
                    res=self.robot_wait()
                    # time.sleep(0.5)
                    cupfinish[2]+=100
                    res=self.move('P',cupfinish,holding=False)
                    cupfinish[2]-=100
                    # cup_finish_pose_3[2]+=100
                    # res=self.move('P',cup_finish_pose_3,holding=False)
                    # cup_finish_pose_3[2]-=100
                    # res=self.move('P',cup_finish_pose_3)
                    # res=self.robot_wait()
                    # res=self.jaw('open')
                    # res=self.robot_wait()
                    # time.sleep(0.5)
            elif finish_order[order_count][1]==2:
                cupfinish[2]+=250
                res=self.move('P',cupfinish,holding=False)
                cupfinish[2]-=250
                res=self.move('P',cupfinish)
                # res=self.robot_wait()
                res=self.jaw('open')
                res=self.robot_wait()
                # time.sleep(0.5)
                cupfinish[2]+=100
                res=self.move('P',cupfinish,holding=False)
                cupfinish[2]-=100
                # cup_finish_pose_3[2]+=100
                # res=self.move('P',cup_finish_pose_3,holding=False)
                # cup_finish_pose_3[2]-=100
                # res=self.move('P',cup_finish_pose_3)
                # res=self.robot_wait()
                # res=self.jaw('open')
                # res=self.robot_wait()
                # time.sleep(0.5)
            # cup_pose[2]-=6.1
            # cup_lid_pose[2]-=3.7
            order[order_count][0]=order[order_count][0]-1
            finish_order[order_count][0]=finish_order[order_count][0]+1
            nest_state = States.gohome
        elif state == States.SAUSE_FINISH:
            self.get_logger().info('SAUSE_FINISH')
            if finish_order[order_count][1]==0:
                sausefinish[2]+=250
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=250
                sausefinish[2]+=100
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=100
                res=self.move('P',sausefinish)
                # res=self.robot_wait()
                res=self.jaw('open')
                res=self.robot_wait()
                # time.sleep(0.5)
                sausefinish[2]+=250
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=250
                # sause_finish_pose_1[2]+=100
                # res=self.move('P',sause_finish_pose_1,holding=False)
                # sause_finish_pose_1[2]-=100
                # res=self.move('P',sause_finish_pose_1)
                # res=self.robot_wait()
                # res=self.jaw('open')
                # res=self.robot_wait()
                # time.sleep(0.5)
            elif finish_order[order_count][1]==1:
                sausefinish[2]+=250
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=250
                res=self.move('P',sausefinish)
                # res=self.robot_wait()
                res=self.jaw('open')
                res=self.robot_wait()
                # time.sleep(0.5)
                sausefinish[2]+=100
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=100
                # sause_finish_pose_2[2]+=100
                # res=self.move('P',sause_finish_pose_2,holding=False)
                # sause_finish_pose_2[2]-=100
                # res=self.move('P',sause_finish_pose_2)
                # res=self.robot_wait()
                # res=self.jaw('open')
                # res=self.robot_wait()
                # time.sleep(0.5)
            elif finish_order[order_count][1]==2:
                sausefinish[2]+=250
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=250
                res=self.move('P',sausefinish)
                # res=self.robot_wait()
                res=self.jaw('open')
                res=self.robot_wait()
                # time.sleep(0.5)
                sausefinish[2]+=100
                res=self.move('P',sausefinish,holding=False)
                sausefinish[2]-=100
                # sause_finish_pose_3[2]+=100
                # res=self.move('P',sause_finish_pose_3,holding=False)
                # sause_finish_pose_3[2]-=100
                # res=self.move('P',sause_finish_pose_3)
                # res=self.robot_wait()
                # res=self.jaw('open')
                # res=self.robot_wait()
                # time.sleep(0.5)
            # sause_pose[2]-=8.025
            # sause_lid_pose[2]-=5.6
            order[order_count][1]=order[order_count][1]-1
            finish_order[order_count][1]=finish_order[order_count][1]+1
            nest_state = States.gohome
        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT')
            req = self.generate_robot_request(cmd_mode=RobotCommand.Request.CLOSE)
            res = self.call_hiwin(req)
            nest_state = States.FINISH

        else:
            nest_state = None
            self.get_logger().error('Input state not supported!')
            # return
        return nest_state

    def _main_loop(self):
        state = States.INIT
        while state != States.FINISH:
            state = self._state_machine(state)
            if state == None:
                break
        self.destroy_node()

    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        return True
    
    def generate_robot_request(
            self, 
            holding=True,
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            tool=1,
            base=30,
            do_timer=0,
            digital_output_pin=0,
            digital_input_pin=2,
            digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
            pose=Twist(),
            joints=[float('inf')]*6,
            circ_s=[],
            circ_end=[],
            jog_joint=6,
            jog_dir=0
            ):
        request = RobotCommand.Request()
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.digital_input_pin = digital_input_pin

        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.tool = tool
        request.base = base
        request.do_timer = do_timer
        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type
        request.circ_end = circ_end
        request.jog_dir = jog_dir
        request.holding = holding
        request.joints = joints
        request.circ_s = circ_s
        request.pose = pose
        return request

    def call_hiwin(self, req):
        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res

    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

def main(args=None):
    rclpy.init(args=args)

    stratery = ExampleStrategy()
    stratery.start_main_loop_thread()

    rclpy.spin(stratery)
    rclpy.shutdown()

if __name__ == "__main__":
    ordertest.getorder()
    order=ordertest.order
    finish_order=ordertest.finish_order
    get_order=ordertest.get_order
    cupmiss=[ordertest.cupx.get(),ordertest.cupy.get(),ordertest.cupz.get()]
    cuplidmiss=[ordertest.cuplidx.get(),ordertest.cuplidy.get(),ordertest.cuplidz.get()]
    sausemiss=[ordertest.sausex.get(),ordertest.sausey.get(),ordertest.sausez.get()]
    sauselidmiss=[ordertest.sauselidx.get(),ordertest.sauselidy.get(),ordertest.sauselidz.get()]
    cupworkmiss=[ordertest.cupworkx.get(),ordertest.cupworky.get(),ordertest.cupworkz.get()]
    cuplidworkmiss=[ordertest.cuplidworkx.get(),ordertest.cuplidworky.get(),ordertest.cuplidworkz.get()]
    sauseworkmiss=[ordertest.sauseworkx.get(),ordertest.sauseworky.get(),ordertest.sauseworkz.get()]
    sauselidworkmiss=[ordertest.sauselidworkx.get(),ordertest.sauselidworky.get(),ordertest.sauselidworkz.get()]
    for i in range(0,3):
        cup_pose[i]=cup_pose[i]+cupmiss[i]
        cup_lid_pose[i]=cup_lid_pose[i]+cuplidmiss[i]
        sause_pose[i]=sause_pose[i]+sausemiss[i]
        sause_lid_pose[i]=sause_lid_pose[i]+sauselidmiss[i]
        cup_work_pose[i]=cup_work_pose[i]+cupworkmiss[i]
        cup_lid_work_pose[i]=cup_lid_work_pose[i]+cuplidworkmiss[i]
        cup_push_pose1[i]=cup_push_pose1[i]+cuplidworkmiss[i]
        cup_push_pose2[i]=cup_push_pose2[i]+cuplidworkmiss[i]
        cup_push_pose3[i]=cup_push_pose3[i]+cuplidworkmiss[i]
        cup_push_pose4[i]=cup_push_pose4[i]+cuplidworkmiss[i]
        cup_push_pose5[i]=cup_push_pose5[i]+cuplidworkmiss[i]
        cup_push_pose6[i]=cup_push_pose6[i]+cuplidworkmiss[i]
        cup_push_pose7[i]=cup_push_pose7[i]+cuplidworkmiss[i]
        cup_push_pose8[i]=cup_push_pose8[i]+cuplidworkmiss[i]
        sause_work_pose[i]=sause_work_pose[i]+sauseworkmiss[i]
        sause_work_pose[i]=sause_work_pose[i]+sauseworkmiss[i]
        sause_lid_work_pose[i]=sause_lid_work_pose[i]+sauselidworkmiss[i]
    print(cup_pose,cup_lid_pose,sause_pose,sause_lid_pose)
    main()
    print(' ')
