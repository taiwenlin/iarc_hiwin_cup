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


# 點餐界面

order_count=0
water_count=1
DEFAULT_VELOCITY = 20
DEFAULT_ACCELERATION =20
LINE_VELOCITY = 100
LINE_ACCELERATION = 100
VACUUM_PIN1 = 1
VACUUM_PIN2 = 2
PUMP = 3
cup_pose = [427.14, 361.8, 210.656, 179.372, 0.664, 89.557]
cup_work_pose = [262.097, 379.033, 182.488, 179.373, 0.668, 89.551]
cup_work_up_pose=[356.652,189.788,105.152,179.794,0.594,90.307]
cup_lid_pose = [50.268, 214.936, 99.106, 179.372, 0.663, 89.557]
cup_lid_work_pose = [262.097, 379.033, 232.367, 179.373, 0.663, 89.547]

#cuppushtool8
# cup_push_pose1 = [265.791, 423.304, 51.264, 179.373, 0.668, 89.551]
# cup_push_pose2 = [265.791, 423.304, 51.264, 179.336, 5.373, 89.549]
# cup_push_pose3 = [265.791, 423.304, 41.522, 179.336, 5.373, 89.549]
# cup_push_pose4 =[265.791, 423.304, 40.429, 179.368, 1.32, 89.551]
# cup_push_pose5 = [265.791, 423.304, 40.429, 179.378, 0.023, 89.551]
# cup_push_pose6 = [267.635, 390.659, 232.367, 179.373, 0.663, 89.547]
cup_push_pose1 = [263.509, 431.921, 48.21, 179.378, 0.033, 89.551]
cup_push_pose2 = [263.509, 431.921, 48.21, 179.336, 5.335, 89.549]
cup_push_pose3 = [263.509, 431.921, 39.192, 179.336, 5.335, 89.549]
cup_push_pose4 = [263.509, 422.921, 39.192, 179.368, 5.335, 89.551]
cup_push_pose5 = [263.509, 422.921, 39.192, 179.378, -0.5, 89.551]
cup_push_pose6 = [263.509, 422.921, 35.398, 179.378, -0.5, 89.551]
cup_push_pose7 = [263.509, 422.921, 40.998, 179.378, -0.5, 89.551]
cup_push_pose8 = [263.509, 422.921, 34.234, 179.378, -0.5, 89.551]

sause_pose = [417.963, 177.340, 181.324, 179.372, 0.664, 89.557]
sause_work_pose = [262.097, 379.033, 148.511, 179.378, 0.035, 89.551]
sause_lid_pose = [227.843, 210.694, 115.753, 179.373, 0.666, 89.557]
sause_lid_work_pose = [262.097, 379.033, 140.117, 179.378, 0.035, 89.551]
sause_push_pose = [262.097, 379.033, 130.994, 179.378, 0.035, 89.551]

home_joint = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint
home_pose = [314.951,216.448,414.945,179.794,0.594,90.307]
# OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572] #pose
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint

# water_up_pose=[277.54,199.47,178.13,180.00,0.00,90.00]
sause_work_up_pose=[203.557,189.788,105.152,179.794,0.594,90.307]
# water_pose1=[263.977, 378.515, 249.824, 179.373, 0.664, 89.548]
water_pose1=[241.689, 196.261, 272.64, 179.373, 0.664, 89.548]
water_pose2=[43.476, 210.261, 198.852, 179.373, 0.664, 89.548]
water_pose3 = [11.788, 279.536, 198.856, 179.374, 0.663, 89.547]
water_cup_going_pose=[356.652,189.788,105.152,-120.734,0.594,90.307]
water_sause_going_pose=[203.557,189.788,105.152,-120.734,0.594,90.307]

cupfinish = [624.337, 316.785, 170.025, 179.373, 0.664, 89.548]
sausefinish = [639.569, 318.52, 112.867, 179.372, 0.662, 89.547]

cup_finish_pose_1=[595.791,-466.855,172.666,179.794,0.594,90.307]
cup_finish_pose_2=[731.533,-466.855,172.666,179.794,0.594,90.307]
cup_finish_pose_3=[865.939,-466.855,172.666,179.794,0.594,90.307]
sause_finish_pose_1=[595.791,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_2=[731.533,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_3=[865.939,-466.855,140.359,179.794,0.594,90.307]
# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times

NUM_OBJECTS = 5


class States(Enum):
    INIT = 0
    FINISH = 1
    gohome = 2
    cupwork = 3
    sausework = 4
    cuplidwork = 5
    sauselidwork = 6
    cupworkcatch = 7
    cuplidworkcatch = 8
    sauseworkcatch = 9
    sauselidworkcatch = 10
    CLOSE_ROBOT = 11
    WATERCHECK = 12
    CUP_WATER_1 = 13
    SAUSE_WATER_1 = 14
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
        if(self.mode=='P'):
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
        elif(self.mode=='L'):
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
        elif(self.mode=='J'):
            req = self.generate_robot_request(
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=self.six_number
                ,velocity=self.v,acceleration=self.a,
                tool=self.toolnum,
                base=self.basenum,
                holding=self.hold
                )
            res = self.call_hiwin(req)
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
    # btn_order=tk.Button(ordertest.mainwindow,text='send order',bg='yellow',font=('標楷體',40), command=ordertest.mainwindow.destroy)
    # btn_order.place(x=850,y=900)
    
    def _state_machine(self, state: States) -> States:
        global order_count
        global water_count
        global times
        if state == States.INIT:
            self.get_logger().info('INIT')
            times=1
            # res=self.vacuum_control(VACUUM_PIN1,'ON')
            # res=self.vacuum_control(VACUUM_PIN1,'OFF')
            # res=self.vacuum_control(VACUUM_PIN2,'ON')
            # res=self.vacuum_control(VACUUM_PIN2,'OFF')
            # res=self.vacuum_control(PUMP,'OFF')
            nest_state = States.gohome
        elif state == States.gohome:
            self.get_logger().info('gohome')
            res=self.move('J',home_joint,holding=False)
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    res=self.jaw('close')
                    break
                else:
                    nest_state=States.gohome
                    return nest_state
            nest_state=States.WATERCHECK
        elif state==States.WATERCHECK:
            self.get_logger().info('WATERCHECK')
            res=self.move('P',water_pose1,holding=False)
            res=self.move('P',water_pose2,holding=False)
            res=self.move('P',water_pose3,holding=False)
            while True:
                again = input("check Again?:")
                if again == 'C' or again == 'c':
                    print("Contiune")
                    nest_state=States.CUP_WATER_1
                    return nest_state
                elif again == 's' or again == 'S':
                    nest_state=States.SAUSE_WATER_1
                    return nest_state
                elif again == 'Y' or again =='y':
                    continue
                elif again == 'n' or again=='N':
                    res=self.move('P',water_pose2,holding=False)
                    res=self.move('P',water_pose1,holding=False)
                    nest_state = States.gohome
                    return nest_state
                elif again == 'F' or again=='f':
                    res=self.vacuum_control(PUMP,'ON',holding=False,do_timer=22)
                    continue
                else:
                    continue
        elif state == States.CUP_WATER_1:
            self.get_logger().info('CUP_WATER_1')
            water_pose3[0]+=80
            water_pose3[2]+=155
            res=self.move('L',water_pose3,holding=True)
            water_pose3[2]-=155
            water_pose3[0]-=80
            time.sleep(0.352)
            res=self.move('L',water_pose3,holding=False)
            res=self.move('P',water_pose2,holding=False)
            res=self.move('P',water_pose1,holding=False)
            nest_state = States.gohome
        elif state == States.SAUSE_WATER_1:
            self.get_logger().info('SAUSE_WATER_1')
            water_pose3[0]+=25
            water_pose3[2]+=60
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[2]-=60
            water_pose3[0]-=25
            water_pose3[0]+=50
            water_pose3[2]+=120
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=50
            water_pose3[2]-=120
            water_pose3[2]+=140
            water_pose3[0]+=70
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=70
            water_pose3[2]-=140
            water_pose3[0]+=80
            water_pose3[2]+=155
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            water_pose3[0]-=80
            water_pose3[2]-=155
            # time.sleep(0.15)
            res=self.move('L',water_pose3)
            # res=self.robot_wait()
            res=self.move('P',water_pose2,holding=False)
            res=self.move('P',water_pose1,holding=False)
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
        sause_work_pose[i]=sause_work_pose[i]+sauseworkmiss[i]
        sause_lid_work_pose[i]=sause_lid_work_pose[i]+sauselidworkmiss[i]
    main()
