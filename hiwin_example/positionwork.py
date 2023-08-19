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
cup_work_pose = [264.386, 376.426, 202.488, 179.373, 0.668, 89.551]
cup_work_up_pose=[356.652,189.788,105.152,179.794,0.594,90.307]
cup_lid_pose = [52.648, 219.663, 99.106, 179.372, 0.663, 89.557]
cup_lid_work_pose = [264.635, 376.659, 232.367, 179.373, 0.663, 89.547]

#cuppushtool8
# cup_push_pose1 = [265.791, 423.304, 51.264, 179.373, 0.668, 89.551]
# cup_push_pose2 = [265.791, 423.304, 51.264, 179.336, 5.373, 89.549]
# cup_push_pose3 = [265.791, 423.304, 41.522, 179.336, 5.373, 89.549]
# cup_push_pose4 =[265.791, 423.304, 40.429, 179.368, 1.32, 89.551]
# cup_push_pose5 = [265.791, 423.304, 40.429, 179.378, 0.023, 89.551]
# cup_push_pose6 = [267.635, 390.659, 232.367, 179.373, 0.663, 89.547]
cup_push_pose1 = [265.915, 429.589, 48.21, 179.378, 0.033, 89.551]
cup_push_pose2 = [265.915, 429.589, 48.21, 179.336, 5.335, 89.549]
cup_push_pose3 = [265.915, 429.589, 38.192, 179.336, 5.335, 89.549]
cup_push_pose4 = [265.915, 421.589, 38.192, 179.368, 5.335, 89.551]
cup_push_pose5 = [265.915, 421.589, 38.192, 179.378, 0.036, 89.551]
cup_push_pose6 = [265.915, 421.589, 34.398, 179.378, 0.036, 89.551]
cup_push_pose7 = [265.915, 421.589, 39.998, 179.378, 0.035, 89.551]
cup_push_pose8 = [265.915, 421.589, 33.234, 179.378, 0.035, 89.551]

sause_pose = [421.728, 181.31, 181.324, 179.372, 0.664, 89.557]
sause_work_pose = [264.386, 373.426, 168.511, 179.378, 0.035, 89.551]
sause_lid_pose = [230.97, 215.082, 115.753, 179.373, 0.666, 89.557]
sause_lid_work_pose = [264.386, 373.426, 140.117, 179.378, 0.035, 89.551]
sause_push_pose = [264.386, 376.426, 130.994, 179.378, 0.035, 89.551]

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
water_pose3 = [13.473, 281.922, 198.856, 179.374, 0.663, 89.547]
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
    def vacuum_control(self,number,OF,holding=True):
        self.num=number
        self.mode=OF
        self.hold=holding
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
                        holding=self.hold
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
            times=3
            # res=self.vacuum_control(VACUUM_PIN1,'ON')
            # res=self.vacuum_control(VACUUM_PIN1,'OFF')
            # res=self.vacuum_control(VACUUM_PIN2,'ON')
            # res=self.vacuum_control(VACUUM_PIN2,'OFF')
            # res=self.vacuum_control(PUMP,'OFF')
            nest_state = States.gohome
        elif state == States.gohome:
            self.get_logger().info('gohome')
            res=self.move('J',home_joint,holding=False)
            res=self.jaw('open')
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    res=self.jaw('close')
                    break
                else:
                    nest_state=States.gohome
                    return nest_state
            if times==1:
                nest_state = States.cupwork
            elif times==2:
                nest_state=States.cuplidwork
            elif times==3:
                nest_state=States.sausework
            elif times==4:
                nest_state=States.sauselidwork
            else:
                nest_state=States.gohome
        elif state==States.cupwork:
            self.get_logger().info('cupwork')
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    break
                else:
                    times=1
                    nest_state=States.gohome
                    return nest_state
            cup_work_pose[2]+=0
            res=self.move('P',cup_work_pose,holding=False)
            cup_work_pose[2]-=0
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    break
                else:
                    times=1
                    nest_state=States.gohome
                    return nest_state
            cup_work_pose[2]-=67
            res=self.move('P',cup_work_pose,holding=False)
            cup_work_pose[2]+=67
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    times=2
                    break
                else:
                    times=1
                    nest_state=States.gohome
                    return nest_state
            res=self.move('L',cup_work_pose,holding=False)
            res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            nest_state = States.gohome
        elif state==States.sausework:
            self.get_logger().info('sausework')
            
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    break
                else:
                    times=3
                    nest_state=States.gohome
                    return nest_state
            sause_work_pose[2]+=10
            res=self.move('P',sause_work_pose,holding=False)
            sause_work_pose[2]-=10
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    times=4
                    break
                else:
                    times=3
                    nest_state=States.gohome
                    return nest_state
            res=self.move('P',sause_work_pose,holding=False)
            res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            nest_state=States.gohome
        elif state == States.cuplidwork:
            self.get_logger().info('cuplidwork')
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    break
                else:
                    times=2
                    nest_state=States.gohome
                    return nest_state
            cup_lid_work_pose[2]+=30
            res=self.move('P',cup_lid_work_pose,holding=False)
            cup_lid_work_pose[2]-=30
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    times=3
                    break
                else:
                    times=2
                    nest_state=States.gohome
                    return nest_state
            res=self.move('P',cup_lid_work_pose,holding=False)
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
            res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            nest_state = States.gohome
        elif state == States.sauselidwork:
            self.get_logger().info('sauselidwork')
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    break
                else:
                    times=4
                    nest_state=States.gohome
                    return nest_state
            sause_lid_work_pose[2]+=20
            res=self.move('P',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=20
            while True:
                again = input("check Again?:")
                if again == 'y' or again == 'Y':
                    print("Contiune")
                    times=1
                    break
                else:
                    times=4
                    nest_state=States.gohome
                    return nest_state
            res=self.move('L',sause_lid_work_pose)
            res=self.jaw('open')
            res=self.robot_wait()
            res=self.move('L',sause_push_pose)
            # res=self.robot_wait()
            res=self.jaw('close')
            res=self.robot_wait()
            sause_lid_work_pose[2]+=50
            res=self.move('L',sause_lid_work_pose,holding=False)
            sause_lid_work_pose[2]-=50
            res=self.robot_wait()
            res=self.jaw('open')
            res=self.robot_wait()
            nest_state=States.gohome
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
            digital_output_pin=0,
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
        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.tool = tool
        request.base = base
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
