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


order_count=0
water_count=1
DEFAULT_VELOCITY = 30
DEFAULT_ACCELERATION =30
LINE_VELOCITY = 100
LINE_ACCELERATION = 100
VACUUM_PIN1 = 1
VACUUM_PIN2 = 2
PUMP = 3
cup_push_pose = [388.923,196.452,121.251,179.794,0.594,90.307]
cup_work_pose = [388.923,196.452,122.475,179.794,0.594,90.307]
cup_lid_work_pose = [388.923,196.452,125.000,179.794,0.594,90.307]
sause_push_pose = [365.621,190.802,56.628,179.794,0.594,90.307]
sause_work_pose = [365.621,190.802,59.071,179.794,0.594,90.307]
sause_lid_work_pose = [365.621,190.802,64.000,179.794,0.594,90.307]
cup_lid_pose = [579.678,111.091,43.912,179.794,0.594,113.233]
sause_lid_pose = [412.810,226.325,49.292,179.795,0.597,7.005]
cup_pose = [545.992,277.335,176.289,179.794,0.594,90.307]
sause_pose = [340.987,360.974,127.070,179.794,0.594,90.307]
home_joint = [0.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint
home_pose = [314.951,216.448,414.945,179.794,0.594,90.307]
# OBJECT_POSE = [20.00, 0.00, 0.00, 0.00, -90.00, 0.00]
OBJECT_POSE = [-67.517, 361.753, 293.500, 180.00, 0.00, 100.572] #pose
PLACE_POSE = [-20.00, 0.00, 0.00, 0.00, -90.00, 0.00] #joint
water_pose=[55.766,209.323,163.00,179.794,0.594,90.307]
# water_up_pose=[277.54,199.47,178.13,180.00,0.00,90.00]
cup_work_up_pose=[356.652,189.788,105.152,179.794,0.594,90.307]
water_cup_going_pose=[356.652,189.788,105.152,-120.734,0.594,90.307]
sause_work_up_pose=[203.557,189.788,105.152,179.794,0.594,90.307]
water_sause_going_pose=[203.557,189.788,105.152,-120.734,0.594,90.307]
cup_finish_pose_1=[595.791,-466.855,140.359,179.794,0.594,90.307]
cup_finish_pose_2=[731.533,-466.855,140.359,179.794,0.594,90.307]
cup_finish_pose_3=[865.939,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_1=[724.815,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_2=[856.980,-466.855,140.359,179.794,0.594,90.307]
sause_finish_pose_3=[737.736,-466.855,140.359,180.00,0.00,-89.69]
# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times

NUM_OBJECTS = 5


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

    def move(self,PLJ,six_num,velocity=DEFAULT_VELOCITY,acceleration=DEFAULT_ACCELERATION,tool=1,base=3,holding=True,velocity_L=LINE_VELOCITY,acceleration_L=LINE_ACCELERATION):
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
            nest_state = States.gohome
        elif state == States.gohome:
            self.get_logger().info('gohome')
            res=self.move('J',home_joint,holding=False)
            res=self.move('J',home_joint,holding=True)
            home_joint[0]+=1
            res=self.move('J',home_joint,holding=True)
            home_joint[0]-=1
            if res.arm_state == RobotCommand.Response.IDLE:
                 nest_state = States.gohome
            else:
                nest_state = None
        

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
            base=3,
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

    # def call_yolo(self):
    #     class YoloResponse(NamedTuple):
    #         has_object: bool
    #         object_pose: list
    #     has_object = True if self.object_cnt < 5 else False
    #     object_pose = OBJECT_POSE
    #     res = YoloResponse(has_object,object_pose)
    #     # res.has_object = True if self.object_cnt < 5 else False
    #     # res.object_pose = OBJECT_POSE
    #     self.object_cnt += 1
    #     return res

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
    main()
