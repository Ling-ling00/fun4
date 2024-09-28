#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi
from std_srvs.srv import SetBool
from robot_interfaces.srv import Robot

class TargetNode(Node):
    def __init__(self):
        super().__init__('target_node')
        self.robot = rtb.DHRobot(
        [
            rtb.RevoluteMDH(d=0.2),
            rtb.RevoluteMDH(alpha = -pi/2, d = -0.12, offset = -pi/2),
            rtb.RevoluteMDH(a = 0.25, d = 0.1),
        ],
        tool = SE3.Tx(0.28) @ SE3.Rz(pi/2) @ SE3.Rx(pi/2),
        name = "RRR_Robot"
        )

        self.create_timer(0.01, self.mode3_callback)

        #service server
        self.mode3_start = self.create_service(SetBool, '/mode3_start', self.mode3_start_callback)
        self.mode3_finish = self.create_client(Robot, '/mode3_finish')

        #variable
        self.target = [0.0, 0.0, 0.0]
        self.isEnable = False
        self.future_check = False

    def target_random(self):
        t1 = np.random.uniform(-pi,pi)
        t2 = np.random.uniform(-pi,pi)
        t3 = np.random.uniform(-pi,pi)
        while t1 in [-pi,pi]:
            t1 = np.random.uniform(-pi,pi)
        while t2 in [-pi,pi]:
            t2 = np.random.uniform(-pi,pi)
        while t3 in [-pi,pi]:
            t3 = np.random.uniform(-pi,pi)
        T = self.robot.fkine([t1,t2,t3])
        self.target[0] = T.x
        self.target[1] = T.y
        self.target[2] = T.z

    def mode3_start_callback(self, request:SetBool.Request, response:SetBool.Response):
        if request.data:
            self.isEnable = True
            self.get_logger().info('Start random target')
        elif self.isEnable:
            self.get_logger().info('Stop random target')
            self.isEnable = False
        return response
    
    def mode3_callback(self):
        if self.isEnable and self.future_check == False:
            msg = Robot.Request()
            self.target_random()
            msg.taskspace.x = self.target[0]
            msg.taskspace.y = self.target[1]
            msg.taskspace.z = self.target[2]
            self.future = self.mode3_finish.call_async(msg)
            self.future_check = True
        if self.future_check:
            if self.future.result() != None:
                if self.future.result().start == False:
                    self.isEnable = False
                    self.get_logger().info('Stop random target')
                self.future_check = False


def main(args=None):
    rclpy.init(args=args)
    node = TargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
