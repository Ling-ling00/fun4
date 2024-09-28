#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

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

        #publisher
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)

        #service server
        self.mode3_start = self.create_service(SetBool, '/mode3_start', self.mode3_start_callback)
        self.mode3_finish = self.create_service(SetBool, "/mode3_finish", self.mode3_finish_callback)

        #variable
        self.target = [0.0, 0.0, 0.0]
        self.isEnable = False

    def target_publisher(self):
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
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "link_0"
        pose.pose.position.x = T.x
        pose.pose.position.y = T.y
        pose.pose.position.z = T.z
        self.target[0] = T.x
        self.target[1] = T.y
        self.target[2] = T.z
        self.target_pub.publish(pose)

    def mode3_start_callback(self, request:SetBool.Request, response:SetBool.Response):
        if request.data:
            self.isEnable = True
            self.target_publisher()
            self.get_logger().info('Start random target')
        elif self.isEnable:
            self.get_logger().info('Stop random target')
            self.isEnable = False
        return response
    
    def mode3_finish_callback(self, request:SetBool.Request, response:SetBool.Response):
        if request.data and self.isEnable:
            self.target_publisher()
        elif self.isEnable:
            self.isEnable = False
            self.get_logger().info('Stop random target')
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = TargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
