#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Robot
import numpy as np
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String


class RobotcontrolNode(Node):
    def __init__(self):
        super().__init__('robotcontrol_node')

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
        self.end_effector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.singularity_pub = self.create_publisher(String, "/singularity", 10)

        #subscription
        self.create_subscription(Twist, "/cmd_vel", self.cmdvel_callback, 10)

        #service server
        self.mode_server = self.create_service(Robot, '/mode', self.mode_callback)
        self.mode2ref_server = self.create_service(Robot, '/mode2', self.mode2_callback)
        self.mode3_finish = self.create_service(Robot, "/mode3_finish", self.mode3_callback)

        #service client
        self.mode3_start = self.create_client(SetBool, "/mode3_start")

        #variable
        self.mode = 1
        self.target = [0.0,0.0,0.0]
        self.q_target = [0.0, 0.0, 0.0]
        self.finish = False # for check is IK possible
        self.finish2 = True #for check singularity
        self.mode2_ref = 1 #1 is base 2 is end-effector
        self.v = [0, 0, 0]

        #for pub joint to model
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        self.q = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]

#=========================================================inverse kinematic===================================================
    def inverse_kinematic(self, target):
        T = SE3(target[0], target[1], target[2])
        q_target = self.robot.ik_LM(T, mask = [1,1,1,0,0,0])
        return (q_target[0], q_target[1])

#============================================================Jacobian=======================================================
    def change_frame(self, q, v):
        r0_e = self.robot.fkine(q).R
        return r0_e @ np.transpose(v)

    def jacobian(self, q, v):
        J = self.robot.jacob0(q)[:3]
        if np.linalg.det(J) != 0:
            J_inv= np.linalg.inv(J)
            dq_dt = J_inv @ np.transpose(v)
        else:
            dq_dt = [0, 0, 0]
        return dq_dt
    
#=========================================================simulation loop===================================================
    def sim_loop(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        q_d = [0.0,0.0,0.0]
        if self.mode == 1 or self.mode == 3:
            q_d[0] = self.q_target[0] - self.q[0]
            q_d[1] = self.q_target[1] - self.q[1]
            q_d[2] = self.q_target[2] - self.q[2]
        elif self.mode == 2:
            if abs(self.q[2] - ((self.q[2]//(2*pi))*2*pi)) > 0.1:
                self.finish2 = True
                dq_dt = self.calculate_joint_velocity()
                q_d[0] = dq_dt[0] * self.dt
                q_d[1] = dq_dt[1] * self.dt
                q_d[2] = dq_dt[2] * self.dt
            else:
                if self.finish2:
                    msg2 = String()
                    msg2.data = 'Singularity aleart!!!!'
                    self.singularity_pub.publish(msg2)
                    self.get_logger().info('Singularity aleart!!!!')
                    self.get_logger().info('Please change mode or reset')
                    self.finish2 = False
                self.v = [0.0, 0.0, 0.0]
                q_d = [0.0, 0.0, 0.0]
        else:
            q_d = [0.0, 0.0, 0.0]
        self.q[0] = self.q[0] + (q_d[0] * self.dt)
        self.q[1] = self.q[1] + (q_d[1] * self.dt)
        self.q[2] = self.q[2] + (q_d[2] * self.dt)
        for i in range(len(self.q)):
            if self.q[i] > pi:
                self.q[i] -= 2*pi
            elif self.q[i] < -pi:
                self.q[i] += 2*pi
            msg.position.append(self.q[i])
            msg.name.append(self.name[i])
        self.joint_pub.publish(msg)
        self.ee_publisher()

#=========================================================change mode===================================================
    def mode_callback(self, request:Robot.Request, response:Robot.Response):
        self.mode = request.mode
        if self.mode in [1,2,3]:
            self.get_logger().info(f"Change mode to {self.mode}")
            response.mode = True
        else:
            self.get_logger().info("Please enter mode between 1, 2, 3")
            self.get_logger().info("1 is Inverse Pose Kinematics")
            self.get_logger().info("2 is Teleoperation")
            self.get_logger().info("3 is Auto")
            response.mode = False

        if self.mode == 1:
            self.target[0] = request.taskspace.x
            self.target[1] = request.taskspace.y
            self.target[2] = request.taskspace.z
            self.target_publisher()
            ikine_result = self.inverse_kinematic(self.target)
            if ikine_result[1]:
                response.start = True
                response.q1 = ikine_result[0][0]
                response.q2 = ikine_result[0][1]
                response.q3 = ikine_result[0][2]
                self.q_target = ikine_result[0]
                self.get_logger().info(f"Go to point x:{round(self.target[0],2)} y:{round(self.target[1],2)} z:{round(self.target[2],2)}")
                self.get_logger().info(f"With q1:{round(self.q_target[0],2)} q2:{round(self.q_target[1],2)} q3:{round(self.q_target[2],2)}")
            else:
                self.get_logger().info("Cannot find inverse kinematic")
                response.start = False

        if self.mode == 2:
            response.q1 = self.q[0]
            response.q2 = self.q[1]
            response.q3 = self.q[2]
            response.start = True

        if self.mode == 3:
            response.q1 = self.q[0]
            response.q2 = self.q[1]
            response.q3 = self.q[2]
            try:
                msg = SetBool.Request()
                msg.data = True
                self.mode3_start.call_async(msg)
                response.start = True
            except:
                response.start = False
        else:
            msg = SetBool.Request()
            msg.data = False
            self.mode3_start.call_async(msg)

        return response
    
#=========================================================mode2===================================================
    def mode2_callback(self, request, response):
        self.mode2_ref = request.mode
        if request.mode == 1:
            self.get_logger().info('change reference to base')
        elif request.mode == 2:
            self.get_logger().info('change reference to end effector')
        return response
    
    def cmdvel_callback(self, msg):
        if self.mode == 2:
            self.v[0] = msg.linear.x
            self.v[1] = msg.linear.y
            self.v[2] = msg.linear.z

    def calculate_joint_velocity(self):
        if self.mode2_ref == 2:
            self.v = self.change_frame(self.q, self.v)
        return self.jacobian(self.q, self.v)

#=========================================================mode3===================================================
    def mode3_callback(self, request:Robot.Request, response:Robot.Response):
        task = request.taskspace
        self.finish = False
        if self.mode == 3:
            time = (self.get_clock().now().nanoseconds)/10**9
            time2 = (self.get_clock().now().nanoseconds)/10**9
            time3 = (self.get_clock().now().nanoseconds)/10**9
            self.target[0] = task.x
            self.target[1] = task.y
            self.target[2] = task.z
            self.target_publisher()
            ikine_result = self.inverse_kinematic(self.target)
            self.finish = ikine_result[1]
            if self.finish:
                self.q_target = ikine_result[0]
                self.get_logger().info(f"Go to point x:{round(self.target[0],2)} y:{round(self.target[1],2)} z:{round(self.target[2],2)}")
                self.get_logger().info(f"With q1:{round(self.q_target[0],2)} q2:{round(self.q_target[1],2)} q3:{round(self.q_target[2],2)}")
            else:
                self.get_logger().info("Cannot find inverse kinematic")
                self.get_logger().info('Please change mode or reset')
        if self.finish == True:
            while (np.linalg.norm(self.q_target - self.q) > 0.005) and (time2 - time < 10):
                time2 = (self.get_clock().now().nanoseconds)/10**9
                if time2 - time3 > 0.01:
                    time3 = time2
                    self.sim_loop()
            if np.linalg.norm(self.q_target - self.q) < 0.005:
                response.start = True
                return response
            else:
                self.get_logger().info("Cannot go to point in 10 sec")
                self.get_logger().info('Please change mode or reset')
        response.start = False
        return response

#=========================================================publish to Rviz===================================================
    def ee_publisher(self):
        T = self.robot.fkine(self.q)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "link_0"
        pose.pose.position.x = T.t[0]
        pose.pose.position.y = T.t[1]
        pose.pose.position.z = T.t[2]
        q = quaternion_from_euler(T.eul()[0], T.eul()[1], T.eul()[2], axes='rzyz')
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.end_effector_pub.publish(pose)

    def target_publisher(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "link_0"
        pose.pose.position.x = self.target[0]
        pose.pose.position.y = self.target[1]
        pose.pose.position.z = self.target[2]
        self.target_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = RobotcontrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
