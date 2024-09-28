#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import Robot
import numpy as np
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
from scipy.optimize import minimize
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

        self.timer = self.create_timer(10, self.mode3_callback)

        #publisher
        self.end_effector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.singularity_pub = self.create_publisher(String, "/singularity", 10)

        #subscription
        self.create_subscription(PoseStamped, "/target", self.randomtarget_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmdvel_callback, 10)

        #service server
        self.mode_server = self.create_service(Robot, '/mode', self.mode_callback)
        self.mode2ref_server = self.create_service(Robot, '/mode2', self.mode2_callback)

        #service client
        self.mode3_start = self.create_client(SetBool, "/mode3_start")
        self.mode3_finish = self.create_client(SetBool, '/mode3_finish')

        #variable
        self.mode = 1
        self.target = [0.0,0.0,0.0]
        self.q_target = [0.0, 0.0, 0.0]
        self.finish = False
        self.finish2 = True
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
    def custom_ikine(self, T_desired, initial_guess):
        # Define the objective function
        def objective(q):
            T_actual = self.robot.fkine(q)
            # return np.linalg.norm(T_actual.t - T_desired.t)
            return abs(T_actual.t[0] - T_desired.t[0]) + abs(T_actual.t[1] - T_desired.t[1]) + abs(T_actual.t[2] - T_desired.t[2])
        
        # Run the optimization
        result = minimize(objective, initial_guess, bounds=[(-pi, pi) for _ in initial_guess])
        
        return result

    def inverse_kinematic(self, target):
        T = SE3(target[0], target[1], target[2])
        initial_guess = [[pi/2,pi/2,pi/2], [-pi/2,pi/2,pi/2], [pi/2,-pi/2,pi/2], [pi/2,pi/2,-pi/2],
                         [pi/2,-pi/2,-pi/2], [-pi/2,pi/2,-pi/2], [-pi/2,-pi/2,pi/2], [-pi/2,-pi/2,-pi/2]]
        q_target = []
        finish = False
        for init_guess in initial_guess:
            q = self.custom_ikine(T, init_guess)
            if q.fun <= 0.001:
                q_target.append(q.x[0])
                q_target.append(q.x[1])
                q_target.append(q.x[2])
                finish = True
                break
        return (q_target, finish)

#============================================================Jacobian=======================================================
    def change_frame(self, q, v):
        r0_1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0],
                         [np.sin(q[0]),  np.cos(q[0]), 0],
                         [0,             0,            1]])
        r1_2 = np.array([[np.sin(q[1]),  np.cos(q[1]), 0],
                         [0,             0,            1],
                         [np.cos(q[1]), -np.sin(q[1]), 0]])
        r2_3 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0],
                         [np.sin(q[2]),  np.cos(q[2]), 0],
                         [0,             0,            1]])
        r3_e = np.array([[0, 0, 1],
                         [1, 0, 0],
                         [0, 1, 0]])
        r0_e = r0_1 @ r1_2 @ r2_3 @ r3_e
        return r0_e @ np.transpose(v)

    def jacobian(self, q, v):
        x1 = -0.28*((np.sin(q[0])*np.sin(q[1])*np.cos(q[2]))+(np.sin(q[0])*np.cos(q[1])*np.sin(q[2]))) - 0.25*np.sin(q[0])*np.sin(q[1]) + 0.02*np.cos(q[0])
        x2 = 0.28*((np.cos(q[0])*np.cos(q[1])*np.cos(q[2]))-(np.cos(q[0])*np.sin(q[1])*np.sin(q[2]))) + 0.25*np.cos(q[0])*np.cos(q[1])
        x3 = 0.28*((np.cos(q[0])*np.cos(q[1])*np.cos(q[2]))-(np.cos(q[0])*np.sin(q[1])*np.sin(q[2])))

        y1 = 0.28*((np.cos(q[0])*np.sin(q[1])*np.cos(q[2]))+(np.cos(q[0])*np.cos(q[1])*np.sin(q[2]))) + 0.25*np.cos(q[0])*np.sin(q[1]) + 0.02*np.sin(q[0])
        y2 = 0.28*((np.sin(q[0])*np.cos(q[1])*np.cos(q[2]))-(np.sin(q[0])*np.sin(q[1])*np.sin(q[2]))) + 0.25*np.sin(q[0])*np.cos(q[1])
        y3 = 0.28*((np.sin(q[0])*np.cos(q[1])*np.cos(q[2]))-(np.sin(q[0])*np.sin(q[1])*np.sin(q[2])))

        z1 = 0
        z2 = -0.28*((np.sin(q[1])*np.cos(q[2]))+(np.cos(q[1])*np.sin(q[2]))) - 0.25*np.sin(q[1])
        z3 = -0.28*((np.cos(q[1])*np.sin(q[2]))+(np.sin(q[1])*np.cos(q[2])))

        J = np.array([[x1, x2, x3], [y1, y2, y3], [z1, z2, z3]])

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
                self.timer.reset()
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
    def randomtarget_callback(self, msg):
        task = msg.pose.position
        self.finish = False
        if self.mode == 3:
            self.target[0] = task.x
            self.target[1] = task.y
            self.target[2] = task.z
            ikine_result = self.inverse_kinematic(self.target)
            self.finish = ikine_result[1]
            if self.finish:
                self.q_target = ikine_result[0]
                self.get_logger().info(f"Go to point x:{round(self.target[0],2)} y:{round(self.target[1],2)} z:{round(self.target[2],2)}")
                self.get_logger().info(f"With q1:{round(self.q_target[0],2)} q2:{round(self.q_target[1],2)} q3:{round(self.q_target[2],2)}")
            else:
                self.get_logger().info("Cannot find inverse kinematic")

    def mode3_callback(self):
        msg = SetBool.Request()
        if abs(self.q_target[0] - self.q[0]) + abs(self.q_target[1] - self.q[1]) + abs(self.q_target[2] - self.q[2]) < 0.005 and self.finish == True:
            msg.data = True
            self.mode3_finish.call_async(msg)
        elif self.mode == 3:
            self.get_logger().info("Cannot go to point in 10 sec")
            msg.data = False
            self.mode3_finish.call_async(msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = RobotcontrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
