#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
from robot_interfaces.srv import Robot


class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key_node')
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.velocity = Twist()

        self.changeref_client = self.create_client(Robot, "/mode2")
        self.speed = 3.0

        self.init_keyboard()


    def init_keyboard(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        print("Control Robot\n")
        print('moving around by:')
        print(' q     w     e')
        print(' a     s     d')
        print('q : move forward in x axis')
        print('a : move backward in x axis')
        print('w : move forward in y axis')
        print('s : move backward in y axis')
        print('e : move forward in z axis')
        print('d : move backward in z axis')
        print('x : stop')
        print('u : decrease speed')
        print('i : increase speed')
        print('o : change reference to base')
        print('p : change reference to end effector')
        print('z : exit')
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == 'q':
                    self.velocity.linear.x = self.speed
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0
                elif key == 'a':
                    self.velocity.linear.x = -self.speed
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0
                elif key == 'w':
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = self.speed
                    self.velocity.linear.z = 0.0
                elif key == 's':
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = -self.speed
                    self.velocity.linear.z = 0.0
                elif key == 'e':
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = self.speed
                elif key == 'd':
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = -self.speed
                elif key == 'x':
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0

                if key == 'u':
                    self.speed -= 0.5
                    if self.speed < 0:
                        self.speed = 0
                elif key == 'i':
                    self.speed += 0.5
                    
                
                if key in ['u', 'i']:
                    if self.velocity.linear.x > 0:
                        self.velocity.linear.x = self.speed
                    elif self.velocity.linear.x < 0:
                        self.velocity.linear.x = -self.speed
                    elif self.velocity.linear.y > 0:
                        self.velocity.linear.y = self.speed
                    elif self.velocity.linear.y < 0:
                        self.velocity.linear.y = -self.speed
                    elif self.velocity.linear.z > 0:
                        self.velocity.linear.z = self.speed
                    elif self.velocity.linear.z < 0:
                        self.velocity.linear.z = -self.speed

                if key == 'o':
                    req2 = Robot.Request()
                    req2.mode = 1
                    self.changeref_client.call_async(req2)
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0
                elif key == 'p':
                    req3 = Robot.Request()
                    req3.mode = 2
                    self.changeref_client.call_async(req3)
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0

                if key == 'z':
                    sys.exit()
                self.cmd_pub.publish(self.velocity)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)    

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
