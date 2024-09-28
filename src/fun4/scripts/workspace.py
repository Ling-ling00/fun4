import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from math import pi
import matplotlib.pyplot as plt

# all robot
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.2),
        rtb.RevoluteMDH(alpha = -pi/2, d = -0.12, offset = -pi/2),
        rtb.RevoluteMDH(a = 0.25, d = 0.1),
    ],
    tool = SE3.Tx(0.28) @ SE3.Rz(pi/2) @ SE3.Rx(pi/2),
    name = "RRR_Robot"
)

# 2 upper RR joint
robot2 = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = -pi/2, d = -0.12, offset = -pi/2),
        rtb.RevoluteMDH(a = 0.25, d = 0.1),
    ],
    tool = SE3.Tx(0.28) @ SE3.Rz(pi/2) @ SE3.Rx(pi/2),
    name = "RR_Robot"
)

# plot home config
print(robot)
robot.plot([0,0,0])

#plot workspace
theta = np.linspace(-pi, pi, 30)
theta2 = np.linspace(-pi, pi, 50)
x = []
y = []
z = []

x2 = []
z2 = []

for q1 in theta:
    for q2 in theta:
        for q3 in theta:
            T_sol = robot.fkine([q1,q2,q3])
            x.append(T_sol.x)
            y.append(T_sol.y)
            z.append(T_sol.z)

for q1 in theta2:
    for q2 in theta2:
        T_sol = robot2.fkine([q1,q2])
        x2.append(T_sol.x)
        z2.append(T_sol.z)

fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.scatter(x, y, z, c = 'b', marker='.')
ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')
ax.set_zlabel('Z-axis (m)')
ax.set_title('Workspace of Robot')

ax2 = fig.add_subplot(122)
ax2.scatter(x2,z2, c='r', marker='.')
ax2.set_xlabel('X-axis (m)')
ax2.set_ylabel('Z-axis (m)')
ax2.set_title('Workspace of 2 upper RR joint')

circle1  = plt.Circle((0,0), 0.53, color = 'g', fill = False, linewidth = 3)
circle2  = plt.Circle((0,0), 0.03, color = 'g', fill = False, linewidth = 3)
ax2.add_patch(circle1)
ax2.add_patch(circle2)
plt.show()
input('enter to close')