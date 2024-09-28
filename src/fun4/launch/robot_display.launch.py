#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
    
def generate_launch_description():
    
    pkg = get_package_share_directory('fun4')
    rviz_path = os.path.join(pkg,'config','display2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    Robot_controller = Node(package = 'fun4',
                            executable = 'robotcontrol_script.py',
                            name = 'robot_control',
    )

    Target_random = Node(package = 'fun4',
                         executable = 'targetrandom_script.py',
                         name = 'target_random',
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(Robot_controller)
    launch_description.add_action(Target_random)
    
    return launch_description