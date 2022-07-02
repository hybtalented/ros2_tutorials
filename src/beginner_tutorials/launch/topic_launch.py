'''
Author Youbiao He hybtalented@163.com
Date 2022-07-02
LastEditors Youbiao He
LastEditTime 2022-07-02
FilePath /src/beginner_tutorials/launch/topic_launch.py
Description

Example
'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="beginner_tutorials",
             executable="talker",
             name="talker_node",
             output="screen",
             emulate_tty=True,
             parameters=[{
                 "target": "hybtalented"
             }]
             ),
        Node(package="beginner_tutorials",
             executable="listener",
             name="listener_node",
             output="screen",
             emulate_tty=True,
             )
    ])
