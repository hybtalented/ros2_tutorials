'''
Author Youbiao He hybtalented@163.com
Date 2022-07-08
LastEditors Youbiao He
LastEditTime 2022-07-15
FilePath /src/beginner_tutorials/launch/action_launch.py
Description 

Example 
'''
from struct import pack
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 创建一个组建的宿主节点
    container = ComposableNodeContainer(
        name="fibonacci",
        namespace="action_test",
        package='rclcpp_components',
        executable="component_container",
        composable_node_descriptions=[
            # 创建组件节点实例
            ComposableNode(name="fibonacci_server",
                           package="beginner_tutorials",
                           plugin="FibonacciActionServer"),
            ComposableNode(package="beginner_tutorials",
                           name="fibonacci_client", plugin="FibonacciActionClient")
        ],
        output='screen')
    return launch.LaunchDescription([container])
