#!/usr/bin/env python3
"""Launch franka gripper node + sigma-to-franka gripper bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'),
        'config', 'franka_gripper_node.yaml')

    return LaunchDescription([
        Node(
            package='franka_gripper',
            executable='franka_gripper_node',
            name='franka_gripper',
            namespace='NS_1',
            parameters=[{
                'robot_ip': '172.16.0.2',
                'joint_names': ['fr3_finger_joint1', 'fr3_finger_joint2'],
            }, gripper_config],
        ),
        Node(
            package=None,
            executable=os.path.expanduser(
                '~/ws_yunlong/ws_franka_ros2/gripper_bridge.py'),
            name='gripper_bridge',
            parameters=[{
                'namespace': '/NS_1',
                'close_threshold': 0.05,
                'open_threshold': 0.12,
            }],
        ),
    ])
