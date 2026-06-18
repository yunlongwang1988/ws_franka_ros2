#!/usr/bin/env python3
"""Bridge sigma7 gripper angle to franka gripper actions."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from franka_msgs.action import Grasp, Move


class GripperBridge(Node):
    def __init__(self):
        super().__init__('gripper_bridge')
        self.declare_parameter('namespace', '/NS_1')
        self.declare_parameter('close_threshold', 0.05)
        self.declare_parameter('open_threshold', 0.12)

        ns = self.get_parameter('namespace').value
        self.close_thresh = self.get_parameter('close_threshold').value
        self.open_thresh = self.get_parameter('open_threshold').value

        self.grasp_client = ActionClient(
            self, Grasp, f'{ns}/franka_gripper/grasp')
        self.move_client = ActionClient(
            self, Move, f'{ns}/franka_gripper/move')

        self.state = 'open'  # open, closed, moving
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.create_subscription(
            Float32, '/sigma1/gripper_angle', self.cb, qos)
        self.get_logger().info(
            f'GripperBridge: close<{self.close_thresh} '
            f'open>{self.open_thresh} ns={ns}')

    def cb(self, msg):
        angle = msg.data
        if angle < self.close_thresh and self.state == 'open':
            self.state = 'moving'
            goal = Grasp.Goal()
            goal.width = 0.0
            goal.speed = 0.05
            goal.force = 50.0
            goal.epsilon.inner = 0.04
            goal.epsilon.outer = 0.04
            future = self.grasp_client.send_goal_async(goal)
            future.add_done_callback(self.grasp_response_cb)
            self.get_logger().info(f'Closing gripper (angle={angle:.3f})')

        elif angle > self.open_thresh and self.state == 'closed':
            self.state = 'moving'
            goal = Move.Goal()
            goal.width = 0.08
            goal.speed = 0.1
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.move_response_cb)
            self.get_logger().info(f'Opening gripper (angle={angle:.3f})')

    def grasp_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.set_state('closed'))
        else:
            self.state = 'open'

    def move_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.set_state('open'))
        else:
            self.state = 'closed'

    def set_state(self, s):
        self.state = s
        self.get_logger().info(f'Gripper now: {s}')


def main():
    rclpy.init()
    node = GripperBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
