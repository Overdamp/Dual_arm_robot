#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from copy import deepcopy


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        self.left_msg = None
        self.right_msg = None

        self.left_sub = self.create_subscription(
            JointState,
            '/left/joint_states',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            JointState,
            '/right/joint_states',
            self.right_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publish merged message at fixed rate
        self.timer = self.create_timer(0.02, self.publish_merged_joint_states)  # 50 Hz

    def left_callback(self, msg):
        self.left_msg = msg

    def right_callback(self, msg):
        self.right_msg = msg

    def publish_merged_joint_states(self):
        if self.left_msg is None or self.right_msg is None:
            return

        merged_msg = JointState()
        merged_msg.header.stamp = self.get_clock().now().to_msg()

        # Merge name, position, velocity, and effort fields
        merged_msg.name = self.left_msg.name + self.right_msg.name
        merged_msg.position = self.left_msg.position + self.right_msg.position
        merged_msg.velocity = self.left_msg.velocity + self.right_msg.velocity
        merged_msg.effort = self.left_msg.effort + self.right_msg.effort

        self.publisher.publish(merged_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
