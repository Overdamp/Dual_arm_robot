#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
import time
from rclpy.time import Time
from quaternion_utils import quaternion_multiply, quaternion_inverse

class VRToServoNode(Node):
    def __init__(self):
        super().__init__('vr_to_servo_node')

        # Declare parameters
        self.declare_parameter('publish_period', 0.004)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 0.5)
        self.declare_parameter('right_frame_id', 'right_tool0')
        self.declare_parameter('left_frame_id', 'left_tool0')
        self.declare_parameter('right_joint_names', [
            "right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint",
            "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"])
        self.declare_parameter('left_joint_names', [
            "left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint",
            "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"])

        # Get parameters
        self.publish_period = self.get_parameter('publish_period').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.right_frame_id = self.get_parameter('right_frame_id').value
        self.left_frame_id = self.get_parameter('left_frame_id').value
        self.right_joint_names = self.get_parameter('right_joint_names').value
        self.left_joint_names = self.get_parameter('left_joint_names').value

        # Subscribers for right controller
        self.right_pose_sub = self.create_subscription(
            Pose, '/vr_controller/right/pose', self.right_pose_callback, 10)
        self.right_move_enable_sub = self.create_subscription(
            Bool, '/vr_controller/right/move_enable', self.right_move_enable_callback, 10)
        self.right_joint_state_sub = self.create_subscription(
            JointState, '/right_joint_states', self.right_joint_state_callback, 10)

        # Subscribers for left controller
        self.left_pose_sub = self.create_subscription(
            Pose, '/vr_controller/left/pose', self.left_pose_callback, 10)
        self.left_move_enable_sub = self.create_subscription(
            Bool, '/vr_controller/left/move_enable', self.left_move_enable_callback, 10)
        self.left_joint_state_sub = self.create_subscription(
            JointState, '/left_joint_states', self.left_joint_state_callback, 10)

        # Publishers for right arm
        self.right_servo_twist_pub = self.create_publisher(
            TwistStamped, '/right_servo_node/delta_twist_cmds', 10)
        self.right_servo_joint_pub = self.create_publisher(
            JointState, '/right_servo_node/delta_joint_cmds', 10)

        # Publishers for left arm
        self.left_servo_twist_pub = self.create_publisher(
            TwistStamped, '/left_servo_node/delta_twist_cmds', 10)
        self.left_servo_joint_pub = self.create_publisher(
            JointState, '/left_servo_node/delta_joint_cmds', 10)

        # State variables for right arm
        self.right_prev_position = None
        self.right_prev_orientation = None
        self.right_prev_time = None
        self.right_move_enable = False
        self.right_current_joint_state = None
        self.right_last_move_enable_time = 0.0
        self.right_prev_linear_velocity = np.zeros(3)
        self.right_prev_angular_velocity = np.zeros(3)

        # State variables for left arm
        self.left_prev_position = None
        self.left_prev_orientation = None
        self.left_prev_time = None
        self.left_move_enable = False
        self.left_current_joint_state = None
        self.left_last_move_enable_time = 0.0
        self.left_prev_linear_velocity = np.zeros(3)
        self.left_prev_angular_velocity = np.zeros(3)

        self.filter_alpha = 0.3

        self.get_logger().info('VRToServoNode initialized for dual arms')

    def right_joint_state_callback(self, msg: JointState):
        self.right_current_joint_state = msg
        self.get_logger().debug(f'Received right joint state: {msg.name}')

    def left_joint_state_callback(self, msg: JointState):
        self.left_current_joint_state = msg
        self.get_logger().debug(f'Received left joint state: {msg.name}')

    def right_move_enable_callback(self, msg: Bool):
        current_time = time.time()
        if current_time - self.right_last_move_enable_time < 0.5:
            return
        self.right_move_enable = msg.data
        self.right_last_move_enable_time = current_time
        if not self.right_move_enable:
            self.right_prev_position = None
            self.right_prev_orientation = None
            self.right_prev_time = None
            self.get_logger().info('Right arm movement disabled')
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.right_frame_id
            self.right_servo_twist_pub.publish(twist_msg)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.right_joint_names
            joint_msg.velocity = [0.0] * len(self.right_joint_names)
            self.right_servo_joint_pub.publish(joint_msg)
        else:
            self.get_logger().info('Right arm movement enabled')

    def left_move_enable_callback(self, msg: Bool):
        current_time = time.time()
        if current_time - self.left_last_move_enable_time < 0.5:
            return
        self.left_move_enable = msg.data
        self.left_last_move_enable_time = current_time
        if not self.left_move_enable:
            self.left_prev_position = None
            self.left_prev_orientation = None
            self.left_prev_time = None
            self.get_logger().info('Left arm movement disabled')
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.left_frame_id
            self.left_servo_twist_pub.publish(twist_msg)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.left_joint_names
            joint_msg.velocity = [0.0] * len(self.left_joint_names)
            self.left_servo_joint_pub.publish(joint_msg)
        else:
            self.get_logger().info('Left arm movement enabled')

    def right_pose_callback(self, msg: Pose):
        if not self.right_move_enable:
            return
        self.process_pose(msg, 'right')

    def left_pose_callback(self, msg: Pose):
        if not self.left_move_enable:
            return
        self.process_pose(msg, 'left')

    def process_pose(self, msg: Pose, arm: str):
        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        orientation = orientation / np.linalg.norm(orientation)

        current_time = self.get_clock().now()

        # Select arm-specific variables
        if arm == 'right':
            prev_position = self.right_prev_position
            prev_orientation = self.right_prev_orientation
            prev_time = self.right_prev_time
            servo_twist_pub = self.right_servo_twist_pub
            servo_joint_pub = self.right_servo_joint_pub
            frame_id = self.right_frame_id
            joint_names = self.right_joint_names
            current_joint_state = self.right_current_joint_state
            prev_linear_velocity = self.right_prev_linear_velocity
            prev_angular_velocity = self.right_prev_angular_velocity
        else:
            prev_position = self.left_prev_position
            prev_orientation = self.left_prev_orientation
            prev_time = self.left_prev_time
            servo_twist_pub = self.left_servo_twist_pub
            servo_joint_pub = self.left_servo_joint_pub
            frame_id = self.left_frame_id
            joint_names = self.left_joint_names
            current_joint_state = self.left_current_joint_state
            prev_linear_velocity = self.left_prev_linear_velocity
            prev_angular_velocity = self.left_prev_angular_velocity

        twist_msg = TwistStamped()
        joint_msg = JointState()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = frame_id
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = joint_names

        if prev_position is None or prev_orientation is None or prev_time is None:
            if arm == 'right':
                self.right_prev_position = position
                self.right_prev_orientation = orientation
                self.right_prev_time = current_time
            else:
                self.left_prev_position = position
                self.left_prev_orientation = orientation
                self.left_prev_time = current_time
            self.get_logger().info(f'Initialized {arm} pose state')
            return

        dt = (current_time - prev_time).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().warn(f'Invalid time difference for {arm}, skipping')
            return

        # Calculate linear and angular velocity
        delta_pos = position - prev_position
        linear_velocity = delta_pos / dt * self.linear_scale

        q1 = prev_orientation
        q2 = orientation
        q1_inv = quaternion_inverse(q1)
        q_diff = quaternion_multiply(q2, q1_inv)
        angular_velocity = np.array([q_diff[0], q_diff[1], q_diff[2]]) * 2.0 / dt * self.angular_scale

        # Apply low-pass filter
        linear_velocity = (1 - self.filter_alpha) * prev_linear_velocity + self.filter_alpha * linear_velocity
        angular_velocity = (1 - self.filter_alpha) * prev_angular_velocity + self.filter_alpha * angular_velocity

        # Update previous velocities
        if arm == 'right':
            self.right_prev_linear_velocity = linear_velocity
            self.right_prev_angular_velocity = angular_velocity
        else:
            self.left_prev_linear_velocity = linear_velocity
            self.left_prev_angular_velocity = angular_velocity

        max_linear_vel = 1.0
        max_angular_vel = 1.0
        linear_velocity = np.clip(linear_velocity, -max_linear_vel, max_linear_vel)
        angular_velocity = np.clip(angular_velocity, -max_angular_vel, max_angular_vel)

        # Publish TwistStamped
        twist_msg.twist.linear.x = float(linear_velocity[0])
        twist_msg.twist.linear.y = float(linear_velocity[1])
        twist_msg.twist.linear.z = float(linear_velocity[2])
        twist_msg.twist.angular.x = float(angular_velocity[0])
        twist_msg.twist.angular.y = float(angular_velocity[1])
        twist_msg.twist.angular.z = float(angular_velocity[2])
        servo_twist_pub.publish(twist_msg)
        self.get_logger().info(f'{arm.capitalize()} Twist published: lx={twist_msg.twist.linear.x:.3f}, ax={twist_msg.twist.angular.x:.3f}')

        # Calculate joint velocities
        if current_joint_state is not None and len(current_joint_state.name) == len(joint_names):
            try:
                jacobian = self.compute_jacobian(current_joint_state, arm)
                twist = np.concatenate([linear_velocity, angular_velocity])
                condition_number = np.linalg.cond(jacobian)
                if condition_number > 1000:
                    self.get_logger().warn(f'{arm.capitalize()} Jacobian near singularity, condition number: {condition_number}')
                    self.get_logger().warn(f'{arm.capitalize()} Joint positions: {current_joint_state.position}')
                    joint_msg.velocity = [0.0] * len(joint_names)
                else:
                    joint_velocities = np.dot(np.linalg.pinv(jacobian), twist)
                    joint_velocities = np.clip(joint_velocities, -1.0, 1.0)
                    joint_msg.velocity = joint_velocities.tolist()
                    servo_joint_pub.publish(joint_msg)
                    self.get_logger().info(f'{arm.capitalize()} Joint velocities published: {joint_msg.velocity}')
            except Exception as e:
                self.get_logger().error(f'Failed to compute {arm} joint velocities: {e}')
        else:
            self.get_logger().warn(f'No valid {arm} joint state available, skipping joint command')

        # Update previous states
        if arm == 'right':
            self.right_prev_position = position
            self.right_prev_orientation = orientation
            self.right_prev_time = current_time
        else:
            self.left_prev_position = position
            self.left_prev_orientation = orientation
            self.left_prev_time = current_time

    def compute_jacobian(self, joint_state, arm: str):
        # UR3e DH parameters (assuming same for both arms, adjust if different)
        d = [0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921]
        a = [0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0]
        alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]

        theta = np.array([joint_state.position[i] for i in range(len(joint_state.name))])
        num_joints = len(theta)
        if num_joints != 6:
            self.get_logger().error(f'Invalid number of joints for {arm}: expected 6, got {num_joints}')
            return np.zeros((6, 6))

        jacobian = np.zeros((6, num_joints))

        # Compute transformation matrices
        T = np.eye(4)
        transforms = []
        for i in range(num_joints):
            ct = np.cos(theta[i])
            st = np.sin(theta[i])
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ A
            transforms.append(T.copy())
            self.get_logger().debug(f'{arm.capitalize()} Transform {i}: {T}')

        # End-effector position
        p_ee = transforms[-1][:3, 3]
        self.get_logger().info(f'{arm.capitalize()} End-effector position: {p_ee}')

        # Compute Jacobian
        for i in range(num_joints):
            z_i = transforms[i][:3, 2]
            p_i = transforms[i][:3, 3]
            jacobian[:3, i] = np.cross(z_i, p_ee - p_i)
            jacobian[3:6, i] = z_i

        condition_number = np.linalg.cond(jacobian)
        self.get_logger().info(f'{arm.capitalize()} Jacobian condition number: {condition_number}')
        return jacobian

    def destroy_node(self):
        self.get_logger().info('Shutting down VRToServoNode')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VRToServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()