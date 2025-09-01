#!/usr/bin/python3.10

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard
import math


class KeyboardPrismaticPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_prismatic_publisher')

        # Params for topic and joint names (can be overridden via ROS2 parameters)
        self.declare_parameter('topic', '/pc2000_joint_command')
        self.declare_parameter('joint_bucket', 'bucket_piston')
        self.declare_parameter('joint_arm', 'arm_piston')
        self.declare_parameter('joint_boom', 'boom_piston')
        self.declare_parameter('joint_body_rotate', 'body_rotate')

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.joint_names = [
            'bucket_linear',
            'arm_linear',
            'boom_linear',
            'body_rotate'
        ]

        # Publisher
        self.joint_pub = self.create_publisher(JointState, topic, 10)

        # State (3 prismatic in meters, 1 revolute in radians)
        self.bucket_pos = 0.0
        self.arm_pos = 0.0
        self.boom_pos = 0.0
        self.body_yaw = 0.0

        # Step sizes per timer tick (0.1s): tune as needed
        self.step_linear = 0.05  # meters per tick
        self.step_yaw = 0.1      # radians per tick

        # Limits (symmetric for simplicity)
        self.lin_limit = 0.5    # meters
        self.yaw_limit = math.pi # radians

        # Key states for continuous movement
        # Linear: bucket (U/J), arm (I/K), boom (O/L)
        # Rotate: body (A/D)
        self.active_keys = {
            'u': False,  # bucket +
            'j': False,  # bucket -
            'i': False,  # arm +
            'k': False,  # arm -
            'o': False,  # boom +
            'l': False,  # boom -
            'a': False,  # body + (left)
            'd': False,  # body - (right)
        }

        # Keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.keyboard_listener.start()

        # Publish timer
        self.create_timer(0.1, self.timer_callback)

        # Logs
        self.get_logger().info('Keyboard Prismatic Publisher started')
        self.get_logger().info('Topic: %s' % topic)
        self.get_logger().info('Joint order: %s' % ', '.join(self.joint_names))
        self.get_logger().info('Controls: U/J(bucket) I/K(arm) O/L(boom) A/D(body_yaw) | Q reset | Esc quit')

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                self.get_logger().info('Stopping node...')
                rclpy.shutdown()
                return False

            # Reset all joints
            if key.char == 'q':
                self.bucket_pos = 0.0
                self.arm_pos = 0.0
                self.boom_pos = 0.0
                self.body_yaw = 0.0
            elif key.char in self.active_keys:
                self.active_keys[key.char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in self.active_keys:
                self.active_keys[key.char] = False
        except AttributeError:
            pass

    def update_positions(self):
        # Bucket prismatic (U/J)
        if self.active_keys['u']:
            self.bucket_pos = min(self.bucket_pos + self.step_linear, self.lin_limit)
        if self.active_keys['j']:
            self.bucket_pos = max(self.bucket_pos - self.step_linear, -self.lin_limit)

        # Arm prismatic (I/K)
        if self.active_keys['i']:
            self.arm_pos = min(self.arm_pos + self.step_linear, self.lin_limit)
        if self.active_keys['k']:
            self.arm_pos = max(self.arm_pos - self.step_linear, -self.lin_limit)

        # Boom prismatic (O/L)
        if self.active_keys['o']:
            self.boom_pos = min(self.boom_pos + self.step_linear, self.lin_limit)
        if self.active_keys['l']:
            self.boom_pos = max(self.boom_pos - self.step_linear, -self.lin_limit)

        # Body rotate (A/D)
        if self.active_keys['a']:
            self.body_yaw = min(self.body_yaw + self.step_yaw, self.yaw_limit)
        if self.active_keys['d']:
            self.body_yaw = max(self.body_yaw - self.step_yaw, -self.yaw_limit)

    def timer_callback(self):
        self.update_positions()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            -self.bucket_pos,
            -self.arm_pos,
            -self.boom_pos,
            self.body_yaw,
        ]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPrismaticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keyboard_listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
