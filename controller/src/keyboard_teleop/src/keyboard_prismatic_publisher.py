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
            'body_rotate',
            'front_left_wheel_angle',
            'back_left_wheel_angle',
            'front_right_wheel_angle',
            'back_right_wheel_angle',
        ]

        # Publisher
        self.joint_pub = self.create_publisher(JointState, topic, 10)

        # State (3 prismatic in meters, 1 revolute in radians)
        self.bucket_pos = 0.0
        self.arm_pos = 0.0
        self.boom_pos = 0.0
        self.body_yaw = 0.0
        
        # Track velocities (for differential track model)
        self.left_track_velocity = 0.0
        self.right_track_velocity = 0.0

        # Step sizes per timer tick (0.1s): tune as needed
        self.step_linear = 0.05  # meters per tick
        self.step_yaw = 0.1      # radians per tick
        
        # Track velocity step size and scale
        self.track_velocity_step = 0.5  # velocity change per tick
        self.track_velocity_scale = 10.0  # scale factor for track velocity
        self.track_velocity_min = -3.0   # minimum velocity (rad/s)
        self.track_velocity_max = 3.0    # maximum velocity (rad/s)

        # Limits (symmetric for simplicity)
        self.lin_limit = 0.5    # meters
        self.yaw_limit = math.pi # radians

        # Key states for continuous movement
        # Linear: bucket (U/J), arm (I/K), boom (O/L)
        # Rotate: body (A/D)
        # Tracks: forward/backward (W/S), left/right turn (Q/E)
        self.active_keys = {
            'u': False,  # bucket +
            'j': False,  # bucket -
            'i': False,  # arm +
            'k': False,  # arm -
            'o': False,  # boom +
            'l': False,  # boom -
            'a': False,  # body + (left)
            'd': False,  # body - (right)
            'w': False,  # tracks forward (both)
            's': False,  # tracks backward (both)
            'q': False,  # left track forward / right track backward (left turn)
            'e': False,  # right track forward / left track backward (right turn)
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
        self.get_logger().info('Controls: U/J(bucket) I/K(arm) O/L(boom) A/D(body_yaw)')
        self.get_logger().info('Tracks: W/S(forward/backward) Q/E(left/right turn) | R reset | Esc quit')

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                self.get_logger().info('Stopping node...')
                rclpy.shutdown()
                return False

            # Reset all joints and tracks
            if key.char == 'r':
                self.bucket_pos = 0.0
                self.arm_pos = 0.0
                self.boom_pos = 0.0
                self.body_yaw = 0.0
                self.left_track_velocity = 0.0
                self.right_track_velocity = 0.0
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

        # Track velocities (W/S for forward/backward, Q/E for left/right turn)
        # Forward/backward
        if self.active_keys['w']:
            # Forward: both tracks forward
            delta = self.track_velocity_step
            self.left_track_velocity = max(self.track_velocity_min, 
                                         min(self.track_velocity_max, 
                                             self.left_track_velocity - delta * self.track_velocity_scale))
            self.right_track_velocity = max(self.track_velocity_min, 
                                          min(self.track_velocity_max, 
                                              self.right_track_velocity - delta * self.track_velocity_scale))
        if self.active_keys['s']:
            # Backward: both tracks backward
            delta = self.track_velocity_step
            self.left_track_velocity = max(self.track_velocity_min, 
                                         min(self.track_velocity_max, 
                                             self.left_track_velocity + delta * self.track_velocity_scale))
            self.right_track_velocity = max(self.track_velocity_min, 
                                          min(self.track_velocity_max, 
                                              self.right_track_velocity + delta * self.track_velocity_scale))
        
        # Left/right turn
        if self.active_keys['q']:
            # Left turn: left track backward, right track forward
            delta = self.track_velocity_step
            self.left_track_velocity = max(self.track_velocity_min, 
                                         min(self.track_velocity_max, 
                                             self.left_track_velocity + delta * self.track_velocity_scale))
            self.right_track_velocity = max(self.track_velocity_min, 
                                          min(self.track_velocity_max, 
                                              self.right_track_velocity - delta * self.track_velocity_scale))
        if self.active_keys['e']:
            # Right turn: left track forward, right track backward
            delta = self.track_velocity_step
            self.left_track_velocity = max(self.track_velocity_min, 
                                         min(self.track_velocity_max, 
                                             self.left_track_velocity - delta * self.track_velocity_scale))
            self.right_track_velocity = max(self.track_velocity_min, 
                                          min(self.track_velocity_max, 
                                              self.right_track_velocity + delta * self.track_velocity_scale))
        
        # Decay track velocities when no keys are pressed (gradual stop)
        if not (self.active_keys['w'] or self.active_keys['s'] or 
                self.active_keys['q'] or self.active_keys['e']):
            # Gradual decay to zero
            decay_factor = 0.9
            self.left_track_velocity *= decay_factor
            self.right_track_velocity *= decay_factor
            if abs(self.left_track_velocity) < 0.1:
                self.left_track_velocity = 0.0
            if abs(self.right_track_velocity) < 0.1:
                self.right_track_velocity = 0.0

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
            float('nan'),      # front_left_wheel_angle (velocity control)
            float('nan'),      # back_left_wheel_angle (velocity control)
            float('nan'),      # front_right_wheel_angle (velocity control)
            float('nan'),      # back_right_wheel_angle (velocity control)
        ]
        msg.velocity = [
            0.0,                        # bucket_linear
            0.0,                        # arm_linear
            0.0,                        # boom_linear
            0.0,                        # body_rotate
            self.left_track_velocity,   # front_left_wheel_angle
            self.left_track_velocity,   # back_left_wheel_angle
            self.right_track_velocity,  # front_right_wheel_angle
            self.right_track_velocity,  # back_right_wheel_angle
        ]
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
