#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard
import math

class KeyboardJointPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_joint_publisher')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/pc2000_joint_command', 10)
        
        # Initialize joint states for excavator
        self.joint_names = [
            'forearm_shovel_joint',
            'arm_forearm_joint',
            'tank_body_joint',
            'body_arm_joint'
        ]
        
        # Initialize control values (positions only)
        self.forearm_shovel_position = 0.0
        self.arm_forearm_position = 0.0
        self.tank_body_position = 0.0
        self.body_arm_position = 0.0
        
        # Step sizes (per timer callback, i.e., every 0.1 seconds)
        self.step_shovel = 0.1
        self.step_arm = 0.1
        self.step_tank = 0.1
        self.step_body = 0.1
        
        # Key states for continuous movement
        self.active_keys = {
            'u': False,  # forearm_shovel increase
            'j': False,  # forearm_shovel decrease
            'i': False,  # arm_forearm increase
            'k': False,  # arm_forearm decrease
            'a': False,  # tank_body increase (rotate left)
            'd': False,  # tank_body decrease (rotate right)
            'w': False,  # body_arm increase (raise)
            's': False,  # body_arm decrease (lower)
        }
        
        # Create and start keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()
        
        # Create timer for publishing joint states and updating positions
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Keyboard Joint Publisher started')
        self.get_logger().info('Use the following keys to control joints:')
        self.get_logger().info('U/J: forearm_shovel_joint increase/decrease')
        self.get_logger().info('I/K: arm_forearm_joint increase/decrease')
        self.get_logger().info('A/D: tank_body_joint increase/decrease')
        self.get_logger().info('W/S: body_arm_joint increase/decrease')
        self.get_logger().info('Q: Reset all joint positions to 0')
        self.get_logger().info('Press Esc to quit')

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                # Stop the node
                self.get_logger().info('Stopping node...')
                rclpy.shutdown()
                return False
            
            # Q key resets all joint positions to 0
            if key.char == 'q':
                self.forearm_shovel_position = 0.0
                self.arm_forearm_position = 0.0
                self.tank_body_position = 0.0
                self.body_arm_position = 0.0
            # Update key states for continuous movement
            elif key.char in self.active_keys:
                self.active_keys[key.char] = True
                
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            # Update key states for continuous movement
            if key.char in self.active_keys:
                self.active_keys[key.char] = False
        except AttributeError:
            pass

    def update_positions_and_velocity(self):
        # tank_body_joint (A/D)
        if self.active_keys['a']:
            self.tank_body_position = min(self.tank_body_position + self.step_tank, math.pi)
        if self.active_keys['d']:
            self.tank_body_position = max(self.tank_body_position - self.step_tank, -math.pi)
        
        # forearm_shovel_joint (U/J)
        if self.active_keys['u']:
            self.forearm_shovel_position = min(self.forearm_shovel_position + self.step_shovel, 3.0)
        if self.active_keys['j']:
            self.forearm_shovel_position = max(self.forearm_shovel_position - self.step_shovel, -3.0)
        
        # arm_forearm_joint (I/K)
        if self.active_keys['i']:
            self.arm_forearm_position = min(self.arm_forearm_position + self.step_arm, 3.0)
        if self.active_keys['k']:
            self.arm_forearm_position = max(self.arm_forearm_position - self.step_arm, -3.0)
        
        # body_arm_joint (W/S)
        if self.active_keys['w']:
            self.body_arm_position = min(self.body_arm_position + self.step_body, 3.0)
        if self.active_keys['s']:
            self.body_arm_position = max(self.body_arm_position - self.step_body, -3.0)


    def timer_callback(self):
        # Update positions based on held keys
        self.update_positions_and_velocity()
        
        # Publish joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Set positions (order must match joint_names)
        msg.position = [
            self.forearm_shovel_position,
            self.arm_forearm_position,
            self.tank_body_position,
            self.body_arm_position,
        ]
        
        # Set velocities and efforts (zeros)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJointPublisher()
    
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