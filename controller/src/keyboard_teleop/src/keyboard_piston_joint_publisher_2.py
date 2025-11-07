#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
import math
import threading
import json

class KeyboardJointPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_joint_publisher')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscribe to teleop control topic (JSON in std_msgs/String)
        self.teleop_sub = self.create_subscription(
            StringMsg,
            '/controls/teleop',
            self.teleop_callback,
            10
        )
        
        # Initialize joint states
        self.joint_names = ['chassis_body_revolute', 'RLwheel_revolute', 'RRwheel_revolute', 'FLwheel_revolute', 'FRwheel_revolute', 'bucket_cylinder_prismatic', 'boom_cylinder_prismatic']
        
        # Initialize control values
        self.steer_position = 0.0  # Position control for chasis_body_revolute
        self.bucket_prismatic_length = 0.0  # Length for bucket_cylinder_prismatic
        self.arm_prismatic_length = 0.0    # Length for boom_cylinder_prismatic
        self.wheel_velocity = 0.0  # Velocity for wheel joints (starts at 0)
        
        # Step sizes (per timer callback, i.e., every 0.1 seconds)
        self.steer_limit = 1.57  # radians, [-1.57, 1.57]
        self.bucket_min, self.bucket_max = -700.0, 100.0
        self.arm_min, self.arm_max = -1300.0, 100.0
        self.velocity_step = 0.5
        self.velocity_min, self.velocity_max = -3.0, 3.0
        
        # Threshold for output change detection
        self.output_epsilon = 1e-6
        
        # Previous values for change detection (printing only on change)
        self.prev_values = {
            'steer_position': None,
            'bucket_prismatic_length': None,
            'arm_prismatic_length': None,
            'wheel_velocity': None,
        }
        
        # Latest controls received from teleop topic (defaults per spec)
        self.latest_controls = {
            'rotation': 0.0,
            'brake': 0.0,
            'throttle': 0.0,
            'gear': 'N',
            'boom': 0.0,
            'bucket': 0.0,
            'left_track': 0.0,
            'right_track': 0.0,
            'swing': 0.0,
            'stick': 0.0,
            'device_type': 'wheel_loader',
            'timestamp': 0,
        }
        
        # Create timer (kept but no longer publishes; event-driven publish on input changes)
        self.create_timer(1.0/30.0, self.timer_callback)
        
        self.get_logger().info('Teleop Joint Publisher started (listening to /controls/teleop)')

    def teleop_callback(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
            # Update only known keys; clamp to valid ranges
            def clamp(val, min_v, max_v):
                return max(min_v, min(max_v, val))
            # Track changes only for specified keys
            watched_float_keys = ['rotation', 'brake', 'throttle', 'boom', 'bucket']
            watched_str_keys = ['gear']
            changed_flag = False
            # Snapshot before
            before = {k: self.latest_controls.get(k) for k in watched_float_keys + watched_str_keys}
            if 'rotation' in data:
                self.latest_controls['rotation'] = clamp(float(data['rotation']), -1.0, 1.0)
            if 'brake' in data:
                self.latest_controls['brake'] = clamp(float(data['brake']), 0.0, 1.0)
            if 'throttle' in data:
                self.latest_controls['throttle'] = clamp(float(data['throttle']), 0.0, 1.0)
            if 'gear' in data:
                self.latest_controls['gear'] = str(data['gear'])
            if 'boom' in data:
                self.latest_controls['boom'] = clamp(float(data['boom']), -1.0, 1.0)
            if 'bucket' in data:
                self.latest_controls['bucket'] = clamp(float(data['bucket']), -1.0, 1.0)
            if 'left_track' in data:
                self.latest_controls['left_track'] = clamp(float(data['left_track']), -1.0, 1.0)
            if 'right_track' in data:
                self.latest_controls['right_track'] = clamp(float(data['right_track']), -1.0, 1.0)
            if 'swing' in data:
                self.latest_controls['swing'] = clamp(float(data['swing']), -1.0, 1.0)
            if 'stick' in data:
                self.latest_controls['stick'] = clamp(float(data['stick']), -1.0, 1.0)
            if 'device_type' in data:
                self.latest_controls['device_type'] = str(data['device_type'])
            if 'timestamp' in data:
                self.latest_controls['timestamp'] = int(data['timestamp'])
            # Detect changes
            for k in watched_float_keys:
                if k in data:
                    prev_v = before.get(k)
                    new_v = self.latest_controls.get(k)
                    if prev_v is None or abs(float(prev_v) - float(new_v)) > self.output_epsilon:
                        changed_flag = True
                        break
            if not changed_flag:
                for k in watched_str_keys:
                    if k in data:
                        if before.get(k) != self.latest_controls.get(k):
                            changed_flag = True
                            break
            # If relevant inputs changed, update states and publish once
            if changed_flag:
                self.update_positions_and_velocity()
                self.publish_joint_state()
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /controls/teleop JSON: {e}')

    def update_positions_and_velocity(self):
        # Map rotation (-1..1) to steering position [-1.57..1.57] (inverted)
        rotation = float(self.latest_controls['rotation'])
        self.steer_position = max(-self.steer_limit, min(self.steer_limit, -rotation * self.steer_limit))

        # Map bucket (-1..1) to bucket prismatic length (-1..1), inverted
        bucket = float(self.latest_controls['bucket'])
        self.bucket_prismatic_length = -bucket

        # Map boom (-1..1) to arm prismatic length (-1..1)
        boom = float(self.latest_controls['boom'])
        self.arm_prismatic_length = boom

        # Update wheel velocity using throttle/brake integration
        throttle = float(self.latest_controls['throttle'])  # 0..1
        brake = float(self.latest_controls['brake'])        # 0..1
        delta_v = (throttle - brake) * self.velocity_step
        self.wheel_velocity = max(self.velocity_min, min(self.velocity_max, self.wheel_velocity + delta_v))

        # Only output when values change beyond epsilon
        def changed(a, b):
            if a is None or b is None:
                return True
            return abs(a - b) > self.output_epsilon

        if (
            changed(self.prev_values['bucket_prismatic_length'], self.bucket_prismatic_length) or
            changed(self.prev_values['arm_prismatic_length'], self.arm_prismatic_length) or
            changed(self.prev_values['steer_position'], self.steer_position) or
            changed(self.prev_values['wheel_velocity'], self.wheel_velocity)
        ):
            print("bucket_prismatic_length: ", self.bucket_prismatic_length, "arm_prismatic_length: ", self.arm_prismatic_length)
            print("Chasis angle: ", self.steer_position, "(", self.steer_position*180/3.14159, ")", "Wheel velocity: ", self.wheel_velocity)
            print("-----------------------------------------------------")
            self.prev_values['bucket_prismatic_length'] = self.bucket_prismatic_length
            self.prev_values['arm_prismatic_length'] = self.arm_prismatic_length
            self.prev_values['steer_position'] = self.steer_position
            self.prev_values['wheel_velocity'] = self.wheel_velocity


    def timer_callback(self):
        # Event-driven模式下，定时器不发布，仅保持节点活跃
        return

    def publish_joint_state(self):
        # Publish joint states once (call when inputs changed)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            self.steer_position,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            self.bucket_prismatic_length,
            self.arm_prismatic_length
        ]
        msg.velocity = [
            float('nan'),
            self.wheel_velocity,    # RLwheel_revolute
            self.wheel_velocity,   # RRwheel_revolute
            self.wheel_velocity,    # FLwheel_revolute
            self.wheel_velocity,   # FRwheel_revolute
            float('nan'),
            float('nan'),
        ]
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 