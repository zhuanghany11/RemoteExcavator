#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
import math
import json


class TeleopPrismaticPublisher(Node):
    def __init__(self):
        super().__init__('teleop_prismatic_publisher')

        # Params for topic and joint names (can be overridden via ROS2 parameters)
        self.declare_parameter('topic', '/pc2000_joint_command')
        self.declare_parameter('joint_bucket', 'bucket_linear')
        self.declare_parameter('joint_arm', 'arm_linear')
        self.declare_parameter('joint_boom', 'boom_linear')
        self.declare_parameter('joint_body_rotate', 'body_rotate')
        # Track velocity scale parameter (for differential tracks)
        self.declare_parameter('track_velocity_scale', 10.0)

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

        # QoS profile for teleop subscription (compatible with most publishers)
        teleop_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 兼容 BEST_EFFORT 发布者
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to teleop control topic (JSON in std_msgs/String)
        self.teleop_sub = self.create_subscription(
            StringMsg,
            '/controls/teleop',
            self.teleop_callback,
            teleop_qos
        )
        
        # Subscribe to joint states to get current positions on startup
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # State (3 prismatic in meters, 1 revolute in radians)
        # Initialize to middle position (0.0) on startup
        self.bucket_pos = 0.0
        self.arm_pos = 0.0
        self.boom_pos = 0.0
        self.body_yaw = 0.0
        
        # Track velocities (for differential track model)
        self.left_track_velocity = 0.0
        self.right_track_velocity = 0.0
        
        # Flag to track if we've received first control command
        self.initialized = True  # Set to True so we can publish initial positions immediately

        # Step sizes per timer tick (0.1s): tune as needed (per-axis)
        self.step_linear = 0.01            # default linear step (fallback)
        self.step_bucket_linear = 0.03     # meters/tick for bucket
        self.step_arm_linear = 0.02        # meters/tick for arm (stick)
        self.step_boom_linear = 0.02       # meters/tick for boom
        self.step_body_yaw = 0.04          # radians/tick for body rotation
        
        # Track velocity mapping (direct mapping to wheel angular velocity)
        # Default conservative value to avoid instability; configurable via ROS parameter
        self.track_velocity_scale = float(self.get_parameter('track_velocity_scale').get_parameter_value().double_value)

        # Limits (symmetric for simplicity)
        self.lin_limit = 1.0    # meters (for bucket and arm)
        self.boom_limit = 1.0   # meters (larger range for boom)
        self.yaw_limit = math.pi # radians

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

        # Threshold for output change detection
        self.output_epsilon = 1e-6

        # Previous values for change detection (printing only on change)
        self.prev_values = {
            'bucket_pos': None,
            'arm_pos': None,
            'boom_pos': None,
            'body_yaw': None,
            'left_track_velocity': None,
            'right_track_velocity': None,
        }

        # Log throttling: only log every N seconds to avoid excessive output
        self.last_log_time = 0.0
        self.log_interval = 1.0  # Log at most once per second

        # Publish timer (for continuous updates even without new teleop messages)
        # Reduced frequency to 20Hz (0.05s) to reduce CPU load
        self.create_timer(0.05, self.timer_callback)

        # Logs
        self.get_logger().info('Teleop Prismatic Publisher started')
        self.get_logger().info('Topic: %s' % topic)
        self.get_logger().info('Joint order: %s' % ', '.join(self.joint_names))
        self.get_logger().info('Listening to /controls/teleop for control commands')
        self.get_logger().info('Initializing bucket, arm, and boom to middle position (0.0)')
        
        # Publish initial middle positions after a short delay to ensure subscribers are ready
        self.initial_publish_done = False
        self.create_timer(0.5, self.publish_initial_positions)

    def publish_initial_positions(self):
        """Publish initial middle positions once on startup"""
        if not self.initial_publish_done:
            self.publish_joint_state()
            self.initial_publish_done = True
            self.get_logger().info('Published initial middle positions for bucket, arm, and boom')

    def teleop_callback(self, msg: StringMsg):
        # 添加调试信息：确认收到消息
        # self.get_logger().info(f'[DEBUG] Received teleop message: {msg.data[:100]}...')
        try:
            data = json.loads(msg.data)
            # self.get_logger().info(f'[DEBUG] Parsed JSON successfully. Keys: {list(data.keys())}')
            # Update only known keys; clamp to valid ranges
            def clamp(val, min_v, max_v):
                return max(min_v, min(max_v, val))
            
            # Track changes only for specified keys
            watched_keys = ['bucket', 'stick', 'boom', 'swing', 'rotation', 'left_track', 'right_track']
            changed_flag = False
            
            # Snapshot before
            before = {k: self.latest_controls.get(k) for k in watched_keys}
            
            if 'bucket' in data:
                self.latest_controls['bucket'] = clamp(float(data['bucket']), -1.0, 1.0)
            if 'stick' in data:
                self.latest_controls['stick'] = clamp(float(data['stick']), -1.0, 1.0)
            if 'boom' in data:
                self.latest_controls['boom'] = clamp(float(data['boom']), -1.0, 1.0)
            if 'swing' in data:
                self.latest_controls['swing'] = clamp(float(data['swing']), -1.0, 1.0)
            if 'rotation' in data:
                self.latest_controls['rotation'] = clamp(float(data['rotation']), -1.0, 1.0)
            if 'brake' in data:
                self.latest_controls['brake'] = clamp(float(data['brake']), 0.0, 1.0)
            if 'throttle' in data:
                self.latest_controls['throttle'] = clamp(float(data['throttle']), 0.0, 1.0)
            if 'gear' in data:
                self.latest_controls['gear'] = str(data['gear'])
            if 'left_track' in data:
                self.latest_controls['left_track'] = clamp(float(data['left_track']), -1.0, 1.0)
            if 'right_track' in data:
                self.latest_controls['right_track'] = clamp(float(data['right_track']), -1.0, 1.0)
            if 'device_type' in data:
                self.latest_controls['device_type'] = str(data['device_type'])
            if 'timestamp' in data:
                self.latest_controls['timestamp'] = int(data['timestamp'])
            
            # Detect changes
            for k in watched_keys:
                if k in data:
                    prev_v = before.get(k)
                    new_v = self.latest_controls.get(k)
                    if prev_v is None or abs(float(prev_v) - float(new_v)) > self.output_epsilon:
                        changed_flag = True
                        # self.get_logger().info(f'[DEBUG] Detected change in {k}: {prev_v} -> {new_v}')
                        break

            # Ensure first command always triggers initialization and processing
            if not self.initialized:
                changed_flag = True
            
            # If relevant inputs changed, update states immediately
            if changed_flag:
                # self.get_logger().info('[DEBUG] Processing changed controls...')
                self.update_positions()
                self.publish_joint_state()
            # else:
            #     self.get_logger().info('[DEBUG] No significant changes detected')
                
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /controls/teleop JSON: {e}')

    def joint_state_callback(self, msg: JointState):
        # Note: Positions are now initialized to middle position (0.0) on startup
        # This callback is kept for potential future use but currently does not sync positions
        # Sync current positions from joint states on first message only (disabled)
        if not self.initialized:
            try:
                # Find indices of our joints in the joint state message
                for i, joint_name in enumerate(msg.name):
                    if joint_name == 'bucket_linear':
                        self.bucket_pos = -msg.position[i]  # Inverted to match our convention
                    elif joint_name == 'arm_linear':
                        self.arm_pos = -msg.position[i]  # Inverted to match our convention
                    elif joint_name == 'boom_linear':
                        self.boom_pos = -msg.position[i]  # Inverted to match our convention
                    elif joint_name == 'body_rotate':
                        self.body_yaw = msg.position[i]
                
                # If we got at least one position, mark as initialized
                if self.bucket_pos is not None or self.arm_pos is not None or \
                   self.boom_pos is not None or self.body_yaw is not None:
                    # Set any None values to 0.0
                    if self.bucket_pos is None:
                        self.bucket_pos = 0.0
                    if self.arm_pos is None:
                        self.arm_pos = 0.0
                    if self.boom_pos is None:
                        self.boom_pos = 0.0
                    if self.body_yaw is None:
                        self.body_yaw = 0.0
                    self.initialized = True
                    self.get_logger().info(
                        f'Synced current positions: bucket={self.bucket_pos:.3f}, '
                        f'arm={self.arm_pos:.3f}, boom={self.boom_pos:.3f}, '
                        f'body_yaw={self.body_yaw:.3f}'
                    )
            except Exception as e:
                self.get_logger().warn(f'Failed to sync joint states: {e}')

    def update_positions(self):
        # Positions are already initialized to middle position (0.0) in __init__
        # Map bucket (-1..1) to bucket prismatic position
        # Positive input -> extend bucket (positive position)
        bucket = float(self.latest_controls['bucket'])
        delta_bucket = bucket * self.step_bucket_linear
        self.bucket_pos = max(-self.lin_limit, min(self.lin_limit, self.bucket_pos + delta_bucket))

        # Map stick (-1..1) to arm prismatic position
        # Positive input -> extend arm (positive position)
        stick = float(self.latest_controls['stick'])
        delta_arm = stick * self.step_arm_linear
        self.arm_pos = max(-self.lin_limit, min(self.lin_limit, self.arm_pos + delta_arm))

        # Map boom (-1..1) to boom prismatic position
        # Positive input -> extend boom (positive position)
        boom = float(self.latest_controls['boom'])
        delta_boom = boom * self.step_boom_linear
        self.boom_pos = max(-self.boom_limit, min(self.boom_limit, self.boom_pos + delta_boom))

        # Map swing or rotation (-1..1) to body yaw
        # Prefer swing, fallback to rotation
        swing = - float(self.latest_controls.get('swing', 0.0))
        rotation = float(self.latest_controls.get('rotation', 0.0))
        # Use swing if available, otherwise use rotation
        body_input = swing if abs(swing) > self.output_epsilon else rotation
        delta_yaw = body_input * self.step_body_yaw
        self.body_yaw = max(-self.yaw_limit, min(self.yaw_limit, self.body_yaw + delta_yaw))
        
        # Map left_track and right_track (-1..1) to track velocities (differential drive model)
        # Direct mapping to wheel angular velocities (rad/s or sim units)
        left_track = float(self.latest_controls.get('left_track', 0.0))
        right_track = float(self.latest_controls.get('right_track', 0.0))
        
        # Dead zone: -0.5 to 0.5 range produces no movement
        # Apply dead zone by clamping values outside the dead zone
        track_dead_zone = 0.5
        if abs(left_track) <= track_dead_zone:
            left_track = 0.0
        else:
            # Scale the remaining range (-1 to -0.5 and 0.5 to 1) to full range
            if left_track > 0:
                left_track = (left_track - track_dead_zone) / (1.0 - track_dead_zone)
            else:
                left_track = (left_track + track_dead_zone) / (1.0 - track_dead_zone)
        
        if abs(right_track) <= track_dead_zone:
            right_track = 0.0
        else:
            # Scale the remaining range (-1 to -0.5 and 0.5 to 1) to full range
            if right_track > 0:
                right_track = (right_track - track_dead_zone) / (1.0 - track_dead_zone)
            else:
                right_track = (right_track + track_dead_zone) / (1.0 - track_dead_zone)
        
        # Sign convention: forward ≈ -500, backward ≈ +500
        # Note: direction corrected - positive input now maps to forward (negative velocity)
        self.left_track_velocity = left_track * self.track_velocity_scale
        self.right_track_velocity = right_track * self.track_velocity_scale

        # Only output when values change beyond epsilon
        def changed(a, b):
            if a is None or b is None:
                return True
            return abs(a - b) > self.output_epsilon

        values_changed = (
            changed(self.prev_values['bucket_pos'], self.bucket_pos) or
            changed(self.prev_values['arm_pos'], self.arm_pos) or
            changed(self.prev_values['boom_pos'], self.boom_pos) or
            changed(self.prev_values['body_yaw'], self.body_yaw) or
            changed(self.prev_values['left_track_velocity'], self.left_track_velocity) or
            changed(self.prev_values['right_track_velocity'], self.right_track_velocity)
        )
        
        if values_changed:
            # Throttle logging to avoid excessive output that can cause system freeze
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.last_log_time >= self.log_interval:
                self.get_logger().info(
                    f"bucket: {self.bucket_pos:.3f}, arm: {self.arm_pos:.3f}, "
                    f"boom: {self.boom_pos:.3f}, body_yaw: {self.body_yaw:.3f} ({math.degrees(self.body_yaw):.1f}°), "
                    f"tracks: L={self.left_track_velocity:.2f} R={self.right_track_velocity:.2f}"
                )
                self.last_log_time = current_time
            
            # Always update prev_values even if we don't log
            self.prev_values['bucket_pos'] = self.bucket_pos
            self.prev_values['arm_pos'] = self.arm_pos
            self.prev_values['boom_pos'] = self.boom_pos
            self.prev_values['body_yaw'] = self.body_yaw
            self.prev_values['left_track_velocity'] = self.left_track_velocity
            self.prev_values['right_track_velocity'] = self.right_track_velocity

    def timer_callback(self):
        # Only update and publish after receiving first control command
        if self.initialized:
            # Store previous state to check if we need to publish
            prev_bucket = self.bucket_pos
            prev_arm = self.arm_pos
            prev_boom = self.boom_pos
            prev_yaw = self.body_yaw
            prev_left_track = self.left_track_velocity
            prev_right_track = self.right_track_velocity
            
            # Update positions based on latest controls (for continuous movement)
            self.update_positions()
            
            # Only publish if values actually changed (reduces unnecessary message spam)
            if (abs(self.bucket_pos - prev_bucket) > self.output_epsilon or
                abs(self.arm_pos - prev_arm) > self.output_epsilon or
                abs(self.boom_pos - prev_boom) > self.output_epsilon or
                abs(self.body_yaw - prev_yaw) > self.output_epsilon or
                abs(self.left_track_velocity - prev_left_track) > self.output_epsilon or
                abs(self.right_track_velocity - prev_right_track) > self.output_epsilon):
                self.publish_joint_state()

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            -self.bucket_pos,  # Inverted like in keyboard_prismatic_publisher
            -self.arm_pos,     # Inverted like in keyboard_prismatic_publisher
            -self.boom_pos,    # Inverted like in keyboard_prismatic_publisher
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
    node = TeleopPrismaticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

