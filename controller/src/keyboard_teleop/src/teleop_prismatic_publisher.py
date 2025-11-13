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
        # Initialize boom to 2/3 of maximum height (lifted position)
        self.boom_pos = 1.0 * 2.0 / 3.0  # 2/3 of boom_limit (1.0m) = 0.667m
        self.body_yaw = 0.0
        
        # Track velocities (for differential track model)
        self.left_track_velocity = 0.0
        self.right_track_velocity = 0.0
        
        # Flag to track if we've received first control command
        self.initialized = True  # Set to True so we can publish initial positions immediately

        # Step sizes per timer tick (0.1s): tune as needed (per-axis)
        self.step_linear = 0.01            # default linear step (fallback)
        self.step_bucket_linear = 0.01     # meters/tick for bucket (reduced for less sensitive control)
        self.step_arm_linear = 0.02        # meters/tick for arm (stick)
        self.step_boom_linear = 0.01       # meters/tick for boom (reduced for less sensitive control)
        self.step_body_yaw = 0.01          # radians/tick for body rotation (reduced for less sensitive control)
        
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
        self.log_interval = 2.0  # Log at most once per 2 seconds (reduced frequency)
        self.log_count = 0  # Counter to limit total log messages

        # Cached constants for performance
        self.track_dead_zone = 0.5
        self.track_dead_zone_inv = 1.0 / (1.0 - self.track_dead_zone)  # Pre-compute division
        
        # Pre-allocate message object to reduce allocation overhead
        self.joint_msg = JointState()
        self.joint_msg.name = self.joint_names
        self.joint_msg.effort = [0.0] * len(self.joint_names)
        
        # Pre-allocate position and velocity lists
        self.joint_msg.position = [0.0] * len(self.joint_names)
        self.joint_msg.velocity = [0.0] * len(self.joint_names)
        
        # Publish timer (for continuous updates even without new teleop messages)
        # Reduced frequency to 10Hz (0.1s) to reduce CPU load
        self.create_timer(0.1, self.timer_callback)

        # Logs
        self.get_logger().info('Teleop Prismatic Publisher started')
        self.get_logger().info('Topic: %s' % topic)
        self.get_logger().info('Joint order: %s' % ', '.join(self.joint_names))
        self.get_logger().info('Listening to /controls/teleop for control commands')
        self.get_logger().info('Initializing bucket and arm to middle position (0.0)')
        self.get_logger().info(f'Initializing boom to 2/3 height position ({self.boom_pos:.3f}m)')
        
        # Publish initial positions after a short delay to ensure subscribers are ready
        self.initial_publish_done = False
        self.create_timer(0.5, self.publish_initial_positions)

    def publish_initial_positions(self):
        """Publish initial positions once on startup"""
        if not self.initial_publish_done:
            self.publish_joint_state()
            self.initial_publish_done = True
            self.get_logger().info('Published initial positions: bucket and arm at middle (0.0), boom at 2/3 height (0.667m)')

    def teleop_callback(self, msg: StringMsg):
        # Optimized callback with reduced dictionary lookups and string operations
        try:
            data = json.loads(msg.data)
            
            # Cache controls dict reference to reduce lookups
            controls = self.latest_controls
            epsilon = self.output_epsilon
            changed_flag = False
            
            # Fast path: check for watched keys first and update in one pass
            # Use get() with default to avoid KeyError and reduce lookups
            bucket_val = data.get('bucket')
            stick_val = data.get('stick')
            boom_val = data.get('boom')
            swing_val = data.get('swing')
            rotation_val = data.get('rotation')
            left_track_val = data.get('left_track')
            right_track_val = data.get('right_track')
            
            # Update and check changes in one pass (reduces dictionary lookups)
            if bucket_val is not None:
                new_val = max(-1.0, min(1.0, float(bucket_val)))
                if abs(controls['bucket'] - new_val) > epsilon:
                    controls['bucket'] = new_val
                    changed_flag = True
            if stick_val is not None:
                new_val = max(-1.0, min(1.0, float(stick_val)))
                if abs(controls['stick'] - new_val) > epsilon:
                    controls['stick'] = new_val
                    changed_flag = True
            if boom_val is not None:
                new_val = max(-1.0, min(1.0, float(boom_val)))
                if abs(controls['boom'] - new_val) > epsilon:
                    controls['boom'] = new_val
                    changed_flag = True
            if swing_val is not None:
                new_val = max(-1.0, min(1.0, float(swing_val)))
                if abs(controls['swing'] - new_val) > epsilon:
                    controls['swing'] = new_val
                    changed_flag = True
            if rotation_val is not None:
                new_val = max(-1.0, min(1.0, float(rotation_val)))
                if abs(controls['rotation'] - new_val) > epsilon:
                    controls['rotation'] = new_val
                    changed_flag = True
            if left_track_val is not None:
                new_val = max(-1.0, min(1.0, float(left_track_val)))
                if abs(controls['left_track'] - new_val) > epsilon:
                    controls['left_track'] = new_val
                    changed_flag = True
                    # Print input value for debugging
                    # self.get_logger().info(f'[INPUT] left_track: {new_val:.3f} (raw: {left_track_val})')
            if right_track_val is not None:
                new_val = max(-1.0, min(1.0, float(right_track_val)))
                if abs(controls['right_track'] - new_val) > epsilon:
                    controls['right_track'] = new_val
                    changed_flag = True
                    # Print input value for debugging
                    # self.get_logger().info(f'[INPUT] right_track: {new_val:.3f} (raw: {right_track_val})')
            
            # Update other fields (less frequently changed, no change detection needed)
            brake_val = data.get('brake')
            if brake_val is not None:
                controls['brake'] = max(0.0, min(1.0, float(brake_val)))
            throttle_val = data.get('throttle')
            if throttle_val is not None:
                controls['throttle'] = max(0.0, min(1.0, float(throttle_val)))
            if 'gear' in data:
                controls['gear'] = str(data['gear'])
            if 'device_type' in data:
                controls['device_type'] = str(data['device_type'])
            if 'timestamp' in data:
                controls['timestamp'] = int(data['timestamp'])
            
            # If relevant inputs changed, update states immediately
            if changed_flag:
                self.update_positions()
                self.publish_joint_state()
                
        except (ValueError, TypeError, KeyError) as e:
            # Limit error logging to avoid spam
            if self.log_count < 10:  # Only log first 10 errors
                self.get_logger().warn(f'Failed to parse /controls/teleop JSON: {e}')
                self.log_count += 1
        except Exception as e:
            # Catch-all for unexpected errors, but limit logging
            if self.log_count < 10:
                self.get_logger().warn(f'Unexpected error in teleop_callback: {e}')
                self.log_count += 1

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
        # Positive input -> retract bucket (negative position) - direction inverted
        bucket = float(self.latest_controls['bucket'])
        delta_bucket = -bucket * self.step_bucket_linear  # Inverted direction
        self.bucket_pos = max(-self.lin_limit, min(self.lin_limit, self.bucket_pos + delta_bucket))

        # Map stick (-1..1) to arm prismatic position
        # Positive input -> retract arm (negative position) - direction inverted
        stick = float(self.latest_controls['stick'])
        delta_arm = -stick * self.step_arm_linear  # Inverted direction
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
        # Dead zone: -0.5 to 0.5 range produces no movement (applied BEFORE scaling)
        # Total input range: -1 to 1, dead zone: -0.5 to 0.5
        left_track = self.latest_controls.get('left_track', 0.0)
        right_track = self.latest_controls.get('right_track', 0.0)
        dead_zone = self.track_dead_zone
        dead_zone_inv = self.track_dead_zone_inv
        
        # Apply dead zone: if input is in [-0.5, 0.5], output is 0
        # Otherwise, map from [-1, -0.5] U [0.5, 1] to [-1, 1]
        if abs(left_track) <= dead_zone:
            left_track = 0.0
        else:
            # Map from [-1, -0.5] or [0.5, 1] to [-1, 1]
            # For positive: map [0.5, 1] -> [0, 1] -> scale to [0, 1] then keep sign
            # For negative: map [-1, -0.5] -> [-1, 0] -> scale to [-1, 0] then keep sign
            if left_track > 0:
                # Map [0.5, 1] -> [0, 1]
                left_track = (left_track - dead_zone) * dead_zone_inv
            else:
                # Map [-1, -0.5] -> [-1, 0]
                left_track = (left_track + dead_zone) * dead_zone_inv
        
        if abs(right_track) <= dead_zone:
            right_track = 0.0
        else:
            # Same mapping for right track
            if right_track > 0:
                right_track = (right_track - dead_zone) * dead_zone_inv
            else:
                right_track = (right_track + dead_zone) * dead_zone_inv
        
        # After dead zone processing, left_track and right_track are in [-1, 1] range
        # Now multiply by scale factor to get final velocity
        # Sign convention: forward ≈ -500, backward ≈ +500
        scale = self.track_velocity_scale
        self.left_track_velocity = left_track * scale
        self.right_track_velocity = right_track * scale

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
            # Use simpler time check to reduce overhead
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.last_log_time >= self.log_interval:
                # Use % formatting instead of f-strings for better performance with logging
                self.get_logger().info(
                    "bucket: %.3f, arm: %.3f, boom: %.3f, body_yaw: %.3f (%.1f°), "
                    "tracks: L=%.2f R=%.2f" % (
                        self.bucket_pos, self.arm_pos, self.boom_pos,
                        self.body_yaw, math.degrees(self.body_yaw),
                        self.left_track_velocity, self.right_track_velocity
                    )
                )
                self.last_log_time = current_time
            
            # Always update prev_values even if we don't log
            prev = self.prev_values
            prev['bucket_pos'] = self.bucket_pos
            prev['arm_pos'] = self.arm_pos
            prev['boom_pos'] = self.boom_pos
            prev['body_yaw'] = self.body_yaw
            prev['left_track_velocity'] = self.left_track_velocity
            prev['right_track_velocity'] = self.right_track_velocity

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
        # Use pre-allocated message object to reduce allocation overhead
        msg = self.joint_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Update position and velocity lists in-place (faster than creating new lists)
        pos = msg.position
        vel = msg.velocity
        
        # Update positions (inverted for bucket, arm, boom)
        pos[0] = -self.bucket_pos
        pos[1] = -self.arm_pos
        pos[2] = -self.boom_pos
        pos[3] = self.body_yaw
        # Wheel angles use NaN (velocity control)
        pos[4] = float('nan')
        pos[5] = float('nan')
        pos[6] = float('nan')
        pos[7] = float('nan')
        
        # Update velocities
        vel[0] = 0.0  # bucket_linear
        vel[1] = 0.0  # arm_linear
        vel[2] = 0.0  # boom_linear
        vel[3] = 0.0  # body_rotate
        vel[4] = self.left_track_velocity   # front_left_wheel_angle
        vel[5] = self.left_track_velocity   # back_left_wheel_angle
        vel[6] = self.right_track_velocity  # front_right_wheel_angle
        vel[7] = self.right_track_velocity  # back_right_wheel_angle

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

