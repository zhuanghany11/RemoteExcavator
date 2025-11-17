#!/usr/bin/env python3
"""
高级遥操作控制节点 - 带校准、死区、限幅和非线性响应曲线
实现功能：
1. 校准和归一化 - 处理手柄中位偏移
2. 死区（Deadzone）- 中间 ±3%～±20% 不响应
3. 限幅和斜坡（Rate Limit）- 每20ms最大变化5%
4. 非线性曲线 - 低段细、末端陡，便于微动操作
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
import math
import json
import time


class TeleopAdvancedControl(Node):
    def __init__(self):
        super().__init__('teleop_advanced_control')

        # ROS参数配置
        self.declare_parameter('topic', '/pc2000_joint_command')
        self.declare_parameter('joint_bucket', 'bucket_linear')
        self.declare_parameter('joint_arm', 'arm_linear')
        self.declare_parameter('joint_boom', 'boom_linear')
        self.declare_parameter('joint_body_rotate', 'body_rotate')
        self.declare_parameter('track_velocity_scale', 10.0)

        # 死区配置（可配置范围：3%～20%）
        self.declare_parameter('deadzone_min', 0.03)  # 最小死区 3%
        self.declare_parameter('deadzone_max', 0.20)  # 最大死区 20%
        self.declare_parameter('deadzone_default', 0.10)  # 默认死区 10%

        # 限幅配置
        self.declare_parameter('rate_limit_percent', 0.05)  # 每周期最大变化 5%
        self.declare_parameter('control_period_ms', 20)  # 控制周期 20ms

        # 非线性曲线配置
        self.declare_parameter('response_curve_exponent', 2.0)  # 响应曲线指数（2.0 = 平方曲线）

        # 校准配置
        self.declare_parameter('enable_calibration', True)  # 是否启用校准
        self.declare_parameter('calibration_samples', 100)  # 校准采样次数

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

        # 发布器
        self.joint_pub = self.create_publisher(JointState, topic, 10)

        # QoS配置
        teleop_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅遥操作控制话题
        self.teleop_sub = self.create_subscription(
            StringMsg,
            '/controls/teleop',
            self.teleop_callback,
            teleop_qos
        )

        # 订阅关节状态（用于获取当前位置）
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 创建校准服务
        self.calibration_start_srv = self.create_service(
            Trigger,
            '/teleop_advanced/start_calibration',
            self.start_calibration_service
        )
        self.calibration_stop_srv = self.create_service(
            Trigger,
            '/teleop_advanced/stop_calibration',
            self.stop_calibration_service
        )

        # 关节位置状态
        self.bucket_pos = 0.0
        self.arm_pos = 0.0
        self.boom_pos = 1.0 * 2.0 / 3.0  # 初始化为2/3高度
        self.body_yaw = 0.0
        self.left_track_velocity = 0.0
        self.right_track_velocity = 0.0

        # 控制参数
        self.lin_limit = 1.0  # 米
        self.boom_limit = 1.0  # 米
        self.yaw_limit = math.pi  # 弧度
        self.track_velocity_scale = float(
            self.get_parameter('track_velocity_scale').get_parameter_value().double_value
        )

        # 步进大小（每周期）
        self.step_bucket_linear = 0.01
        self.step_arm_linear = 0.02
        self.step_boom_linear = 0.008
        self.step_body_yaw = 0.01

        # 死区参数
        self.deadzone_min = float(
            self.get_parameter('deadzone_min').get_parameter_value().double_value
        )
        self.deadzone_max = float(
            self.get_parameter('deadzone_max').get_parameter_value().double_value
        )
        self.deadzone_default = float(
            self.get_parameter('deadzone_default').get_parameter_value().double_value
        )
        # 为每个轴单独配置死区（可根据需要调整）
        self.deadzones = {
            'bucket': self.deadzone_default,
            'stick': self.deadzone_default,
            'boom': self.deadzone_default,
            'swing': self.deadzone_default,
            'rotation': self.deadzone_default,
            'left_track': self.deadzone_default,
            'right_track': self.deadzone_default,
        }

        # 限幅参数
        self.rate_limit_percent = float(
            self.get_parameter('rate_limit_percent').get_parameter_value().double_value
        )
        self.control_period_ms = int(
            self.get_parameter('control_period_ms').get_parameter_value().integer_value
        )
        self.control_period_sec = self.control_period_ms / 1000.0

        # 非线性曲线参数
        self.response_curve_exponent = float(
            self.get_parameter('response_curve_exponent').get_parameter_value().double_value
        )

        # 校准相关
        self.enable_calibration = bool(
            self.get_parameter('enable_calibration').get_parameter_value().bool_value
        )
        self.calibration_samples = int(
            self.get_parameter('calibration_samples').get_parameter_value().integer_value
        )

        # 校准数据：存储每个轴的中位偏移值
        self.calibration_offsets = {
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'rotation': 0.0,
            'left_track': 0.0,
            'right_track': 0.0,
        }
        self.calibration_data = {key: [] for key in self.calibration_offsets.keys()}
        self.calibration_active = False
        self.calibration_sample_count = 0

        # 上一次的输出值（用于限幅计算）
        self.last_outputs = {
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'rotation': 0.0,
            'left_track': 0.0,
            'right_track': 0.0,
        }

        # 上一次处理时间（用于限幅计算）
        self.last_process_time = 0.0

        # 最新接收到的控制值
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

        self.initialized = True
        self.output_epsilon = 1e-6

        # 预分配消息对象
        self.joint_msg = JointState()
        self.joint_msg.name = self.joint_names
        self.joint_msg.effort = [0.0] * len(self.joint_names)
        self.joint_msg.position = [0.0] * len(self.joint_names)
        self.joint_msg.velocity = [0.0] * len(self.joint_names)

        # 定时器：按控制周期更新
        timer_period = self.control_period_sec
        self.create_timer(timer_period, self.timer_callback)

        # 初始位置发布
        self.initial_publish_done = False
        self.create_timer(0.5, self.publish_initial_positions)

        self.get_logger().info('高级遥操作控制节点已启动')
        self.get_logger().info(f'死区范围: {self.deadzone_min:.1%} ~ {self.deadzone_max:.1%} (默认: {self.deadzone_default:.1%})')
        self.get_logger().info(f'限幅速率: {self.rate_limit_percent:.1%} 每 {self.control_period_ms}ms')
        self.get_logger().info(f'响应曲线指数: {self.response_curve_exponent:.2f}')
        self.get_logger().info(f'校准功能: {"启用" if self.enable_calibration else "禁用"}')
        self.get_logger().info('校准服务: /teleop_advanced/start_calibration, /teleop_advanced/stop_calibration')

    def publish_initial_positions(self):
        """发布初始位置"""
        if not self.initial_publish_done:
            self.publish_joint_state()
            self.initial_publish_done = True
            self.get_logger().info('已发布初始位置')

    def teleop_callback(self, msg: StringMsg):
        """遥操作回调函数"""
        try:
            data = json.loads(msg.data)

            # 更新控制值
            controls = self.latest_controls
            epsilon = self.output_epsilon
            changed_flag = False

            # 更新各个控制轴
            for key in ['bucket', 'stick', 'boom', 'swing', 'rotation', 'left_track', 'right_track']:
                val = data.get(key)
                if val is not None:
                    new_val = max(-1.0, min(1.0, float(val)))
                    if abs(controls.get(key, 0.0) - new_val) > epsilon:
                        controls[key] = new_val
                        changed_flag = True

                        # 如果正在校准，收集数据
                        if self.calibration_active and self.enable_calibration:
                            self.calibration_data[key].append(new_val)
                            if len(self.calibration_data[key]) > self.calibration_samples:
                                self.calibration_data[key].pop(0)

            # 更新其他字段
            for key in ['brake', 'throttle']:
                val = data.get(key)
                if val is not None:
                    controls[key] = max(0.0, min(1.0, float(val)))

            for key in ['gear', 'device_type']:
                if key in data:
                    controls[key] = str(data[key])

            if 'timestamp' in data:
                controls['timestamp'] = int(data['timestamp'])

            # 如果值发生变化，立即更新
            if changed_flag:
                self.update_positions()
                self.publish_joint_state()

        except (ValueError, TypeError, KeyError) as e:
            self.get_logger().warn(f'解析遥操作JSON失败: {e}')

    def joint_state_callback(self, msg: JointState):
        """关节状态回调（用于同步当前位置）"""
        if not self.initialized:
            try:
                for i, joint_name in enumerate(msg.name):
                    if joint_name == 'bucket_linear':
                        self.bucket_pos = -msg.position[i]
                    elif joint_name == 'arm_linear':
                        self.arm_pos = -msg.position[i]
                    elif joint_name == 'boom_linear':
                        self.boom_pos = -msg.position[i]
                    elif joint_name == 'body_rotate':
                        self.body_yaw = msg.position[i]
                self.initialized = True
            except Exception as e:
                self.get_logger().warn(f'同步关节状态失败: {e}')

    def calibrate_center(self, axis_name):
        """校准指定轴的中位偏移"""
        if not self.enable_calibration or axis_name not in self.calibration_data:
            return

        samples = self.calibration_data[axis_name]
        if len(samples) < self.calibration_samples:
            return

        # 计算中位值（平均值）
        center = sum(samples) / len(samples)
        self.calibration_offsets[axis_name] = center
        self.get_logger().info(f'轴 {axis_name} 校准完成: 中位偏移 = {center:.4f}')

    def start_calibration(self):
        """开始校准流程"""
        if not self.enable_calibration:
            return

        self.calibration_active = True
        self.calibration_sample_count = 0
        for key in self.calibration_data:
            self.calibration_data[key].clear()
        self.get_logger().info(f'开始校准，请保持手柄在中位，采样 {self.calibration_samples} 次...')

    def stop_calibration(self):
        """停止校准并计算偏移"""
        if not self.enable_calibration:
            return

        self.calibration_active = False
        for axis_name in self.calibration_offsets.keys():
            self.calibrate_center(axis_name)
        self.get_logger().info('校准完成')

    def start_calibration_service(self, request, response):
        """启动校准服务回调"""
        if not self.enable_calibration:
            response.success = False
            response.message = "校准功能未启用"
            return response

        self.start_calibration()
        response.success = True
        response.message = f"校准已开始，请保持手柄在中位，将采样 {self.calibration_samples} 次"
        return response

    def stop_calibration_service(self, request, response):
        """停止校准服务回调"""
        if not self.enable_calibration:
            response.success = False
            response.message = "校准功能未启用"
            return response

        self.stop_calibration()
        offsets_str = ", ".join([f"{k}={v:.4f}" for k, v in self.calibration_offsets.items()])
        response.success = True
        response.message = f"校准完成。偏移值: {offsets_str}"
        return response

    def apply_calibration(self, raw_value, axis_name):
        """应用校准：减去中位偏移"""
        if not self.enable_calibration or axis_name not in self.calibration_offsets:
            return raw_value
        return raw_value - self.calibration_offsets[axis_name]

    def apply_deadzone(self, input_val, axis_name='default'):
        """应用死区处理
        Args:
            input_val: 输入值 [-1, 1]
            axis_name: 轴名称，用于获取该轴的死区配置
        Returns:
            处理后的值 [-1, 1]，死区内返回0
        """
        deadzone = self.deadzones.get(axis_name, self.deadzone_default)

        abs_input = abs(input_val)
        if abs_input <= deadzone:
            return 0.0
        else:
            # 线性映射: [deadzone, 1] -> [0, 1]
            sign = 1.0 if input_val > 0 else -1.0
            normalized = (abs_input - deadzone) / (1.0 - deadzone)
            return sign * normalized

    def apply_response_curve(self, input_val):
        """应用非线性响应曲线
        使用幂函数实现：低段细、末端陡
        Args:
            input_val: 输入值 [-1, 1]（已处理死区）
        Returns:
            应用响应曲线后的值 [-1, 1]
        """
        if abs(input_val) < self.output_epsilon:
            return 0.0

        sign = 1.0 if input_val > 0 else -1.0
        abs_val = abs(input_val)

        # 幂函数曲线：y = x^exponent
        # exponent > 1: 低段细、末端陡
        # exponent < 1: 低段陡、末端细
        curved = math.pow(abs_val, self.response_curve_exponent)

        return sign * curved

    def apply_rate_limit(self, desired_value, axis_name):
        """应用限幅和斜坡（Rate Limit）
        限制指令变化速率，避免突然变化
        每20ms最大变化5%（即0.05）
        Args:
            desired_value: 期望输出值 [-1, 1]
            axis_name: 轴名称
        Returns:
            限制后的输出值 [-1, 1]
        """
        last_value = self.last_outputs.get(axis_name, 0.0)
        current_time = time.time()

        # 计算时间差（秒）
        dt = current_time - self.last_process_time
        if dt <= 0 or dt > 1.0:  # 如果时间差异常，使用默认周期
            dt = self.control_period_sec

        # 计算最大允许变化量
        # rate_limit_percent 是每 control_period_ms 的最大变化百分比
        # 例如：0.05 (5%) 每 20ms，意味着每20ms最多变化0.05
        max_delta_per_period = self.rate_limit_percent  # 每个周期的最大变化量
        max_delta = max_delta_per_period * (dt / self.control_period_sec)

        # 限制变化量
        delta = desired_value - last_value
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta

        new_value = last_value + delta
        # 确保输出在有效范围内
        new_value = max(-1.0, min(1.0, new_value))
        self.last_outputs[axis_name] = new_value

        return new_value

    def process_input(self, raw_value, axis_name):
        """完整的输入处理流程
        1. 校准和归一化
        2. 死区
        3. 非线性曲线
        4. 限幅和斜坡
        Args:
            raw_value: 原始输入值 [-1, 1]
            axis_name: 轴名称
        Returns:
            处理后的输出值 [-1, 1]
        """
        # 步骤1: 校准（减去中位偏移）
        calibrated = self.apply_calibration(raw_value, axis_name)

        # 步骤2: 死区处理
        deadzone_processed = self.apply_deadzone(calibrated, axis_name)

        # 步骤3: 非线性响应曲线
        curved = self.apply_response_curve(deadzone_processed)

        # 步骤4: 限幅和斜坡
        rate_limited = self.apply_rate_limit(curved, axis_name)

        return rate_limited

    def update_positions(self):
        """更新关节位置"""
        # 记录当前处理时间（用于限幅计算）
        current_time = time.time()
        if self.last_process_time == 0:
            self.last_process_time = current_time
        
        # 临时保存当前时间，供 apply_rate_limit 使用
        # 在处理完所有轴后，再更新 last_process_time
        process_start_time = current_time

        # 处理各个控制轴
        # Bucket
        bucket_raw = float(self.latest_controls.get('bucket', 0.0))
        bucket_processed = self.process_input(bucket_raw, 'bucket')
        delta_bucket = -bucket_processed * self.step_bucket_linear
        self.bucket_pos = max(-self.lin_limit, min(self.lin_limit, self.bucket_pos + delta_bucket))

        # Stick (Arm)
        stick_raw = float(self.latest_controls.get('stick', 0.0))
        stick_processed = self.process_input(stick_raw, 'stick')
        delta_arm = -stick_processed * self.step_arm_linear
        self.arm_pos = max(-self.lin_limit, min(self.lin_limit, self.arm_pos + delta_arm))

        # Boom
        boom_raw = float(self.latest_controls.get('boom', 0.0))
        boom_processed = self.process_input(boom_raw, 'boom')
        delta_boom = boom_processed * self.step_boom_linear
        self.boom_pos = max(-self.boom_limit, min(self.boom_limit, self.boom_pos + delta_boom))

        # Swing/Rotation (Body Yaw)
        swing_raw = -float(self.latest_controls.get('swing', 0.0))
        rotation_raw = float(self.latest_controls.get('rotation', 0.0))
        swing_processed = self.process_input(swing_raw, 'swing')
        rotation_processed = self.process_input(rotation_raw, 'rotation')
        body_input = swing_processed if abs(swing_processed) > self.output_epsilon else rotation_processed
        delta_yaw = body_input * self.step_body_yaw
        self.body_yaw = max(-self.yaw_limit, min(self.yaw_limit, self.body_yaw + delta_yaw))

        # Tracks
        left_track_raw = float(self.latest_controls.get('left_track', 0.0))
        right_track_raw = float(self.latest_controls.get('right_track', 0.0))
        left_track_processed = self.process_input(left_track_raw, 'left_track')
        right_track_processed = self.process_input(right_track_raw, 'right_track')
        self.left_track_velocity = left_track_processed * self.track_velocity_scale
        self.right_track_velocity = right_track_processed * self.track_velocity_scale

        # 更新处理时间（所有轴处理完成后）
        self.last_process_time = process_start_time

    def timer_callback(self):
        """定时器回调：按控制周期更新"""
        if self.initialized:
            self.update_positions()
            self.publish_joint_state()

    def publish_joint_state(self):
        """发布关节状态"""
        msg = self.joint_msg
        msg.header.stamp = self.get_clock().now().to_msg()

        pos = msg.position
        vel = msg.velocity

        # 更新位置
        pos[0] = -self.bucket_pos
        pos[1] = -self.arm_pos
        pos[2] = -self.boom_pos
        pos[3] = self.body_yaw
        pos[4] = float('nan')
        pos[5] = float('nan')
        pos[6] = float('nan')
        pos[7] = float('nan')

        # 更新速度
        vel[0] = 0.0
        vel[1] = 0.0
        vel[2] = 0.0
        vel[3] = 0.0
        vel[4] = self.left_track_velocity
        vel[5] = self.left_track_velocity
        vel[6] = self.right_track_velocity
        vel[7] = self.right_track_velocity

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopAdvancedControl()

    # 可以通过服务或话题触发校准
    # 这里提供一个简单的示例：启动后等待3秒自动开始校准
    # 实际使用中可以通过ROS服务来控制校准流程

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

