#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String as StringMsg
import json
import time
import sys


class TeleopManualTester(Node):
    def __init__(self):
        super().__init__('teleop_manual_tester')
        
        # Create publisher for teleop control (match subscriber QoS: BEST_EFFORT)
        teleop_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.teleop_pub = self.create_publisher(StringMsg, '/controls/teleop', teleop_qos)
        
        self.get_logger().info('Teleop Manual Tester started')
        self.get_logger().info('Publishing commands to /controls/teleop')
        
    def publish_control(self, control_dict):
        """发布控制命令到 /controls/teleop"""
        msg = StringMsg()
        msg.data = json.dumps(control_dict)
        self.teleop_pub.publish(msg)
        self.get_logger().info(f'Published: {json.dumps(control_dict)}')
        

def print_menu():
    """打印菜单"""
    print('\n' + '='*60)
    print('Teleop Manual Control Tester')
    print('='*60)
    print('Hint: per-axis steps assumed in receiver:')
    print('  bucket=0.03 m/tick, arm=0.02 m/tick, boom=0.02 m/tick, yaw=0.04 rad/tick')
    print('1. Run automatic digging cycle')
    print('2. Run continuous all-actuators motion')
    print('3. Run track mapping test (forward/back/turn sequence)')
    print('0. Exit')
    print('='*60)


def get_float_input(prompt, default=0.0):
    """获取浮点数输入"""
    try:
        value = input(prompt)
        if value.strip() == '':
            return default
        return float(value)
    except ValueError:
        print(f'Invalid input, using default: {default}')
        return default


def main(args=None):
    rclpy.init(args=args)
    node = TeleopManualTester()
    
    print('\nWaiting for subscribers to connect...')
    time.sleep(1)
    
    try:
        while True:
            print_menu()
            choice = input('\nEnter your choice: ')
            
            if choice == '0':
                print('Exiting...')
                break
                
            elif choice == '2':
                # Continuous oscillation test
                print('\nRunning continuous oscillation test...')
                print('Using per-axis steps: bucket=0.03, arm=0.02, boom=0.02, yaw=0.04')
                print('Press Ctrl+C to stop')
                try:
                    import math
                    start_time = time.time()
                    while True:
                        elapsed = time.time() - start_time
                        bucket = 0.5 * math.sin(elapsed * 1.0)
                        stick = 0.5 * math.cos(elapsed * 1.2)
                        boom = 0.5 * math.sin(elapsed * 0.8)
                        swing = 0.3 * math.cos(elapsed * 0.5)
                        # Tracks: alternate left/right turn
                        left_track = 0.5 * math.sin(elapsed * 0.6)
                        right_track = -0.5 * math.sin(elapsed * 0.6)
                        
                        node.publish_control({
                            'bucket': bucket,
                            'stick': stick,
                            'boom': boom,
                            'swing': swing,
                            'left_track': left_track,
                            'right_track': right_track,
                            'device_type': 'excavator',
                            'timestamp': int(time.time() * 1000)
                        })
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print('\nOscillation test stopped')
                    # Reset to zero
                    node.publish_control({
                        'bucket': 0.0,
                        'stick': 0.0,
                        'boom': 0.0,
                        'swing': 0.0,
                        'left_track': 0.0,
                        'right_track': 0.0,
                        'device_type': 'excavator',
                        'timestamp': int(time.time() * 1000)
                    })
            elif choice == '1':
                # Automatic digging cycle
                print('\nRunning automatic digging cycle...')
                print('This will perform: approach -> dig -> lift -> swing -> dump -> return')
                print('Using per-axis steps: bucket=0.03, arm=0.02, boom=0.02, yaw=0.04')
                
                def hold_control(bucket, stick, boom, swing, duration_s):
                    end_time = time.time() + duration_s
                    while time.time() < end_time:
                        node.publish_control({
                            'bucket': bucket,
                            'stick': stick,
                            'boom': boom,
                            'swing': swing,
                            'device_type': 'excavator',
                            'timestamp': int(time.time() * 1000)
                        })
                        time.sleep(0.1)
                
                try:
                    # 0) 安全起始：停止
                    hold_control(0.0, 0.0, 0.0, 0.0, 0.5)

                    # 1) 靠近料堆：放大臂、伸小臂、开斗（更大幅度，延长时间）
                    hold_control(bucket=-1.0, stick=+1.0, boom=-1.0, swing=0.0, duration_s=4.0)

                    # 2) 切入并铲装：强力收斗、继续下压，微收小臂
                    hold_control(bucket=+1.0, stick=-0.5, boom=-0.8, swing=0.0, duration_s=2.5)

                    # 3) 提升铲斗：大幅抬大臂、强力收小臂
                    hold_control(bucket=+1.0, stick=-1.0, boom=+1.0, swing=0.0, duration_s=3.0)

                    # 4) 回转到卸料位：保持抬臂，快速回转
                    # 注意 teleop 节点对 swing 有符号翻转
                    hold_control(bucket=+0.9, stick=-0.4, boom=+0.9, swing=+0.9, duration_s=3.0)

                    # 5) 倾倒：完全打开铲斗
                    hold_control(bucket=-1.0, stick=0.0, boom=+0.6, swing=0.0, duration_s=1.8)

                    # 6) 回到作业位：回转回去，放臂，斗回中
                    hold_control(bucket=0.0, stick=+0.4, boom=-0.9, swing=-0.9, duration_s=3.0)

                    # 7) 稳定
                    hold_control(0.0, 0.0, 0.0, 0.0, 0.8)

                    print('Automatic digging cycle finished.')
                except KeyboardInterrupt:
                    print('\nAutomatic cycle interrupted')
			
            elif choice == '3':
                # Track mapping test: forward -> stop -> back -> stop -> left pivot -> right pivot -> gentle curves
                print('\nRunning track mapping test (forward/back/turn sequence)...')
                def hold_tracks(left, right, duration_s):
                    end_time = time.time() + duration_s
                    while time.time() < end_time:
                        node.publish_control({
                            'bucket': 0.0,
                            'stick': 0.0,
                            'boom': 0.0,
                            'swing': 0.0,
                            'left_track': left,
                            'right_track': right,
                            'device_type': 'excavator',
                            'timestamp': int(time.time() * 1000)
                        })
                        time.sleep(0.1)
                try:
                    # forward
                    print(' - Forward')
                    hold_tracks(+0.8, +0.8, 3.0)
                    # stop
                    hold_tracks(0.0, 0.0, 1.0)
                    # backward
                    print(' - Backward')
                    hold_tracks(-0.8, -0.8, 3.0)
                    # stop
                    hold_tracks(0.0, 0.0, 1.0)
                    # left pivot (on spot)
                    print(' - Left pivot')
                    hold_tracks(-0.8, +0.8, 2.5)
                    # right pivot (on spot)
                    print(' - Right pivot')
                    hold_tracks(+0.8, -0.8, 2.5)
                    # gentle left curve
                    print(' - Gentle left curve')
                    hold_tracks(+0.6, +0.2, 3.0)
                    # gentle right curve
                    print(' - Gentle right curve')
                    hold_tracks(+0.2, +0.6, 3.0)
                    # stop
                    hold_tracks(0.0, 0.0, 1.0)
                    print('Track mapping test finished.')
                except KeyboardInterrupt:
                    print('\nTrack mapping test interrupted')
                
            else:
                print('Invalid choice, please try again.')
                
            # 等待一小段时间确保消息发送
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print('\nExiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

