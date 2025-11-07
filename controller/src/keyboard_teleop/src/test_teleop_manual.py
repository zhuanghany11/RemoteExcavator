#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
import json
import time
import sys


class TeleopManualTester(Node):
    def __init__(self):
        super().__init__('teleop_manual_tester')
        
        # Create publisher for teleop control
        self.teleop_pub = self.create_publisher(StringMsg, '/controls/teleop', 10)
        
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
    print('1. Test bucket control')
    print('2. Test stick/arm control')
    print('3. Test boom control')
    print('4. Test swing/body rotation control')
    print('5. Test combined control (bucket + boom)')
    print('6. Test combined control (stick + swing)')
    print('7. Send custom values')
    print('8. Reset all to zero')
    print('9. Run continuous oscillation test')
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
                
            elif choice == '1':
                # Test bucket
                value = get_float_input('Enter bucket value [-1.0 to 1.0] (default 0.5): ', 0.5)
                value = max(-1.0, min(1.0, value))
                node.publish_control({
                    'bucket': value,
                    'stick': 0.0,
                    'boom': 0.0,
                    'swing': 0.0,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '2':
                # Test stick
                value = get_float_input('Enter stick value [-1.0 to 1.0] (default 0.5): ', 0.5)
                value = max(-1.0, min(1.0, value))
                node.publish_control({
                    'bucket': 0.0,
                    'stick': value,
                    'boom': 0.0,
                    'swing': 0.0,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '3':
                # Test boom
                value = get_float_input('Enter boom value [-1.0 to 1.0] (default 0.5): ', 0.5)
                value = max(-1.0, min(1.0, value))
                node.publish_control({
                    'bucket': 0.0,
                    'stick': 0.0,
                    'boom': value,
                    'swing': 0.0,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '4':
                # Test swing
                value = get_float_input('Enter swing value [-1.0 to 1.0] (default 0.5): ', 0.5)
                value = max(-1.0, min(1.0, value))
                node.publish_control({
                    'bucket': 0.0,
                    'stick': 0.0,
                    'boom': 0.0,
                    'swing': value,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '5':
                # Combined bucket + boom
                bucket = get_float_input('Enter bucket value [-1.0 to 1.0] (default 0.5): ', 0.5)
                boom = get_float_input('Enter boom value [-1.0 to 1.0] (default 0.5): ', 0.5)
                bucket = max(-1.0, min(1.0, bucket))
                boom = max(-1.0, min(1.0, boom))
                node.publish_control({
                    'bucket': bucket,
                    'stick': 0.0,
                    'boom': boom,
                    'swing': 0.0,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '6':
                # Combined stick + swing
                stick = get_float_input('Enter stick value [-1.0 to 1.0] (default 0.5): ', 0.5)
                swing = get_float_input('Enter swing value [-1.0 to 1.0] (default 0.5): ', 0.5)
                stick = max(-1.0, min(1.0, stick))
                swing = max(-1.0, min(1.0, swing))
                node.publish_control({
                    'bucket': 0.0,
                    'stick': stick,
                    'boom': 0.0,
                    'swing': swing,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '7':
                # Custom values
                print('\nEnter custom values for all controls:')
                bucket = get_float_input('  Bucket [-1.0 to 1.0]: ', 0.0)
                stick = get_float_input('  Stick [-1.0 to 1.0]: ', 0.0)
                boom = get_float_input('  Boom [-1.0 to 1.0]: ', 0.0)
                swing = get_float_input('  Swing [-1.0 to 1.0]: ', 0.0)
                
                bucket = max(-1.0, min(1.0, bucket))
                stick = max(-1.0, min(1.0, stick))
                boom = max(-1.0, min(1.0, boom))
                swing = max(-1.0, min(1.0, swing))
                
                node.publish_control({
                    'bucket': bucket,
                    'stick': stick,
                    'boom': boom,
                    'swing': swing,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '8':
                # Reset to zero
                print('Resetting all controls to zero...')
                node.publish_control({
                    'bucket': 0.0,
                    'stick': 0.0,
                    'boom': 0.0,
                    'swing': 0.0,
                    'device_type': 'excavator',
                    'timestamp': int(time.time() * 1000)
                })
                
            elif choice == '9':
                # Continuous oscillation test
                print('\nRunning continuous oscillation test...')
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
                        
                        node.publish_control({
                            'bucket': bucket,
                            'stick': stick,
                            'boom': boom,
                            'swing': swing,
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
                        'device_type': 'excavator',
                        'timestamp': int(time.time() * 1000)
                    })
                
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

