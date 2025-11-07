#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
import json
import time


class TeleopControlTester(Node):
    def __init__(self):
        super().__init__('teleop_control_tester')
        
        # Create publisher for teleop control
        self.teleop_pub = self.create_publisher(StringMsg, '/controls/teleop', 10)
        
        self.get_logger().info('Teleop Control Tester started')
        self.get_logger().info('Publishing test commands to /controls/teleop')
        self.get_logger().info('='*60)
        
    def publish_control(self, control_dict):
        """发布控制命令到 /controls/teleop"""
        msg = StringMsg()
        msg.data = json.dumps(control_dict)
        self.teleop_pub.publish(msg)
        self.get_logger().info(f'Published: {json.dumps(control_dict, indent=2)}')
        
    def test_individual_controls(self):
        """测试各个单独的控制"""
        self.get_logger().info('\n--- Test 1: Individual Controls ---')
        
        # 测试 bucket 控制
        self.get_logger().info('\n>> Testing bucket control (positive)')
        self.publish_control({
            'bucket': 0.5,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 bucket 控制（负方向）
        self.get_logger().info('\n>> Testing bucket control (negative)')
        self.publish_control({
            'bucket': -0.5,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 stick/arm 控制
        self.get_logger().info('\n>> Testing stick/arm control (positive)')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.5,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 stick/arm 控制（负方向）
        self.get_logger().info('\n>> Testing stick/arm control (negative)')
        self.publish_control({
            'bucket': 0.0,
            'stick': -0.5,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 boom 控制
        self.get_logger().info('\n>> Testing boom control (positive)')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.5,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 boom 控制（负方向）
        self.get_logger().info('\n>> Testing boom control (negative)')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': -0.5,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 swing 控制
        self.get_logger().info('\n>> Testing swing control (positive)')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.5,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试 swing 控制（负方向）
        self.get_logger().info('\n>> Testing swing control (negative)')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': -0.5,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
    def test_combined_controls(self):
        """测试组合控制"""
        self.get_logger().info('\n--- Test 2: Combined Controls ---')
        
        # 测试组合动作 1：bucket + boom
        self.get_logger().info('\n>> Testing combined: bucket + boom')
        self.publish_control({
            'bucket': 0.5,
            'stick': 0.0,
            'boom': 0.5,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试组合动作 2：stick + swing
        self.get_logger().info('\n>> Testing combined: stick + swing')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.5,
            'boom': 0.0,
            'swing': 0.5,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试组合动作 3：所有控制
        self.get_logger().info('\n>> Testing combined: all controls')
        self.publish_control({
            'bucket': 0.3,
            'stick': 0.3,
            'boom': 0.3,
            'swing': 0.3,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
    def test_extreme_values(self):
        """测试极限值"""
        self.get_logger().info('\n--- Test 3: Extreme Values ---')
        
        # 测试最大值
        self.get_logger().info('\n>> Testing maximum values (1.0)')
        self.publish_control({
            'bucket': 1.0,
            'stick': 1.0,
            'boom': 1.0,
            'swing': 1.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试最小值
        self.get_logger().info('\n>> Testing minimum values (-1.0)')
        self.publish_control({
            'bucket': -1.0,
            'stick': -1.0,
            'boom': -1.0,
            'swing': -1.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
        # 测试归零
        self.get_logger().info('\n>> Testing reset to zero')
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(2)
        
    def test_rapid_changes(self):
        """测试快速变化"""
        self.get_logger().info('\n--- Test 4: Rapid Changes ---')
        
        self.get_logger().info('\n>> Testing rapid bucket movement')
        for i in range(5):
            value = 0.5 if i % 2 == 0 else -0.5
            self.publish_control({
                'bucket': value,
                'stick': 0.0,
                'boom': 0.0,
                'swing': 0.0,
                'device_type': 'excavator',
                'timestamp': int(time.time() * 1000)
            })
            time.sleep(0.5)
        
        # 归零
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(1)
        
    def test_incremental_values(self):
        """测试渐进值"""
        self.get_logger().info('\n--- Test 5: Incremental Values ---')
        
        self.get_logger().info('\n>> Testing incremental bucket values (0.0 to 1.0)')
        for i in range(11):
            value = i * 0.1
            self.publish_control({
                'bucket': value,
                'stick': 0.0,
                'boom': 0.0,
                'swing': 0.0,
                'device_type': 'excavator',
                'timestamp': int(time.time() * 1000)
            })
            time.sleep(0.3)
        
        # 归零
        self.publish_control({
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': int(time.time() * 1000)
        })
        time.sleep(1)
        
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Starting Teleop Control Tests')
        self.get_logger().info('='*60)
        
        # 等待订阅者连接
        self.get_logger().info('\nWaiting for subscribers to connect...')
        time.sleep(2)
        
        # 运行测试
        self.test_individual_controls()
        self.test_combined_controls()
        self.test_extreme_values()
        self.test_rapid_changes()
        self.test_incremental_values()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('All tests completed!')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopControlTester()
    
    try:
        # 运行所有测试
        node.run_all_tests()
        
        # 保持节点运行一小段时间以确保所有消息都被发送
        time.sleep(1)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

