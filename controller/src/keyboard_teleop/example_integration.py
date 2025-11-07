#!/usr/bin/env python3
"""
示例：如何在你的程序中集成 teleop 控制

这个文件展示了如何创建一个自定义节点来发送控制命令到
teleop_prismatic_publisher。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
import json
import time


class ExcavatorController(Node):
    """
    示例控制器类
    
    展示如何在实际应用中集成 teleop 控制功能。
    """
    
    def __init__(self):
        super().__init__('excavator_controller')
        
        # 创建 teleop 控制发布者
        self.teleop_pub = self.create_publisher(StringMsg, '/controls/teleop', 10)
        
        # 当前控制状态
        self.current_controls = {
            'bucket': 0.0,
            'stick': 0.0,
            'boom': 0.0,
            'swing': 0.0,
            'device_type': 'excavator',
            'timestamp': 0
        }
        
        self.get_logger().info('Excavator Controller initialized')
    
    def send_control(self, bucket=None, stick=None, boom=None, swing=None):
        """
        发送控制命令
        
        参数:
            bucket: 铲斗控制 [-1.0, 1.0]
            stick: 臂控制 [-1.0, 1.0]
            boom: 动臂控制 [-1.0, 1.0]
            swing: 旋转控制 [-1.0, 1.0]
        """
        # 更新指定的控制值
        if bucket is not None:
            self.current_controls['bucket'] = max(-1.0, min(1.0, bucket))
        if stick is not None:
            self.current_controls['stick'] = max(-1.0, min(1.0, stick))
        if boom is not None:
            self.current_controls['boom'] = max(-1.0, min(1.0, boom))
        if swing is not None:
            self.current_controls['swing'] = max(-1.0, min(1.0, swing))
        
        # 更新时间戳
        self.current_controls['timestamp'] = int(time.time() * 1000)
        
        # 发布消息
        msg = StringMsg()
        msg.data = json.dumps(self.current_controls)
        self.teleop_pub.publish(msg)
        
        self.get_logger().info(
            f"Control sent - bucket: {self.current_controls['bucket']:.2f}, "
            f"stick: {self.current_controls['stick']:.2f}, "
            f"boom: {self.current_controls['boom']:.2f}, "
            f"swing: {self.current_controls['swing']:.2f}"
        )
    
    def reset_controls(self):
        """归零所有控制"""
        self.send_control(bucket=0.0, stick=0.0, boom=0.0, swing=0.0)
        self.get_logger().info('All controls reset to zero')
    
    def execute_dig_sequence(self):
        """
        示例：执行挖掘序列
        
        这是一个示例，展示如何组合多个动作。
        """
        self.get_logger().info('Starting dig sequence...')
        
        # 1. 降下动臂
        self.get_logger().info('Step 1: Lowering boom')
        self.send_control(boom=-0.5)
        time.sleep(2)
        
        # 2. 伸出臂
        self.get_logger().info('Step 2: Extending stick')
        self.send_control(boom=0.0, stick=0.5)
        time.sleep(2)
        
        # 3. 闭合铲斗（挖掘）
        self.get_logger().info('Step 3: Closing bucket (digging)')
        self.send_control(stick=0.0, bucket=0.5)
        time.sleep(2)
        
        # 4. 收回臂
        self.get_logger().info('Step 4: Retracting stick')
        self.send_control(bucket=0.0, stick=-0.5)
        time.sleep(2)
        
        # 5. 抬起动臂
        self.get_logger().info('Step 5: Raising boom')
        self.send_control(stick=0.0, boom=0.5)
        time.sleep(2)
        
        # 6. 旋转到卸载位置
        self.get_logger().info('Step 6: Swinging to dump position')
        self.send_control(boom=0.0, swing=0.5)
        time.sleep(2)
        
        # 7. 打开铲斗（卸载）
        self.get_logger().info('Step 7: Opening bucket (dumping)')
        self.send_control(swing=0.0, bucket=-0.5)
        time.sleep(2)
        
        # 8. 归零
        self.get_logger().info('Step 8: Resetting to neutral')
        self.reset_controls()
        
        self.get_logger().info('Dig sequence completed!')
    
    def example_gamepad_callback(self, axes, buttons):
        """
        示例：如何处理游戏手柄输入
        
        参数:
            axes: 游戏手柄轴值列表 (例如 [left_x, left_y, right_x, right_y])
            buttons: 游戏手柄按钮状态列表
        """
        # 假设轴映射：
        # axes[0] = 左摇杆 X (swing)
        # axes[1] = 左摇杆 Y (boom)
        # axes[2] = 右摇杆 X (bucket)
        # axes[3] = 右摇杆 Y (stick)
        
        self.send_control(
            swing=axes[0],
            boom=axes[1],
            bucket=axes[2],
            stick=axes[3]
        )
    
    def example_keyboard_mapping(self, key_states):
        """
        示例：如何处理键盘输入
        
        参数:
            key_states: 字典，包含按键状态 {'w': True, 's': False, ...}
        """
        bucket = 0.0
        stick = 0.0
        boom = 0.0
        swing = 0.0
        
        # Bucket 控制 (U/J)
        if key_states.get('u', False):
            bucket = 0.5
        elif key_states.get('j', False):
            bucket = -0.5
        
        # Stick 控制 (I/K)
        if key_states.get('i', False):
            stick = 0.5
        elif key_states.get('k', False):
            stick = -0.5
        
        # Boom 控制 (O/L)
        if key_states.get('o', False):
            boom = 0.5
        elif key_states.get('l', False):
            boom = -0.5
        
        # Swing 控制 (A/D)
        if key_states.get('a', False):
            swing = 0.5
        elif key_states.get('d', False):
            swing = -0.5
        
        self.send_control(bucket=bucket, stick=stick, boom=boom, swing=swing)


def example_1_simple_control():
    """示例 1: 简单的控制命令"""
    print('\n=== Example 1: Simple Control ===')
    
    rclpy.init()
    controller = ExcavatorController()
    
    try:
        # 等待连接
        time.sleep(1)
        
        # 发送简单的控制命令
        controller.send_control(bucket=0.5)
        time.sleep(2)
        
        controller.send_control(boom=0.3)
        time.sleep(2)
        
        # 归零
        controller.reset_controls()
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()


def example_2_sequence():
    """示例 2: 执行动作序列"""
    print('\n=== Example 2: Dig Sequence ===')
    
    rclpy.init()
    controller = ExcavatorController()
    
    try:
        # 等待连接
        time.sleep(1)
        
        # 执行挖掘序列
        controller.execute_dig_sequence()
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()


def example_3_interactive():
    """示例 3: 交互式控制"""
    print('\n=== Example 3: Interactive Control ===')
    print('使用数字键控制：')
    print('1 - Bucket +')
    print('2 - Bucket -')
    print('3 - Stick +')
    print('4 - Stick -')
    print('5 - Boom +')
    print('6 - Boom -')
    print('7 - Swing +')
    print('8 - Swing -')
    print('0 - Reset')
    print('q - Quit')
    
    rclpy.init()
    controller = ExcavatorController()
    
    try:
        # 等待连接
        time.sleep(1)
        
        while True:
            key = input('\n输入命令: ').strip()
            
            if key == 'q':
                break
            elif key == '1':
                controller.send_control(bucket=0.5)
            elif key == '2':
                controller.send_control(bucket=-0.5)
            elif key == '3':
                controller.send_control(stick=0.5)
            elif key == '4':
                controller.send_control(stick=-0.5)
            elif key == '5':
                controller.send_control(boom=0.5)
            elif key == '6':
                controller.send_control(boom=-0.5)
            elif key == '7':
                controller.send_control(swing=0.5)
            elif key == '8':
                controller.send_control(swing=-0.5)
            elif key == '0':
                controller.reset_controls()
            else:
                print('无效命令')
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print('\n退出...')
    finally:
        controller.reset_controls()
        controller.destroy_node()
        rclpy.shutdown()


def main():
    """主函数 - 选择要运行的示例"""
    print('\n' + '='*60)
    print('Excavator Controller Integration Examples')
    print('='*60)
    print('\n选择要运行的示例:')
    print('1. 简单控制命令')
    print('2. 执行挖掘序列')
    print('3. 交互式控制')
    print('0. 退出')
    
    choice = input('\n输入选项: ').strip()
    
    if choice == '1':
        example_1_simple_control()
    elif choice == '2':
        example_2_sequence()
    elif choice == '3':
        example_3_interactive()
    elif choice == '0':
        print('退出')
    else:
        print('无效选项')


if __name__ == '__main__':
    main()

