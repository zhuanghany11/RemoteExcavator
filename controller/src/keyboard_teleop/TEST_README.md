# Teleop Prismatic Publisher 测试指南

## 概述

本目录包含远程控制挖掘机的节点及其测试脚本。

## 文件说明

### 主程序
- **`teleop_prismatic_publisher.py`** - 主节点，订阅 `/controls/teleop` topic，控制挖掘机的 bucket、arm、boom 和 body_rotate

### 测试脚本
- **`test_teleop_control.py`** - 自动化测试脚本，运行完整的测试套件
- **`test_teleop_manual.py`** - 交互式手动测试脚本，通过菜单控制

## 使用方法

### 1. 编译包（如果需要）

```bash
cd /home/cyber007/code_ws
colcon build --packages-select keyboard_teleop
source install/setup.bash
```

### 2. 运行主节点

在终端 1 中运行：

```bash
ros2 run keyboard_teleop teleop_prismatic_publisher.py
```

这个节点会：
- 订阅 `/controls/teleop` topic（JSON 格式控制命令）
- 发布 `/pc2000_joint_command` topic（JointState 消息）
- 将接收到的控制命令转换为挖掘机关节命令

### 3. 运行测试

#### 选项 A: 自动化测试

在终端 2 中运行完整的自动化测试：

```bash
ros2 run keyboard_teleop test_teleop_control.py
```

这个脚本会自动运行以下测试：
1. **单独控制测试** - 测试每个控制（bucket、stick、boom、swing）的正负方向
2. **组合控制测试** - 测试多个控制同时工作
3. **极限值测试** - 测试最大值、最小值和归零
4. **快速变化测试** - 测试快速交替的控制命令
5. **渐进值测试** - 测试从 0.0 到 1.0 的渐进变化

#### 选项 B: 交互式手动测试

在终端 2 中运行交互式测试：

```bash
ros2 run keyboard_teleop test_teleop_manual.py
```

这个脚本提供一个菜单，允许你：
1. 测试单个控制（bucket、stick、boom、swing）
2. 测试组合控制
3. 发送自定义值
4. 归零所有控制
5. 运行连续振荡测试

### 4. 监控输出

#### 监控 teleop 输入

```bash
ros2 topic echo /controls/teleop
```

#### 监控关节命令输出

```bash
ros2 topic echo /pc2000_joint_command
```

## 控制映射

| 输入 (teleop) | 范围 | 输出 (joint) | 说明 |
|--------------|------|-------------|------|
| `bucket` | -1.0 ~ 1.0 | `bucket_linear` | 铲斗线性运动（取反） |
| `stick` | -1.0 ~ 1.0 | `arm_linear` | 臂线性运动（取反） |
| `boom` | -1.0 ~ 1.0 | `boom_linear` | 动臂线性运动（取反） |
| `swing` 或 `rotation` | -1.0 ~ 1.0 | `body_rotate` | 车身旋转 |

**注意**：位置值会被取反（与 `keyboard_prismatic_publisher.py` 保持一致）

## JSON 消息格式

发送到 `/controls/teleop` 的消息格式：

```json
{
  "bucket": 0.5,
  "stick": 0.3,
  "boom": -0.2,
  "swing": 0.1,
  "device_type": "excavator",
  "timestamp": 1699363200000
}
```

## 示例：使用命令行发送测试命令

```bash
# 测试 bucket 控制
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.5, \"stick\": 0.0, \"boom\": 0.0, \"swing\": 0.0}"'

# 测试组合控制
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.5, \"stick\": 0.3, \"boom\": 0.2, \"swing\": 0.1}"'

# 归零
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.0, \"stick\": 0.0, \"boom\": 0.0, \"swing\": 0.0}"'
```

## 参数配置

可以通过 ROS2 参数覆盖默认配置：

```bash
ros2 run keyboard_teleop teleop_prismatic_publisher.py \
  --ros-args \
  -p topic:=/my_custom_topic \
  -p joint_bucket:=my_bucket \
  -p joint_arm:=my_arm \
  -p joint_boom:=my_boom \
  -p joint_body_rotate:=my_rotate
```

## 故障排除

### 问题：节点运行但没有输出

**解决方案**：
1. 检查 topic 连接：`ros2 topic list`
2. 确认有消息发送：`ros2 topic echo /controls/teleop`
3. 检查节点日志

### 问题：控制响应不正常

**解决方案**：
1. 检查输入值范围是否在 -1.0 ~ 1.0
2. 检查 JSON 格式是否正确
3. 查看节点日志中的错误信息

### 问题：测试脚本无法运行

**解决方案**：
1. 确认已编译：`colcon build --packages-select keyboard_teleop`
2. 确认已 source：`source install/setup.bash`
3. 检查文件权限：`chmod +x src/*.py`

## 开发和调试

### 查看节点信息

```bash
ros2 node info /teleop_prismatic_publisher
```

### 查看 topic 信息

```bash
ros2 topic info /controls/teleop
ros2 topic info /pc2000_joint_command
```

### 记录数据用于回放

```bash
ros2 bag record /controls/teleop /pc2000_joint_command
```

### 回放数据

```bash
ros2 bag play <bag_file>
```

## 注意事项

1. 主节点 `teleop_prismatic_publisher.py` 必须先启动
2. 测试脚本会等待 1-2 秒让订阅者连接
3. 位置限制：线性运动 ±0.5m，旋转 ±π rad
4. 每个控制循环周期为 0.1 秒（10 Hz）
5. 输入值会自动钳制到有效范围 [-1.0, 1.0]

## 联系

如有问题或建议，请联系维护者。

