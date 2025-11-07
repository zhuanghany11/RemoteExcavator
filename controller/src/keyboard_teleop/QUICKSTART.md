# 快速开始指南

## 快速测试（推荐）

### 方法 1: 使用启动脚本

```bash
cd /home/cyber007/code_ws/src/RemoteExcavator/controller/src/keyboard_teleop
./test_launcher.sh
```

按照菜单提示操作即可。

### 方法 2: 手动启动

#### 终端 1: 启动主节点

```bash
cd /home/cyber007/code_ws
source install/setup.bash
ros2 run keyboard_teleop teleop_prismatic_publisher.py
```

#### 终端 2: 运行测试（选择一种）

**选项 A - 自动化测试：**
```bash
cd /home/cyber007/code_ws
source install/setup.bash
ros2 run keyboard_teleop test_teleop_control.py
```

**选项 B - 交互式测试：**
```bash
cd /home/cyber007/code_ws
source install/setup.bash
ros2 run keyboard_teleop test_teleop_manual.py
```

## 测试内容

### 自动化测试 (`test_teleop_control.py`)

自动运行以下测试序列：
1. ✓ 单独控制测试（bucket、stick、boom、swing 各方向）
2. ✓ 组合控制测试
3. ✓ 极限值测试
4. ✓ 快速变化测试
5. ✓ 渐进值测试

**预计时间**: ~60 秒

### 交互式测试 (`test_teleop_manual.py`)

提供菜单选项：
- 测试各个单独控制
- 测试组合控制
- 发送自定义值
- 归零所有控制
- 连续振荡测试

## 预期输出

### 主节点输出示例

```
[INFO] [teleop_prismatic_publisher]: Teleop Prismatic Publisher started
[INFO] [teleop_prismatic_publisher]: Topic: /pc2000_joint_command
[INFO] [teleop_prismatic_publisher]: Joint order: bucket_linear, arm_linear, boom_linear, body_rotate
[INFO] [teleop_prismatic_publisher]: Listening to /controls/teleop for control commands
[INFO] [teleop_prismatic_publisher]: bucket: 0.500, arm: 0.000, boom: 0.000, body_yaw: 0.000 (0.0°)
```

### 测试脚本输出示例

```
[INFO] [teleop_control_tester]: Published: {
  "bucket": 0.5,
  "stick": 0.0,
  "boom": 0.0,
  "swing": 0.0,
  "device_type": "excavator",
  "timestamp": 1699363200000
}
```

## 验证测试成功

### 检查 1: Topic 连接

```bash
ros2 topic list | grep -E "(teleop|joint_command)"
```

应该看到：
```
/controls/teleop
/pc2000_joint_command
```

### 检查 2: 消息流动

在主节点运行时，在另一个终端执行：

```bash
# 发送测试命令
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.5, \"stick\": 0.0, \"boom\": 0.0, \"swing\": 0.0}"'

# 查看输出
ros2 topic echo /pc2000_joint_command --once
```

应该看到 JointState 消息输出。

### 检查 3: 日志输出

主节点应该在接收到控制命令时打印日志，显示当前的关节位置。

## 故障排除

### 问题 1: "No executable found"

**解决方案**：
```bash
cd /home/cyber007/code_ws
colcon build --packages-select keyboard_teleop
source install/setup.bash
```

### 问题 2: 没有输出或反应

**解决方案**：
1. 确认主节点正在运行
2. 检查 topic 连接：`ros2 topic list`
3. 查看节点日志是否有错误

### 问题 3: 权限错误

**解决方案**：
```bash
chmod +x /home/cyber007/code_ws/src/RemoteExcavator/controller/src/keyboard_teleop/src/*.py
chmod +x /home/cyber007/code_ws/src/RemoteExcavator/controller/src/keyboard_teleop/*.sh
```

## 测试完成后

测试完成后，按 `Ctrl+C` 停止节点。

如果需要清理：
```bash
# 杀死所有相关进程
pkill -f teleop_prismatic_publisher
pkill -f test_teleop
```

## 下一步

- 查看 [TEST_README.md](TEST_README.md) 了解详细文档
- 修改测试脚本以适应你的需求
- 集成到你的实际控制系统中

## 文件列表

| 文件 | 说明 | 类型 |
|------|------|------|
| `teleop_prismatic_publisher.py` | 主节点 | 主程序 |
| `test_teleop_control.py` | 自动化测试 | 测试 |
| `test_teleop_manual.py` | 交互式测试 | 测试 |
| `test_launcher.sh` | 启动脚本 | 工具 |
| `TEST_README.md` | 详细文档 | 文档 |
| `QUICKSTART.md` | 本文件 | 文档 |

## 支持

如有问题，请检查：
1. ROS2 环境是否正确 source
2. 包是否已编译
3. 文件权限是否正确
4. 节点是否正在运行

祝测试顺利！ 🚀

