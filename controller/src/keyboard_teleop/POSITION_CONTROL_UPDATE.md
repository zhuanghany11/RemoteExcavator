# 位置控制模式更新说明

## 📋 更新概述

将 `teleop_prismatic_publisher.py` 从**增量/速度控制模式**改为**位置控制模式**，以匹配前端 `useRosPublisher` 的映射逻辑。

**更新日期**: 2025-11-07

---

## 🔄 架构说明

```
前端 (React + Gamepad)
    ↓ 发送 JSON (通过 WebRTC/WebSocket)
/controls/teleop (std_msgs/String)
    ↓ 订阅
teleop_prismatic_publisher.py (本节点)
    ↓ 映射并发布
/pc2000_joint_command (sensor_msgs/JointState)
    ↓ 订阅
挖掘机仿真器/实体
```

**关键点**: 
- 前端发送控制值到 `/controls/teleop` (JSON格式)
- 本节点将其转换为 JointState 消息
- 发布到 `/pc2000_joint_command` 供仿真器使用

---

## ⚙️ 主要修改

### 1. 控制模式变更

**之前 (增量控制)**:
```python
# 输入值 → 速度 → 位置积分
delta = input * step_size
position += delta
```

**现在 (位置控制)**:
```python
# 输入值 → 直接映射到目标位置
position = input * scale_factor
```

### 2. 映射逻辑 - 与前端完全一致

参考前端代码 (`useRosPublisher`):
```typescript
position: [
  -currentControls.bucket * 3.0,  // bucket_linear
  -currentControls.boom * 3.0,    // arm_linear (大臂)
  -currentControls.stick * 3.0,   // boom_linear (小臂)
  currentControls.swing * Math.PI // body_rotate
]
```

**Python 实现**:
```python
bucket_pos = -bucket_input * 3.0      # bucket → bucket_linear
arm_pos = -boom_input * 3.0           # boom → arm_linear (大臂)
boom_pos = -stick_input * 3.0         # stick → boom_linear (小臂)
body_yaw = swing_input * math.pi      # swing → body_rotate
```

### 3. 比例因子

| 参数 | 值 | 说明 |
|------|-----|------|
| `linear_scale` | 3.0 | 线性关节: 输入±1.0 → ±3.0米 |
| `rotation_scale` | π (3.14159) | 旋转关节: 输入±1.0 → ±180° |

### 4. 移除定时器

**之前**: 10Hz 定时器持续发布
```python
self.create_timer(0.1, self.timer_callback)
```

**现在**: 事件驱动，仅在收到新消息时发布
```python
# 在 teleop_callback 中直接调用 publish_joint_state()
```

---

## 🎯 完整映射表

| 前端字段 | 输入范围 | 语义 | ROS2 关节 | 映射公式 | 输出范围 |
|---------|---------|------|-----------|----------|---------|
| `bucket` | -1.0 ~ 1.0 | 收回→翻出 | `bucket_linear` | `-bucket * 3.0` | -3.0m ~ 3.0m |
| `boom` | -1.0 ~ 1.0 | 下降→提升 | `arm_linear` | `-boom * 3.0` | -3.0m ~ 3.0m |
| `stick` | -1.0 ~ 1.0 | 收回→伸出 | `boom_linear` | `-stick * 3.0` | -3.0m ~ 3.0m |
| `swing` | -1.0 ~ 1.0 | 左旋→右旋 | `body_rotate` | `swing * π` | -π ~ π |

**关键注意事项**:
1. ⚠️ **boom 和 stick 在前端映射中交叉**: 
   - 前端的 `boom` → ROS2 的 `arm_linear`
   - 前端的 `stick` → ROS2 的 `boom_linear`
2. ⚠️ **负号**: 所有线性关节都取反（负号）
3. ✅ **旋转**: `body_rotate` 不取反

---

## 📝 代码示例

### 输入消息示例 (JSON)

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

### 输出消息示例 (JointState)

```yaml
header:
  stamp: {sec: 1699363200, nanosec: 0}
  frame_id: ""
name: 
  - bucket_linear
  - arm_linear
  - boom_linear
  - body_rotate
position:
  - -1.5    # -bucket(0.5) * 3.0 = -1.5
  - 0.6     # -boom(-0.2) * 3.0 = 0.6
  - -0.9    # -stick(0.3) * 3.0 = -0.9
  - 0.314   # swing(0.1) * π ≈ 0.314
velocity: [0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0]
```

---

## 🧪 测试方法

### 1. 启动节点

```bash
cd /home/cyber007/code_ws
source install/setup.bash
ros2 run keyboard_teleop teleop_prismatic_publisher.py
```

### 2. 发送测试命令

```bash
# 测试 bucket (正向)
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 1.0, \"stick\": 0.0, \"boom\": 0.0, \"swing\": 0.0}"'

# 预期输出: bucket_linear = -3.0

# 测试 boom (负向)
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.0, \"stick\": 0.0, \"boom\": -1.0, \"swing\": 0.0}"'

# 预期输出: arm_linear = 3.0

# 测试 swing (旋转)
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  'data: "{\"bucket\": 0.0, \"stick\": 0.0, \"boom\": 0.0, \"swing\": 1.0}"'

# 预期输出: body_rotate = 3.14159
```

### 3. 监控输出

```bash
# 查看关节命令
ros2 topic echo /pc2000_joint_command

# 查看节点日志
# 应该显示: "目标位置 → bucket: -3.000m, arm: 0.000m, boom: 0.000m, body_yaw: 0.000rad (0.0°)"
```

---

## ✅ 验证清单

测试以下场景确保正确性：

- [ ] **Bucket 控制**
  - 输入 `bucket: 1.0` → 输出 `bucket_linear: -3.0`
  - 输入 `bucket: -1.0` → 输出 `bucket_linear: 3.0`

- [ ] **Boom 控制** (注意映射到 arm_linear)
  - 输入 `boom: 1.0` → 输出 `arm_linear: -3.0`
  - 输入 `boom: -1.0` → 输出 `arm_linear: 3.0`

- [ ] **Stick 控制** (注意映射到 boom_linear)
  - 输入 `stick: 1.0` → 输出 `boom_linear: -3.0`
  - 输入 `stick: -1.0` → 输出 `boom_linear: 3.0`

- [ ] **Swing 控制**
  - 输入 `swing: 1.0` → 输出 `body_rotate: 3.14` (约180°)
  - 输入 `swing: -1.0` → 输出 `body_rotate: -3.14` (约-180°)

- [ ] **组合控制**
  - 多个输入同时发送，所有关节都正确响应

- [ ] **边界值**
  - 输入超出 ±1.0 范围会被自动钳制

---

## 🐛 常见问题排查

### 问题 1: 挖掘机不动作

**可能原因**:
1. 节点未启动或崩溃
2. Topic 连接问题
3. 映射方向错误

**排查步骤**:
```bash
# 1. 检查节点是否运行
ros2 node list | grep teleop_prismatic_publisher

# 2. 检查 topic 连接
ros2 topic info /controls/teleop
ros2 topic info /pc2000_joint_command

# 3. 查看实时数据
ros2 topic echo /pc2000_joint_command
```

### 问题 2: 动作方向相反

**解决方案**: 检查前端发送的值的符号，确保与预期一致。

### 问题 3: 动作幅度不对

**解决方案**: 
- 检查比例因子 (`linear_scale = 3.0`)
- 确认前端发送的值在 -1.0 到 1.0 范围内

---

## 📚 相关文件

| 文件 | 说明 |
|------|------|
| `teleop_prismatic_publisher.py` | 主节点（已更新） |
| `test_teleop_control.py` | 自动化测试脚本 |
| `test_teleop_manual.py` | 交互式测试脚本 |
| `QUICKSTART.md` | 快速开始指南 |
| `TEST_README.md` | 详细测试文档 |

---

## 🔧 参数调整

如需修改映射范围，在代码中调整：

```python
# 在 __init__ 方法中
self.linear_scale = 3.0      # 修改线性范围 (默认 ±3.0m)
self.rotation_scale = math.pi # 修改旋转范围 (默认 ±180°)
```

例如，如果想增大线性范围到 ±5.0m：
```python
self.linear_scale = 5.0  # 输入±1.0 → ±5.0米
```

---

## 📞 技术支持

如有问题：
1. 查看节点启动日志
2. 使用 `ros2 topic echo` 监控消息
3. 检查此文档的测试方法章节
4. 参考前端代码确认输入值范围

---

**版本**: 1.1 (位置控制模式)  
**维护者**: RemoteExcavator Team  
**最后更新**: 2025-11-07

