# 高级遥操作控制节点使用说明

## 概述

`teleop_advanced_control.py` 是一个增强版的遥操作控制节点，在原有 `teleop_prismatic_publisher.py` 的基础上，添加了以下高级控制功能：

1. **校准和归一化** - 处理手柄中位偏移
2. **死区（Deadzone）** - 中间 ±3%～±20% 不响应，防止轻微抖动
3. **限幅和斜坡（Rate Limit）** - 限制指令变化速率，每20ms最大变化5%
4. **非线性响应曲线** - 低段细、末端陡，便于微动操作

## 功能详解

### 1. 校准和归一化

**问题**：手柄中位可能不在0，导致即使不操作也会产生微小输出。

**解决方案**：
- 启动校准流程，保持手柄在中位
- 系统采样100次（可配置），计算每个轴的平均中位值
- 后续所有输入都会减去这个偏移值

**使用方法**：
```bash
# 启动校准
ros2 service call /teleop_advanced/start_calibration std_srvs/srv/Trigger

# 保持手柄在中位，等待几秒后停止校准
ros2 service call /teleop_advanced/stop_calibration std_srvs/srv/Trigger
```

### 2. 死区（Deadzone）

**问题**：手柄轻微抖动会导致挖掘机"抖三抖"，影响操作精度。

**解决方案**：
- 默认死区：10%（可配置范围：3%～20%）
- 死区内输入被映射为0
- 死区外输入线性映射到 [0, 1] 范围

**配置参数**：
- `deadzone_min`: 最小死区（默认 0.03 = 3%）
- `deadzone_max`: 最大死区（默认 0.20 = 20%）
- `deadzone_default`: 默认死区（默认 0.10 = 10%）

### 3. 限幅和斜坡（Rate Limit）

**问题**：突然推到底会导致指令从0跳到100%，动作过于猛烈。

**解决方案**：
- 限制每20ms（可配置）的最大变化量为5%（可配置）
- 即使输入突然变化，输出也会平滑过渡
- 提供温和的加速/减速过程

**配置参数**：
- `rate_limit_percent`: 每周期最大变化百分比（默认 0.05 = 5%）
- `control_period_ms`: 控制周期（默认 20ms）

**效果**：
- 输入从0突然跳到1.0时，输出会以每20ms 5%的速度平滑增加
- 约400ms后达到最大值（20个周期 × 20ms = 400ms）

### 4. 非线性响应曲线

**问题**：线性响应难以同时满足微动和高速操作的需求。

**解决方案**：
- 使用幂函数曲线：`y = x^exponent`
- 指数 > 1：低段细（便于微动）、末端陡（保留高速能力）
- 默认指数：2.0（平方曲线）

**配置参数**：
- `response_curve_exponent`: 响应曲线指数（默认 2.0）

**曲线特性**：
- `exponent = 1.0`: 线性响应
- `exponent = 2.0`: 平方曲线（推荐，低段细、末端陡）
- `exponent = 3.0`: 立方曲线（更激进）
- `exponent = 0.5`: 平方根曲线（低段陡、末端细）

## 启动节点

### 基本启动

```bash
ros2 run keyboard_teleop teleop_advanced_control
```

### 带参数启动

```bash
# 自定义死区为15%
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p deadzone_default:=0.15

# 自定义限幅速率为3%（更保守）
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p rate_limit_percent:=0.03

# 自定义响应曲线指数为2.5
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p response_curve_exponent:=2.5

# 禁用校准功能
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p enable_calibration:=false
```

### 完整参数示例

```bash
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p deadzone_default:=0.12 \
  -p rate_limit_percent:=0.05 \
  -p control_period_ms:=20 \
  -p response_curve_exponent:=2.0 \
  -p enable_calibration:=true \
  -p calibration_samples:=100
```

## 输入处理流程

每个输入值经过以下处理流程：

```
原始输入 [-1, 1]
    ↓
1. 校准（减去中位偏移）
    ↓
2. 死区处理（死区内→0，死区外→线性映射）
    ↓
3. 非线性响应曲线（幂函数变换）
    ↓
4. 限幅和斜坡（限制变化速率）
    ↓
最终输出 [-1, 1]
```

## 与原始节点的对比

| 特性 | teleop_prismatic_publisher.py | teleop_advanced_control.py |
|------|------------------------------|---------------------------|
| 死区 | 固定30% | 可配置3%～20% |
| 校准 | ❌ | ✅ 支持中位校准 |
| 限幅 | 仅限yaw轴 | ✅ 所有轴都支持 |
| 响应曲线 | 线性 | ✅ 非线性（可配置） |
| 控制周期 | 100ms (10Hz) | 20ms (50Hz) |

## 话题和服务

### 订阅话题

- `/controls/teleop` (std_msgs/String): 遥操作控制输入（JSON格式）
- `/joint_states` (sensor_msgs/JointState): 关节状态反馈

### 发布话题

- `/pc2000_joint_command` (sensor_msgs/JointState): 关节命令输出

### 服务

- `/teleop_advanced/start_calibration` (std_srvs/Trigger): 启动校准
- `/teleop_advanced/stop_calibration` (std_srvs/Trigger): 停止校准并计算偏移

## 参数列表

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `topic` | string | `/pc2000_joint_command` | 输出话题名称 |
| `deadzone_min` | double | 0.03 | 最小死区（3%） |
| `deadzone_max` | double | 0.20 | 最大死区（20%） |
| `deadzone_default` | double | 0.10 | 默认死区（10%） |
| `rate_limit_percent` | double | 0.05 | 每周期最大变化（5%） |
| `control_period_ms` | int | 20 | 控制周期（毫秒） |
| `response_curve_exponent` | double | 2.0 | 响应曲线指数 |
| `enable_calibration` | bool | true | 是否启用校准 |
| `calibration_samples` | int | 100 | 校准采样次数 |
| `track_velocity_scale` | double | 10.0 | 履带速度缩放因子 |

## 使用建议

### 精细操作场景（如精细土方）

```bash
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p deadzone_default:=0.15 \
  -p rate_limit_percent:=0.03 \
  -p response_curve_exponent:=2.5
```

- 更大的死区（15%）减少抖动
- 更小的限幅（3%）提供更平滑的控制
- 更高的指数（2.5）增强微动能力

### 快速操作场景

```bash
ros2 run keyboard_teleop teleop_advanced_control --ros-args \
  -p deadzone_default:=0.08 \
  -p rate_limit_percent:=0.08 \
  -p response_curve_exponent:=1.5
```

- 较小的死区（8%）提高响应速度
- 较大的限幅（8%）允许更快的变化
- 较低的指数（1.5）提供更线性的响应

## 故障排除

### 校准不生效

1. 检查校准功能是否启用：`enable_calibration:=true`
2. 确保校准过程中保持手柄在中位
3. 等待足够的采样次数（默认100次）

### 响应过于迟钝

1. 减小死区：`deadzone_default:=0.05`
2. 增大限幅：`rate_limit_percent:=0.08`
3. 降低响应曲线指数：`response_curve_exponent:=1.5`

### 响应过于敏感

1. 增大死区：`deadzone_default:=0.15`
2. 减小限幅：`rate_limit_percent:=0.03`
3. 提高响应曲线指数：`response_curve_exponent:=2.5`

## 技术细节

### 死区映射公式

```
if |input| <= deadzone:
    output = 0
else:
    output = sign(input) * (|input| - deadzone) / (1 - deadzone)
```

### 响应曲线公式

```
output = sign(input) * |input|^exponent
```

### 限幅公式

```
max_delta = rate_limit_percent * (dt / control_period)
delta = clamp(desired - last, -max_delta, max_delta)
output = last + delta
```

## 许可证

与主项目保持一致。

