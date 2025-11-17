# ROS2 多机通信配置指南

## 问题描述

在同一个局域网的不同电脑上，可以看到ROS2话题，但是话题中没有内容。这是一个常见的ROS2多机通信问题。

## 原因分析

1. **QoS不匹配**: 发布者和订阅者使用不同的QoS策略（RELIABLE vs BEST_EFFORT）
2. **ROS_DOMAIN_ID不一致**: 不同机器上的ROS_DOMAIN_ID不同
3. **防火墙阻止**: 防火墙阻止了DDS通信端口（UDP 7400-7500）
4. **网络接口配置**: ROS2没有正确绑定到网络接口

## 解决方案

### 1. 统一QoS配置

已修复代码中的QoS配置，确保发布者和订阅者使用兼容的QoS策略：

- **发布者**: 使用 `RELIABLE` QoS（确保数据可靠传输）
- **订阅者**: 使用 `RELIABLE` QoS（匹配发布者）

**注意**: 在ROS2中，RELIABLE发布者可以匹配BEST_EFFORT订阅者，但BEST_EFFORT发布者不能匹配RELIABLE订阅者。

### 2. 配置ROS_DOMAIN_ID

在所有需要通信的机器上设置相同的 `ROS_DOMAIN_ID`：

```bash
# 临时设置（当前终端有效）
export ROS_DOMAIN_ID=0

# 永久设置（添加到 ~/.bashrc）
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

**重要**: 所有机器必须使用相同的 `ROS_DOMAIN_ID`（默认值为0）。

### 3. 配置防火墙

开放ROS2 DDS通信所需的UDP端口：

```bash
# Ubuntu/Debian (ufw)
sudo ufw allow 7400:7500/udp

# 或者使用iptables
sudo iptables -A INPUT -p udp --dport 7400:7500 -j ACCEPT
```

### 4. 验证网络连接

确保机器之间可以互相ping通：

```bash
# 在机器A上
ping <机器B的IP>

# 在机器B上
ping <机器A的IP>
```

### 5. 测试话题通信

#### 步骤1: 列出话题
在机器A和机器B上分别执行：
```bash
ros2 topic list
```
应该能看到相同的话题列表。

#### 步骤2: 测试发布和接收
在机器A上发布测试消息：
```bash
ros2 topic pub /test_topic std_msgs/msg/String "{data: 'hello from machine A'}"
```

在机器B上接收消息：
```bash
ros2 topic echo /test_topic
```

如果能看到消息，说明通信正常。

## 使用诊断脚本

项目提供了两个脚本来帮助诊断和配置：

### 诊断脚本
```bash
cd /home/orin64/code_ws/src/RemoteExcavator
./scripts/ros2_network_diagnosis.sh
```

这个脚本会：
- 检查ROS2环境
- 检查ROS_DOMAIN_ID
- 检查网络接口
- 检查防火墙配置
- 测试话题通信

### 配置脚本
```bash
cd /home/orin64/code_ws/src/RemoteExcavator
./scripts/setup_ros2_multimachine.sh
```

这个脚本会：
- 交互式设置ROS_DOMAIN_ID
- 自动配置防火墙
- 可选创建FastDDS配置文件

## 代码修改说明

已修改以下文件以支持多机通信：

1. **teleop_prismatic_publisher.py**
   - 发布者使用RELIABLE QoS
   - 订阅者使用RELIABLE QoS

2. **test_teleop_control.py**
   - 发布者使用RELIABLE QoS

3. **example_integration.py**
   - 发布者使用RELIABLE QoS

## 常见问题

### Q: 为什么能看到话题但收不到数据？

A: 这通常是因为QoS不匹配。确保发布者和订阅者使用兼容的QoS策略。

### Q: 如何检查当前使用的QoS？

A: 使用以下命令查看话题信息：
```bash
ros2 topic info /your_topic_name --verbose
```

### Q: 防火墙已配置，但仍然无法通信？

A: 检查：
1. ROS_DOMAIN_ID是否相同
2. 网络接口是否正确
3. 是否有其他防火墙规则阻止

### Q: 可以使用不同的ROS_DOMAIN_ID吗？

A: 可以，但只有相同ROS_DOMAIN_ID的节点才能互相通信。建议所有机器使用相同的值。

## 参考资源

- [ROS2 DDS Configuration](https://docs.ros.org/en/humble/How-To-Guides/DDS-and-ROS-middleware-implementations.html)
- [ROS2 QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS2 Multi-Machine Setup](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)

