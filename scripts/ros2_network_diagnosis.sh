#!/bin/bash

# ROS2 多机通信诊断脚本
# 用于诊断和修复ROS2在不同电脑间通信时能看到话题但收不到数据的问题

set -e

echo "🔍 ROS2 多机通信诊断工具"
echo "================================"
echo ""

# 1. 检查ROS2环境
echo "1️⃣ 检查ROS2环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS2环境未加载"
    echo "   请先执行: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo "   ✅ ROS_DISTRO: $ROS_DISTRO"
fi

# 2. 检查ROS_DOMAIN_ID
echo ""
echo "2️⃣ 检查ROS_DOMAIN_ID..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "   ⚠️  ROS_DOMAIN_ID未设置（默认值为0）"
    echo "   建议：在所有机器上设置相同的ROS_DOMAIN_ID"
    echo "   执行: export ROS_DOMAIN_ID=0"
    ROS_DOMAIN_ID=0
else
    echo "   ✅ ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
fi

# 3. 检查网络接口
echo ""
echo "3️⃣ 检查网络接口..."
LOCAL_IP=$(hostname -I | awk '{print $1}')
if [ -z "$LOCAL_IP" ]; then
    echo "   ❌ 无法获取本机IP地址"
    exit 1
else
    echo "   ✅ 本机IP: $LOCAL_IP"
fi

# 显示所有网络接口
echo "   网络接口列表:"
ip -4 addr show | grep -E "^[0-9]+:|inet " | grep -v "127.0.0.1" | while read line; do
    if [[ $line =~ ^[0-9]+: ]]; then
        interface=$(echo $line | awk '{print $2}' | tr -d ':')
    elif [[ $line =~ inet ]]; then
        ip=$(echo $line | awk '{print $2}' | cut -d'/' -f1)
        echo "      - $interface: $ip"
    fi
done

# 4. 检查防火墙
echo ""
echo "4️⃣ 检查防火墙配置..."
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status | grep -i "Status:" | awk '{print $2}' || echo "inactive")
    if [ "$UFW_STATUS" = "active" ]; then
        echo "   ⚠️  防火墙已启用"
        echo "   ROS2 DDS通信需要开放以下端口:"
        echo "      - UDP 7400-7500 (DDS发现)"
        echo "      - UDP 7410-7420 (DDS数据)"
        
        # 检查并开放端口
        if ! sudo ufw status | grep -q "7400:7500"; then
            echo "   🔧 正在开放DDS端口..."
            sudo ufw allow 7400:7500/udp
            echo "   ✅ 已开放UDP 7400-7500"
        else
            echo "   ✅ DDS端口已开放"
        fi
    else
        echo "   ✅ 防火墙未启用"
    fi
else
    echo "   ℹ️  未检测到ufw防火墙"
fi

# 5. 检查ROS2话题
echo ""
echo "5️⃣ 检查ROS2话题..."
echo "   正在列出所有话题（可能需要几秒钟）..."
timeout 5 ros2 topic list 2>/dev/null || {
    echo "   ⚠️  无法列出话题，可能没有运行中的ROS2节点"
    echo "   请确保至少有一个ROS2节点在运行"
}

# 6. 测试话题通信
echo ""
echo "6️⃣ 测试话题通信..."
echo "   选择一个话题进行测试（按Ctrl+C跳过）:"
read -t 5 -p "   输入话题名称（留空跳过）: " TEST_TOPIC || TEST_TOPIC=""

if [ -n "$TEST_TOPIC" ]; then
    echo "   正在监听话题: $TEST_TOPIC"
    echo "   如果5秒内没有收到数据，说明存在通信问题"
    timeout 5 ros2 topic echo "$TEST_TOPIC" --once 2>&1 || {
        echo "   ⚠️  未收到数据，可能的原因:"
        echo "      1. QoS不匹配"
        echo "      2. 防火墙阻止"
        echo "      3. ROS_DOMAIN_ID不一致"
        echo "      4. 网络接口配置问题"
    }
fi

# 7. 生成配置建议
echo ""
echo "7️⃣ 配置建议"
echo "================================"
echo ""
echo "为了确保多机通信正常工作，请在所有机器上执行以下配置:"
echo ""
echo "1. 设置ROS_DOMAIN_ID（所有机器必须相同）:"
echo "   export ROS_DOMAIN_ID=0"
echo "   或添加到 ~/.bashrc:"
echo "   echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc"
echo ""
echo "2. 设置网络接口（如果需要）:"
echo "   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "   export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profile.xml"
echo ""
echo "3. 检查防火墙:"
echo "   sudo ufw allow 7400:7500/udp"
echo ""
echo "4. 验证网络连接:"
echo "   在机器A上: ping <机器B的IP>"
echo "   在机器B上: ping <机器A的IP>"
echo ""
echo "5. 测试话题发现:"
echo "   在机器A上: ros2 topic list"
echo "   在机器B上: ros2 topic list"
echo "   应该能看到相同的话题列表"
echo ""
echo "6. 测试数据接收:"
echo "   在机器A上: ros2 topic pub /test_topic std_msgs/msg/String \"{data: 'hello'}\""
echo "   在机器B上: ros2 topic echo /test_topic"
echo "   应该能看到 'hello' 消息"
echo ""

