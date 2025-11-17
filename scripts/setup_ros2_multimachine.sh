#!/bin/bash

# ROS2 多机通信配置脚本
# 自动配置ROS2环境变量，确保多机通信正常工作

set -e

echo "🔧 ROS2 多机通信配置工具"
echo "================================"
echo ""

# 获取本机IP
LOCAL_IP=$(hostname -I | awk '{print $1}')
if [ -z "$LOCAL_IP" ]; then
    echo "❌ 无法获取本机IP地址"
    exit 1
fi

echo "📍 本机IP: $LOCAL_IP"
echo ""

# 1. 设置ROS_DOMAIN_ID
read -p "请输入ROS_DOMAIN_ID (默认: 0，所有机器必须相同): " DOMAIN_ID
DOMAIN_ID=${DOMAIN_ID:-0}
echo "export ROS_DOMAIN_ID=$DOMAIN_ID" >> ~/.bashrc
export ROS_DOMAIN_ID=$DOMAIN_ID
echo "✅ 已设置 ROS_DOMAIN_ID=$DOMAIN_ID"

# 2. 配置防火墙
echo ""
echo "2️⃣ 配置防火墙..."
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status | grep -i "Status:" | awk '{print $2}' || echo "inactive")
    if [ "$UFW_STATUS" = "active" ]; then
        echo "   防火墙已启用，正在开放ROS2 DDS端口..."
        sudo ufw allow 7400:7500/udp
        echo "✅ 已开放UDP 7400-7500 (DDS通信端口)"
    else
        echo "✅ 防火墙未启用，无需配置"
    fi
else
    echo "ℹ️  未检测到ufw防火墙"
fi

# 3. 创建FastDDS配置文件（可选）
echo ""
read -p "是否创建FastDDS配置文件以指定网络接口? (y/N): " CREATE_FASTDDS
if [[ "$CREATE_FASTDDS" =~ ^[Yy]$ ]]; then
    FASTDDS_DIR="$HOME/.ros/fastdds"
    mkdir -p "$FASTDDS_DIR"
    
    FASTDDS_FILE="$FASTDDS_DIR/fastdds_profile.xml"
    cat > "$FASTDDS_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>$LOCAL_IP</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_FILE" >> ~/.bashrc
    export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_FILE
    echo "✅ 已创建FastDDS配置文件: $FASTDDS_FILE"
fi

# 4. 显示配置摘要
echo ""
echo "================================"
echo "✅ 配置完成！"
echo ""
echo "配置摘要:"
echo "  - ROS_DOMAIN_ID: $DOMAIN_ID"
echo "  - 本机IP: $LOCAL_IP"
if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "  - FastDDS配置: $FASTRTPS_DEFAULT_PROFILES_FILE"
fi
echo ""
echo "⚠️  重要提示:"
echo "1. 请在所有需要通信的机器上执行相同的配置"
echo "2. 确保所有机器的ROS_DOMAIN_ID相同"
echo "3. 重新加载环境变量: source ~/.bashrc"
echo "4. 或重新打开终端窗口"
echo ""
echo "测试命令:"
echo "  - 列出话题: ros2 topic list"
echo "  - 测试发布: ros2 topic pub /test_topic std_msgs/msg/String \"{data: 'hello'}\""
echo "  - 测试接收: ros2 topic echo /test_topic"
echo ""

