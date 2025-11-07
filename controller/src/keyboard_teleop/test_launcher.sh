#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Teleop Prismatic Publisher 测试启动器${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查是否在正确的目录
if [ ! -f "package.xml" ]; then
    echo -e "${RED}错误: 请在 keyboard_teleop 包目录下运行此脚本${NC}"
    exit 1
fi

# 显示菜单
echo ""
echo "请选择要运行的程序:"
echo "1. 启动主节点 (teleop_prismatic_publisher)"
echo "2. 运行自动化测试 (test_teleop_control)"
echo "3. 运行交互式测试 (test_teleop_manual)"
echo "4. 同时启动主节点和自动化测试（两个终端）"
echo "5. 监控输入 topic (/controls/teleop)"
echo "6. 监控输出 topic (/pc2000_joint_command)"
echo "0. 退出"
echo ""

read -p "输入选项: " choice

case $choice in
    1)
        echo -e "${YELLOW}启动主节点...${NC}"
        ros2 run keyboard_teleop teleop_prismatic_publisher.py
        ;;
    2)
        echo -e "${YELLOW}运行自动化测试...${NC}"
        echo -e "${YELLOW}请确保主节点已在另一个终端运行${NC}"
        sleep 2
        ros2 run keyboard_teleop test_teleop_control.py
        ;;
    3)
        echo -e "${YELLOW}运行交互式测试...${NC}"
        echo -e "${YELLOW}请确保主节点已在另一个终端运行${NC}"
        sleep 2
        ros2 run keyboard_teleop test_teleop_manual.py
        ;;
    4)
        echo -e "${YELLOW}同时启动主节点和测试...${NC}"
        echo -e "${YELLOW}主节点将在后台运行${NC}"
        
        # 启动主节点
        gnome-terminal -- bash -c "echo '=== 主节点 ==='; ros2 run keyboard_teleop teleop_prismatic_publisher.py; exec bash" &
        sleep 2
        
        # 启动测试
        gnome-terminal -- bash -c "echo '=== 自动化测试 ==='; ros2 run keyboard_teleop test_teleop_control.py; exec bash" &
        
        echo -e "${GREEN}已启动两个终端窗口${NC}"
        ;;
    5)
        echo -e "${YELLOW}监控 /controls/teleop topic...${NC}"
        ros2 topic echo /controls/teleop
        ;;
    6)
        echo -e "${YELLOW}监控 /pc2000_joint_command topic...${NC}"
        ros2 topic echo /pc2000_joint_command
        ;;
    0)
        echo -e "${GREEN}退出${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

