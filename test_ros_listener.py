#!/usr/bin/env python3
"""
ROS监听器测试脚本 - 用于验证QGC指令是否正确转换为ROS话题
"""

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def takeoff_command_callback(msg):
    """接收起飞指令"""
    try:
        command_data = json.loads(msg.data)
        action = command_data.get("action", "")
        altitude = command_data.get("altitude", 0)
        auto_arm = command_data.get("auto_arm", False)
        mission_uuid = command_data.get("mission_uuid", "")
        
        print(f"[TEST] 收到起飞指令:")
        print(f"  动作: {action}")
        print(f"  高度: {altitude}m")
        print(f"  自动解锁: {auto_arm}")
        print(f"  任务UUID: {mission_uuid}")
        print(f"  完整数据: {command_data}")
        print("-" * 50)
        
    except Exception as e:
        print(f"[ERROR] 解析起飞指令失败: {e}")
        print(f"  原始数据: {msg.data}")

def set_home_callback(msg):
    """接收设置Home指令"""
    print(f"[TEST] 收到设置Home指令:")
    print(f"  位置: ({msg.pose.position.y}, {msg.pose.position.x}, {msg.pose.position.z})")
    print(f"  时间戳: {msg.header.stamp}")
    print(f"  坐标系: {msg.header.frame_id}")
    print("-" * 50)

def mission_control_callback(msg):
    """接收任务控制指令"""
    try:
        command_data = json.loads(msg.data)
        print(f"[TEST] 收到任务控制指令:")
        print(f"  完整数据: {command_data}")
        print("-" * 50)
        
    except Exception as e:
        print(f"[ERROR] 解析任务控制指令失败: {e}")
        print(f"  原始数据: {msg.data}")

def main():
    # 初始化ROS节点
    rospy.init_node('test_ros_listener', anonymous=True)
    
    print("[INFO] ROS监听器测试启动")
    print("[INFO] 监听以下话题:")
    print("  /qgc/takeoff_command")
    print("  /qgc/set_home_command") 
    print("  /qgc/mission_control")
    print("=" * 50)
    
    # 订阅话题
    rospy.Subscriber('/qgc/takeoff_command', String, takeoff_command_callback)
    rospy.Subscriber('/qgc/set_home_command', PoseStamped, set_home_callback)
    rospy.Subscriber('/qgc/mission_control', String, mission_control_callback)
    
    # 保持运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n[INFO] 测试结束")

if __name__ == '__main__':
    main() 