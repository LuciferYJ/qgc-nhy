#!/usr/bin/env python3
"""
MAVROS到QGC桥接器 - 接收MAVROS消息，转换为MAVLink heartbeat并发送给QGC
"""

import rospy
from mavros_msgs.msg import State
from pymavlink import mavutil

class MAVROSToQGCBridge:
    def __init__(self):
        # QGC连接配置
        self.qgc_host = "127.0.0.1"
        self.qgc_port = 7777
        
        # 创建MAVLink连接
        self.mavlink_connection = mavutil.mavlink_connection(
            f"udpout:{self.qgc_host}:{self.qgc_port}",
            source_system=1,
            source_component=1
        )
        
        # ROS节点初始化
        rospy.init_node('mavros_to_qgc_bridge', anonymous=True)
        
        # ROS订阅器
        rospy.Subscriber('/mavros/state', State, self._mavros_state_callback)
        
    def _mavros_state_callback(self, msg):
        """处理MAVROS状态消息"""
        # 计算base_mode
        base_mode = 0
        if msg.armed:
            base_mode |= 128  # MAV_MODE_FLAG_SAFETY_ARMED
        if msg.connected:
            base_mode |= 64   # MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            base_mode |= 16   # MAV_MODE_FLAG_STABILIZE_ENABLED
        
        # 设置system_status
        if msg.connected:
            system_status = 4 if msg.armed else 3  # MAV_STATE_ACTIVE : MAV_STATE_STANDBY
        else:
            system_status = 0  # MAV_STATE_UNINIT
        
        # 发送MAVLink心跳
        self.mavlink_connection.mav.heartbeat_send(
            type=2,                    # MAV_TYPE_QUADROTOR
            autopilot=12,              # MAV_AUTOPILOT_PX4
            base_mode=base_mode,       # base_mode bitmap
            custom_mode=0,             # custom_mode
            system_status=system_status, # MAV_STATE
            mavlink_version=3          # MAVLink version
        )
        
        print(f"UDP发送MAVLink心跳: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}, system_status={system_status}")
        
    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = MAVROSToQGCBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass 