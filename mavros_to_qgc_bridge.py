#!/usr/bin/env python3
"""
MAVROS到QGC桥接器 - 接收MAVROS消息，转换为MAVLink heartbeat并发送给QGC
"""

import rospy
import time
from mavros_msgs.msg import State
from std_msgs.msg import Int32, Float32
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
        
        # 记录启动时间，用于计算相对时间戳
        self.start_time = time.time()
        
        # 存储接收到的数据
        self.mission_state = 0
        self.flight_time = 0
        self.remaining_distance = 0
        self.captured_images = 0
        self.location_main_status = 0
        self.location_visual_sub_status = 0
        self.satellite_count = 0
        self.location_accuracy = 0.0
        
        # ROS订阅器 - MAVROS状态
        rospy.Subscriber('/mavros/state', State, self._mavros_state_callback)
        
        # ROS订阅器 - 任务数据
        rospy.Subscriber('/mission/state', Int32, self._mission_state_callback)
        rospy.Subscriber('/mission/flight_time', Int32, self._flight_time_callback)
        rospy.Subscriber('/mission/remaining_distance', Int32, self._remaining_distance_callback)
        rospy.Subscriber('/mission/captured_images', Int32, self._captured_images_callback)
        
        # ROS订阅器 - 定位数据
        rospy.Subscriber('/location/main_status', Int32, self._location_main_status_callback)
        rospy.Subscriber('/location/visual_sub_status', Int32, self._location_visual_sub_status_callback)
        rospy.Subscriber('/location/satellite_count', Int32, self._satellite_count_callback)
        rospy.Subscriber('/location/accuracy', Float32, self._location_accuracy_callback)
        
        print("[INFO] MAVLink桥接器初始化完成，等待接收ROS数据...")
        
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
        
        # 发送任务状态数据
        self._send_mission_status_data()
        
        # 发送定位状态数据
        self._send_location_status_data()
        
        print(f"UDP发送MAVLink心跳: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}, system_status={system_status}")
        
    # ROS回调函数 - 任务数据
    def _mission_state_callback(self, msg):
        self.mission_state = msg.data
        
    def _flight_time_callback(self, msg):
        self.flight_time = msg.data
        
    def _remaining_distance_callback(self, msg):
        self.remaining_distance = msg.data
        
    def _captured_images_callback(self, msg):
        self.captured_images = msg.data
        
    # ROS回调函数 - 定位数据
    def _location_main_status_callback(self, msg):
        self.location_main_status = msg.data
        
    def _location_visual_sub_status_callback(self, msg):
        self.location_visual_sub_status = msg.data
        
    def _satellite_count_callback(self, msg):
        self.satellite_count = msg.data
        
    def _location_accuracy_callback(self, msg):
        self.location_accuracy = msg.data
        
    def _send_mission_status_data(self):
        """发送任务状态数据（使用从ROS接收的真实数据）"""
        # 使用相对时间戳（从启动开始的毫秒数），确保在32位整数范围内
        relative_time_ms = int((time.time() - self.start_time) * 1000) % 4294967295
        
        # 发送任务状态（从ROS接收的数据）
        self.mavlink_connection.mav.named_value_int_send(
            time_boot_ms=relative_time_ms,
            name=b"MISSION_ST",
            value=self.mission_state
        )
        
        # 发送飞行时间（从ROS接收的数据）
        self.mavlink_connection.mav.named_value_int_send(
            time_boot_ms=relative_time_ms,
            name=b"FLIGHT_TM", 
            value=self.flight_time
        )
        
        # 发送剩余航程（从ROS接收的数据）
        self.mavlink_connection.mav.named_value_int_send(
            time_boot_ms=relative_time_ms,
            name=b"REMAIN_DST",
            value=self.remaining_distance
        )
        
        # 发送采集图片数（从ROS接收的数据）
        self.mavlink_connection.mav.named_value_int_send(
            time_boot_ms=relative_time_ms,
            name=b"IMG_COUNT",
            value=self.captured_images
        )
        
    def _send_location_status_data(self):
        """发送定位状态数据（使用从ROS接收的真实数据）"""
        try:
            # 使用相对时间戳
            relative_time_ms = int((time.time() - self.start_time) * 1000) % 4294967295
            
            # 使用从ROS接收的定位状态数据
            main_status = self.location_main_status
            visual_sub_status = self.location_visual_sub_status
            satellite_count = self.satellite_count
            accuracy = self.location_accuracy
            
            # 位运算编码：低8位=主状态，次8位=视觉子状态
            location_status_encoded = main_status | (visual_sub_status << 8)
            
            # 发送定位状态（使用位运算编码）
            self.mavlink_connection.mav.named_value_int_send(
                time_boot_ms=relative_time_ms,
                name=b"LOC_ST",
                value=location_status_encoded
            )
            
            # 发送卫星数量
            self.mavlink_connection.mav.named_value_int_send(
                time_boot_ms=relative_time_ms,
                name=b"SAT_COUNT",
                value=satellite_count
            )
            
            # 发送定位精度（浮点型）
            self.mavlink_connection.mav.named_value_float_send(
                time_boot_ms=relative_time_ms,
                name=b"LOC_ACCUR",
                value=accuracy
            )
            
            # 调试：打印定位状态发送信息
            status_names = ["无定位", "卫导定位", "视觉定位"]
            visual_names = ["未初始化", "单应", "ORB SLAM"]
            main_name = status_names[main_status] if main_status < len(status_names) else "未知"
            visual_name = visual_names[visual_sub_status] if visual_sub_status < len(visual_names) else "未知"
            print(f"桥接发送定位状态: {main_name}-{visual_name} (编码值:{location_status_encoded}, 主:{main_status}, 子:{visual_sub_status})")
            
        except Exception as e:
            print(f"[ERROR] 发送定位状态数据失败: {e}")
            import traceback
            traceback.print_exc()
        
    def run(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = MAVROSToQGCBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass 