#!/usr/bin/env python3
"""
MAVROS发布器 - 模拟发布ROS消息供mavros_to_qgc_bridge.py使用
"""

import rospy
import time
from mavros_msgs.msg import State
from std_msgs.msg import Int32, Float32

class MAVROSPublisher:
    def __init__(self):
        # 消息类型常量 (与CustomUdpTypes.h中的MessageType枚举对应)
        # MSG_HEARTBEAT = 0, MSG_TAKEOFF_COMMAND = 1, MSG_SET_HOME_COMMAND = 2
        # MSG_MISSION_STATUS = 3, MSG_LOCATION_STATUS = 4, MSG_ACK_RESPONSE = 5, MSG_ERROR = 6
        
        # ROS节点初始化
        rospy.init_node('mavros_publisher', anonymous=True)
        
        # ROS发布器 - MAVROS状态
        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=10)
        
        # ROS发布器 - 任务数据
        self.mission_state_pub = rospy.Publisher('/mission/state', Int32, queue_size=10)
        self.flight_time_pub = rospy.Publisher('/mission/flight_time', Int32, queue_size=10)
        self.remaining_distance_pub = rospy.Publisher('/mission/remaining_distance', Int32, queue_size=10)
        self.captured_images_pub = rospy.Publisher('/mission/captured_images', Int32, queue_size=10)
        self.flown_distance_pub = rospy.Publisher('/mission/flown_distance', Int32, queue_size=10)
        self.total_distance_pub = rospy.Publisher('/mission/total_distance', Int32, queue_size=10)
        
        # ROS发布器 - 定位数据
        self.location_main_status_pub = rospy.Publisher('/location/main_status', Int32, queue_size=10)
        self.location_visual_sub_status_pub = rospy.Publisher('/location/visual_sub_status', Int32, queue_size=10)
        self.satellite_count_pub = rospy.Publisher('/location/satellite_count', Int32, queue_size=10)
        self.location_accuracy_pub = rospy.Publisher('/location/accuracy', Float32, queue_size=10)
        
        # 等待发布器准备就绪
        rospy.sleep(1)
        
        # 初始化模拟数据
        self.mission_time = 0
        
        print("[INFO] MAVROS发布器初始化完成，开始发布模拟数据...")
        
    def publish_mavros_state(self):
        """发布MAVROS状态消息"""
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = False
        state_msg.guided = True
        state_msg.manual_input = False
        state_msg.mode = "OFFBOARD"
        state_msg.system_status = 4  # MAV_STATE_ACTIVE
        
        self.state_pub.publish(state_msg)
        print(f"发布MAVROS状态: connected={state_msg.connected}, mode={state_msg.mode}")
        
    def publish_mission_data(self):
        """发布任务状态模拟数据"""
        # 任务状态模拟 (0:空闲, 1:就绪, 2:起飞, 3:巡航, 4:返航, 5:降落)
        mission_states = [0, 1, 2, 3, 4, 5]
        current_state = mission_states[(self.mission_time // 10) % len(mission_states)]
        
        # 飞行时间（秒）
        flight_time = self.mission_time * 5 if current_state > 1 else 0
        
        # 剩余航程（米）
        remaining_distance = max(0, 1000 - self.mission_time * 20) if current_state in [2, 3] else 0
        
        # 采集图片数
        captured_images = self.mission_time // 3 if current_state == 3 else 0
        
        # 已飞行长度（米）
        flown_distance = self.mission_time * 20 if current_state > 1 else 0
        
        # 总航线长度（米）- 固定值，可根据需要调整
        total_distance = 1000
        
        # 发布任务数据
        self.mission_state_pub.publish(Int32(data=current_state))
        self.flight_time_pub.publish(Int32(data=flight_time))
        self.remaining_distance_pub.publish(Int32(data=remaining_distance))
        self.captured_images_pub.publish(Int32(data=captured_images))
        self.flown_distance_pub.publish(Int32(data=flown_distance))
        self.total_distance_pub.publish(Int32(data=total_distance))
        
        print(f"发布任务数据: 状态={current_state}, 时间={flight_time}s, 剩余={remaining_distance}m, 已飞行={flown_distance}m, 总长={total_distance}m, 图片={captured_images}")
        
    def publish_location_data(self):
        """发布定位状态模拟数据"""
        # 定位状态模拟 (0:无定位, 1:卫导定位, 2:视觉定位)
        location_status = [0, 1, 2][(self.mission_time // 15) % 3]
        
        # 视觉子状态模拟 (0:未初始化, 1:单应, 2:ORB SLAM)
        visual_sub_status = [0, 1, 2][(self.mission_time // 8) % 3] if location_status == 2 else 0
        
        # 卫星数量
        satellite_count = 8 + (self.mission_time % 8) if location_status >= 1 else 0
        
        # 定位精度（米）
        base_accuracy = 0.5 if location_status == 2 else (2.0 if location_status == 1 else 10.0)
        accuracy = base_accuracy + (self.mission_time % 3) * 0.1
        
        # 发布定位数据
        self.location_main_status_pub.publish(Int32(data=location_status))
        self.location_visual_sub_status_pub.publish(Int32(data=visual_sub_status))
        self.satellite_count_pub.publish(Int32(data=satellite_count))
        self.location_accuracy_pub.publish(Float32(data=accuracy))
        
        # 状态名称
        status_names = ["无定位", "卫导定位", "视觉定位"]
        visual_names = ["未初始化", "单应", "ORB SLAM"]
        main_name = status_names[location_status]
        visual_name = visual_names[visual_sub_status] if location_status == 2 else "无"
        
        print(f"发布定位数据: {main_name}-{visual_name}, 卫星={satellite_count}, 精度={accuracy:.1f}m")
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            # 发布所有数据
            self.publish_mavros_state()
            self.publish_mission_data()
            self.publish_location_data()
            
            print("-" * 50)
            
            # 更新时间计数器
            self.mission_time += 1
            
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = MAVROSPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        print("[INFO] MAVROS发布器已停止")
        pass 