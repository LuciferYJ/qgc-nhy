#!/usr/bin/env python3
"""
MAVROS消息发布器 - 发送MAVROS state消息
"""

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Header

class MAVROSPublisher:
    def __init__(self):
        rospy.init_node('mavros_publisher', anonymous=True)
        
        # 发布器
        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=10)
        
        rospy.loginfo("[INFO] MAVROS消息发布器已启动")
        
    def publish_state(self):
        """发布MAVROS状态消息"""
        state_msg = State()
        state_msg.header = Header()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = ""
        
        state_msg.connected = True
        state_msg.armed = False
        state_msg.guided = True
        state_msg.manual_input = False
        state_msg.mode = "OFFBOARD"
        state_msg.system_status = 4  # MAV_STATE_ACTIVE
        
        self.state_pub.publish(state_msg)
        print(f"发送MAVROS state: connected={state_msg.connected}, armed={state_msg.armed}, mode={state_msg.mode}")
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = MAVROSPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[INFO] MAVROS消息发布器已停止")
    except Exception as e:
        rospy.logerr(f"[ERROR] MAVROS消息发布器错误: {e}") 