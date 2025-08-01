#!/usr/bin/env python3
"""
MAVROS到QGC桥接器 - 双向通信：
1. 接收MAVROS消息，转换为结构体化JSON并发送给QGC  
2. 接收QGC指令，验证后转换为ROS话题发出
"""

import rospy
import json
import time
import socket
import threading
import uuid
from mavros_msgs.msg import State
from std_msgs.msg import Int32, Float32, String, Bool
from geometry_msgs.msg import PoseStamped

class MAVROSToQGCBridge:
    def __init__(self):
        # QGC连接配置
        self.qgc_host = "127.0.0.1"
        self.qgc_send_port = 7777  # 发送数据到QGC的端口
        self.qgc_recv_port = 8888  # 接收QGC指令的端口
        
        # 消息类型常量 (与CustomUdpTypes.h中的MessageType枚举对应)
        self.MSG_HEARTBEAT = 0
        self.MSG_TAKEOFF_COMMAND = 1
        self.MSG_SET_HOME_COMMAND = 2
        self.MSG_MISSION_STATUS = 3
        self.MSG_LOCATION_STATUS = 4
        self.MSG_ACK_RESPONSE = 5
        self.MSG_ERROR = 6
        
        # 创建UDP套接字
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 发送到QGC
        self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 接收QGC指令
        
        # 绑定接收端口
        self.recv_socket.bind(('0.0.0.0', self.qgc_recv_port))
        
        # ROS节点初始化
        rospy.init_node('mavros_to_qgc_bridge', anonymous=True)
        
        # ROS发布器 - 发送指令到MAVROS/ROS系统
        self.takeoff_pub = rospy.Publisher('/qgc/takeoff_command', String, queue_size=10)
        self.set_home_pub = rospy.Publisher('/qgc/set_home_command', PoseStamped, queue_size=10)
        self.mission_control_pub = rospy.Publisher('/qgc/mission_control', String, queue_size=10)
        
        # 任务状态结构数据
        self.mission_status = {
            "mission_state": 0,
            "flight_time": 0,
            "remaining_distance": 0,
            "captured_images": 0
        }
        
        # 定位状态结构数据
        self.location_status = {
            "location_status": 0,
            "visual_sub_status": 0,
            "satellite_count": 0,
            "accuracy": 0.0
        }
        
        # 指令验证配置
        self.valid_takeoff_actions = ["takeoff", "land", "return", "abort"]
        self.max_takeoff_altitude = 150.0  # 最大起飞高度限制
        self.min_takeoff_altitude = 1.0    # 最小起飞高度限制
        
        # ROS订阅器 - MAVROS状态
        rospy.Subscriber('/mavros/state', State, self._mavros_state_callback)
        
        # ROS订阅器 - 任务数据（整体结构）
        rospy.Subscriber('/mission/state', Int32, self._mission_state_callback)
        rospy.Subscriber('/mission/flight_time', Int32, self._flight_time_callback)
        rospy.Subscriber('/mission/remaining_distance', Int32, self._remaining_distance_callback)
        rospy.Subscriber('/mission/captured_images', Int32, self._captured_images_callback)
        
        # ROS订阅器 - 定位数据（整体结构）
        rospy.Subscriber('/location/main_status', Int32, self._location_main_status_callback)
        rospy.Subscriber('/location/visual_sub_status', Int32, self._location_visual_sub_status_callback)
        rospy.Subscriber('/location/satellite_count', Int32, self._satellite_count_callback)
        rospy.Subscriber('/location/accuracy', Float32, self._location_accuracy_callback)
        
        # 启动QGC指令接收线程
        self.recv_thread = threading.Thread(target=self._qgc_command_receiver, daemon=True)
        self.recv_thread.start()
        
        # print("[INFO] JSON桥接器初始化完成，双向通信已建立")
        # print(f"[INFO] 发送端口: {self.qgc_send_port}, 接收端口: {self.qgc_recv_port}")
        
    def _send_json_message(self, msg_type, data):
        """发送结构体化JSON消息到QGC"""
        try:
            message = {
                "type": msg_type,  # 直接使用数字类型
                "timestamp": int(time.time() * 1000),
                "data": data
            }
            
            json_str = json.dumps(message)
            self.send_socket.sendto(json_str.encode('utf-8'), (self.qgc_host, self.qgc_send_port))
            
            # print(f"发送消息类型{msg_type}: {json_str}")
            return True
            
        except Exception as e:
            # print(f"[ERROR] 发送JSON消息失败: {e}")
            return False
    
    def _send_ack_response(self, success, message, command_uuid=None):
        """发送ACK响应到QGC"""
        ack_data = {
            "success": success,
            "message": message,
            "command_uuid": command_uuid or "",
            "timestamp": int(time.time() * 1000)
        }
        return self._send_json_message(self.MSG_ACK_RESPONSE, ack_data)
    
    def _validate_takeoff_command(self, cmd_data):
        """验证起飞指令"""
        try:
            action = cmd_data.get("action", "")
            altitude = cmd_data.get("altitude", 0)
            mission_uuid = cmd_data.get("mission_uuid", "")
            params = cmd_data.get("params", {})
            
            # 验证动作类型
            if action not in self.valid_takeoff_actions:
                return False, f"无效的动作类型: {action}"
            
            # 验证起飞高度
            if action == "takeoff":
                if altitude < self.min_takeoff_altitude or altitude > self.max_takeoff_altitude:
                    return False, f"起飞高度超出范围 ({self.min_takeoff_altitude}-{self.max_takeoff_altitude}m): {altitude}m"
            
            # 验证UUID格式
            if mission_uuid:
                try:
                    uuid.UUID(mission_uuid)
                except ValueError:
                    return False, f"无效的UUID格式: {mission_uuid}"
            
            return True, "指令验证通过"
            
        except Exception as e:
            return False, f"指令验证错误: {str(e)}"
    
    def _process_takeoff_command(self, cmd_data):
        """处理起飞指令"""
        # 验证指令
        is_valid, validation_msg = self._validate_takeoff_command(cmd_data)
        command_uuid = cmd_data.get("mission_uuid", "")
        
        if not is_valid:
            # print(f"[ERROR] 起飞指令验证失败: {validation_msg}")
            self._send_ack_response(False, validation_msg, command_uuid)
            return
        
        try:
            # 解析指令数据 (注意params子对象)
            action = cmd_data.get("action", "")
            altitude = cmd_data.get("altitude", 0)
            params = cmd_data.get("params", {})
            auto_arm = params.get("auto_arm", False)
            
            ros_command = {
                "action": action,
                "altitude": altitude,
                "auto_arm": auto_arm,
                "mission_uuid": command_uuid,
                "timestamp": time.time()
            }
            
            # 发布到ROS话题
            ros_msg = String()
            ros_msg.data = json.dumps(ros_command)
            self.takeoff_pub.publish(ros_msg)
            
            # 发送成功ACK
            success_msg = f"起飞指令已发送: {action} at {altitude}m (auto_arm: {auto_arm})"
            # print(f"[INFO] {success_msg}")
            self._send_ack_response(True, success_msg, command_uuid)
            
        except Exception as e:
            error_msg = f"处理起飞指令时发生错误: {str(e)}"
            # print(f"[ERROR] {error_msg}")
            self._send_ack_response(False, error_msg, command_uuid)
    
    def _process_set_home_command(self, cmd_data):
        """处理设置Home指令"""
        try:
            command_uuid = cmd_data.get("command_uuid", "")
            use_current_position = cmd_data.get("use_current_position", False)
            
            # 正确解析坐标数据（从coordinate子对象中）
            coordinate = cmd_data.get("coordinate", {})
            latitude = coordinate.get("latitude", 0.0)
            longitude = coordinate.get("longitude", 0.0)
            altitude = coordinate.get("altitude", 0.0)
            
            # 验证坐标范围
            if not (-90 <= latitude <= 90):
                error_msg = f"纬度超出范围: {latitude}"
                self._send_ack_response(False, error_msg, command_uuid)
                return
                
            if not (-180 <= longitude <= 180):
                error_msg = f"经度超出范围: {longitude}"
                self._send_ack_response(False, error_msg, command_uuid)
                return
            
            # 构建ROS PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = longitude  # 简化处理，实际应用需要坐标转换
            pose_msg.pose.position.y = latitude
            pose_msg.pose.position.z = altitude
            
            # 发布到ROS话题
            self.set_home_pub.publish(pose_msg)
            
            # 发送成功ACK（使用正确解析的坐标）
            success_msg = f"Home位置已设置: ({latitude}, {longitude}, {altitude})"
            if use_current_position:
                success_msg += " (使用当前位置)"
            # print(f"[INFO] {success_msg}")
            self._send_ack_response(True, success_msg, command_uuid)
            
        except Exception as e:
            error_msg = f"处理设置Home指令时发生错误: {str(e)}"
            # print(f"[ERROR] {error_msg}")
            self._send_ack_response(False, error_msg, cmd_data.get("command_uuid", ""))
    
    def _qgc_command_receiver(self):
        """QGC指令接收线程"""
        # print(f"[INFO] 开始监听QGC指令，端口: {self.qgc_recv_port}")
        
        while True:
            try:
                data, addr = self.recv_socket.recvfrom(4096)
                # print(f"[DEBUG] 收到来自 {addr} 的数据: {data}")
                
                # 解析JSON消息
                message = json.loads(data.decode('utf-8'))
                msg_type = message.get("type")
                msg_data = message.get("data", {})
                
                # print(f"[INFO] 收到QGC指令类型: {msg_type}")
                
                # 根据消息类型处理
                if msg_type == self.MSG_TAKEOFF_COMMAND:
                    self._process_takeoff_command(msg_data)
                elif msg_type == self.MSG_SET_HOME_COMMAND:
                    self._process_set_home_command(msg_data)
                else:
                    # print(f"[WARNING] 未处理的指令类型: {msg_type}")
                    self._send_ack_response(False, f"未支持的指令类型: {msg_type}")
                    
            except json.JSONDecodeError as e:
                # print(f"[ERROR] JSON解析错误: {e}")
                self._send_ack_response(False, f"JSON格式错误: {str(e)}")
            except Exception as e:
                # print(f"[ERROR] 指令接收错误: {e}")
                pass
    
    def _mavros_state_callback(self, msg):
        """处理MAVROS状态消息"""
        # 发送结构体化心跳消息
        heartbeat_data = {
            "system_id": 1,
            "component_id": 1,
            "status": "active" if msg.connected else "disconnected",
            "armed": msg.armed,
            "mode": msg.mode,
            "connected": msg.connected
        }
        
        self._send_json_message(self.MSG_HEARTBEAT, heartbeat_data)
        
        # 发送完整的任务状态结构
        self._send_mission_status()
        
        # 发送完整的定位状态结构
        self._send_location_status()
        
        # print(f"发送心跳: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}")
        
    # ROS回调函数 - 任务数据（更新结构体）
    def _mission_state_callback(self, msg):
        self.mission_status["mission_state"] = msg.data
        
    def _flight_time_callback(self, msg):
        self.mission_status["flight_time"] = msg.data
        
    def _remaining_distance_callback(self, msg):
        self.mission_status["remaining_distance"] = msg.data
        
    def _captured_images_callback(self, msg):
        self.mission_status["captured_images"] = msg.data
        
    # ROS回调函数 - 定位数据（更新结构体）
    def _location_main_status_callback(self, msg):
        self.location_status["location_status"] = msg.data
        
    def _location_visual_sub_status_callback(self, msg):
        self.location_status["visual_sub_status"] = msg.data
        
    def _satellite_count_callback(self, msg):
        self.location_status["satellite_count"] = msg.data
        
    def _location_accuracy_callback(self, msg):
        self.location_status["accuracy"] = msg.data
        
    def _send_mission_status(self):
        """发送完整的任务状态结构"""
        self._send_json_message(self.MSG_MISSION_STATUS, self.mission_status.copy())
        
    def _send_location_status(self):
        """发送完整的定位状态结构"""
        try:
            # 调试：打印定位状态发送信息
            status_names = ["无定位", "卫导定位", "视觉定位"]
            visual_names = ["未初始化", "单应", "ORB SLAM"]
            
            main_status = self.location_status["location_status"]
            visual_status = self.location_status["visual_sub_status"]
            
            main_name = status_names[main_status] if main_status < len(status_names) else "未知"
            visual_name = visual_names[visual_status] if visual_status < len(visual_names) else "未知"
            
            self._send_json_message(self.MSG_LOCATION_STATUS, self.location_status.copy())
            
            # print(f"发送定位状态: {main_name}-{visual_name} (主:{main_status}, 子:{visual_status})")
            
        except Exception as e:
            # print(f"[ERROR] 发送定位状态数据失败: {e}")
            # import traceback
            # traceback.print_exc()
            pass
        
    def run(self):
        """主循环"""
        try:
            rospy.spin()
        finally:
            self.send_socket.close()
            self.recv_socket.close()

if __name__ == '__main__':
    try:
        bridge = MAVROSToQGCBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass 