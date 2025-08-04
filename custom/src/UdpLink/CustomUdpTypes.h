#pragma once

#include <QObject>
#include <QJsonObject>
#include <QString>
#include <QtMath>

/**
 * @brief 自定义UDP通信的数据类型定义
 */

// JSON消息类型枚举
enum class MessageType {
    HEARTBEAT = 0,          // 心跳消息
    TAKEOFF_COMMAND,        // 起飞/结束命令  
    SET_HOME_COMMAND,       // 设置Home命令
    MISSION_STATUS,         // 任务状态数据
    LOCATION_STATUS,        // 定位状态数据
    ACK_RESPONSE,           // 命令确认响应
    ERROR                   // 错误消息
};

/**
 * @brief 任务状态数据结构
 */
struct MissionStatus {
    int missionState = 0;           // 任务状态：0=空闲, 1=就绪, 2=起飞, 3=巡航, 4=返航, 5=降落
    int flightTime = 0;             // 飞行时间（秒）
    int remainingDistance = 0;      // 剩余航程（米）
    int capturedImages = 0;         // 采集图片张数
    int flownDistance = 0;          // 已飞行长度（米）
    int totalDistance = 0;          // 总航线长度（米）
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["mission_state"] = missionState;
        obj["flight_time"] = flightTime;
        obj["remaining_distance"] = remainingDistance;
        obj["captured_images"] = capturedImages;
        obj["flown_distance"] = flownDistance;
        obj["total_distance"] = totalDistance;
        return obj;
    }
    
    // 从JSON对象构造
    static MissionStatus fromJson(const QJsonObject& obj) {
        MissionStatus status;
        status.missionState = obj["mission_state"].toInt();
        status.flightTime = obj["flight_time"].toInt();
        status.remainingDistance = obj["remaining_distance"].toInt();
        status.capturedImages = obj["captured_images"].toInt();
        status.flownDistance = obj["flown_distance"].toInt();
        status.totalDistance = obj["total_distance"].toInt();
        return status;
    }
    
    // 比较操作符
    bool operator==(const MissionStatus& other) const {
        return missionState == other.missionState &&
               flightTime == other.flightTime &&
               remainingDistance == other.remainingDistance &&
               capturedImages == other.capturedImages;
    }
    
    bool operator!=(const MissionStatus& other) const {
        return !(*this == other);
    }
};

/**
 * @brief 定位状态数据结构
 */
struct LocationStatus {
    int locationStatus = 0;         // 主定位状态：0=无定位, 1=卫导定位, 2=视觉定位
    int visualSubStatus = 0;        // 视觉子状态：0=未初始化, 1=单应, 2=ORB SLAM
    int satelliteCount = 0;         // 卫星数量
    float accuracy = 0.0f;          // 定位精度（米）
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["location_status"] = locationStatus;
        obj["visual_sub_status"] = visualSubStatus;
        obj["satellite_count"] = satelliteCount;
        obj["accuracy"] = accuracy;
        return obj;
    }
    
    // 从JSON对象构造
    static LocationStatus fromJson(const QJsonObject& obj) {
        LocationStatus status;
        status.locationStatus = obj["location_status"].toInt();
        status.visualSubStatus = obj["visual_sub_status"].toInt();
        status.satelliteCount = obj["satellite_count"].toInt();
        status.accuracy = static_cast<float>(obj["accuracy"].toDouble());
        return status;
    }
    
    // 比较操作符
    bool operator==(const LocationStatus& other) const {
        return locationStatus == other.locationStatus &&
               visualSubStatus == other.visualSubStatus &&
               satelliteCount == other.satelliteCount &&
               qAbs(accuracy - other.accuracy) < 0.001f;
    }
    
    bool operator!=(const LocationStatus& other) const {
        return !(*this == other);
    }
};

/**
 * @brief 心跳数据结构
 */
struct HeartbeatData {
    int systemId = 1;
    int componentId = 1;
    QString status = "active";
    bool armed = false;
    QString mode = "OFFBOARD";
    bool connected = true;
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["system_id"] = systemId;
        obj["component_id"] = componentId;
        obj["status"] = status;
        obj["armed"] = armed;
        obj["mode"] = mode;
        obj["connected"] = connected;
        return obj;
    }
    
    // 从JSON对象构造
    static HeartbeatData fromJson(const QJsonObject& obj) {
        HeartbeatData data;
        data.systemId = obj["system_id"].toInt();
        data.componentId = obj["component_id"].toInt();
        data.status = obj["status"].toString();
        data.armed = obj["armed"].toBool();
        data.mode = obj["mode"].toString();
        data.connected = obj["connected"].toBool();
        return data;
    }
};

/**
 * @brief 起飞命令数据结构
 */
struct TakeoffCommand {
    QString action;                 // "takeoff" | "land"
    QString missionUuid;            // 任务UUID
    float altitude = 50.0f;         // 高度
    bool autoArm = true;            // 自动解锁
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["action"] = action;
        obj["mission_uuid"] = missionUuid;
        obj["altitude"] = altitude;
        
        QJsonObject params;
        params["auto_arm"] = autoArm;
        obj["params"] = params;
        
        return obj;
    }
    
    // 从JSON对象构造
    static TakeoffCommand fromJson(const QJsonObject& obj) {
        TakeoffCommand cmd;
        cmd.action = obj["action"].toString();
        cmd.missionUuid = obj["mission_uuid"].toString();
        cmd.altitude = static_cast<float>(obj["altitude"].toDouble());
        
        QJsonObject params = obj["params"].toObject();
        cmd.autoArm = params["auto_arm"].toBool();
        
        return cmd;
    }
};

/**
 * @brief 命令确认响应数据结构
 */
struct AckResponse {
    bool success = false;
    QString message;
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["success"] = success;
        obj["message"] = message;
        return obj;
    }
    
    // 从JSON对象构造
    static AckResponse fromJson(const QJsonObject& obj) {
        AckResponse ack;
        ack.success = obj["success"].toBool();
        ack.message = obj["message"].toString();
        return ack;
    }
}; 