/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "CustomStatusReceiver.h"
#include "CustomCommandSender.h"
#include <QDebug>
#include <QDateTime>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 静态成员初始化
CustomStatusReceiver* CustomStatusReceiver::_instance = nullptr;

CustomStatusReceiver::CustomStatusReceiver(QObject *parent)
    : QObject(parent)
    , _udpLink(nullptr)
    , _connected(false)
    , _latitude(0.0)
    , _longitude(0.0)
    , _altitude(0.0f)
    , _yaw(0.0f)
    , _pitch(0.0f)
    , _roll(0.0f)
    , _lastMessageTime(0)
    , _timeoutTimer(nullptr)
{
    // 初始化MAVLink状态
    memset(&_mavlinkStatus, 0, sizeof(_mavlinkStatus));
    memset(&_mavlinkMessage, 0, sizeof(_mavlinkMessage));
    
    // 创建超时检测定时器
    _timeoutTimer = new QTimer(this);
    _timeoutTimer->setInterval(1000); // 每秒检查一次
    connect(_timeoutTimer, &QTimer::timeout, this, &CustomStatusReceiver::_checkConnectionTimeout);
    
    qDebug() << "CustomStatusReceiver: 初始化完成";
}

CustomStatusReceiver::~CustomStatusReceiver()
{
    stopReceiving();
    if (_timeoutTimer) {
        _timeoutTimer->stop();
    }
    qDebug() << "CustomStatusReceiver: 析构完成";
}

CustomStatusReceiver* CustomStatusReceiver::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine)
{
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    return CustomStatusReceiver::instance();
}

CustomStatusReceiver* CustomStatusReceiver::instance()
{
    if (!_instance) {
        _instance = new CustomStatusReceiver();
    }
    return _instance;
}

void CustomStatusReceiver::startReceiving()
{
    // 获取或创建UDP链接实例
    if (!_udpLink) {
        // 首先尝试获取现有的UDP链接实例
        _udpLink = UdpCommandLink::instance();
        
        // 检查UDP链接是否已正确初始化
        if (!_udpLink || !_udpLink->isInitialized()) {
            qDebug() << "CustomStatusReceiver: UDP链接未初始化，使用默认配置初始化";
            
            // 使用默认配置初始化UDP链接
            const QString DEFAULT_REMOTE_HOST = "127.0.0.1";
            const quint16 DEFAULT_REMOTE_PORT = 8888;  // 发送到远端8888端口
            const quint16 DEFAULT_RECEIVE_PORT = 7777; // 接收7777端口的数据
            
            _udpLink = UdpCommandLink::instance(DEFAULT_REMOTE_HOST, DEFAULT_REMOTE_PORT, DEFAULT_RECEIVE_PORT);
            
            if (!_udpLink || !_udpLink->initialize()) {
                qWarning() << "CustomStatusReceiver: 无法初始化UDP链接";
                return;
            }
        }
        
        // 连接数据接收信号
        connect(_udpLink, &UdpCommandLink::dataReceived, 
                this, &CustomStatusReceiver::_handleReceivedData);
    }
    
    // 启动超时检测定时器
    _timeoutTimer->start();
    
    qDebug() << "CustomStatusReceiver: 开始接收数据，监听端口:" << _udpLink->getReceiveLocalPort();
}

void CustomStatusReceiver::stopReceiving()
{
    // 停止超时检测定时器
    if (_timeoutTimer) {
        _timeoutTimer->stop();
    }
    
    if (_udpLink) {
        disconnect(_udpLink, &UdpCommandLink::dataReceived, 
                   this, &CustomStatusReceiver::_handleReceivedData);
    }
    
    // 更新连接状态
    if (_connected) {
        _connected = false;
        emit connectedChanged();
    }
    
    qDebug() << "CustomStatusReceiver: 停止接收数据";
}

void CustomStatusReceiver::_handleReceivedData(const QByteArray &data)
{
    // 解析MAVLink消息
    for (int i = 0; i < data.size(); ++i) {
        uint8_t c = static_cast<uint8_t>(data[i]);
        
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &_mavlinkMessage, &_mavlinkStatus)) {
            // 成功解析到一个完整的MAVLink消息
            _parseMavlinkMessage(_mavlinkMessage);
        }
    }
}

void CustomStatusReceiver::_parseMavlinkMessage(const mavlink_message_t &message)
{
    // 处理不同类型的MAVLink消息
    if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        _handleGlobalPositionInt(message);
    } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        _handleAttitude(message);
    }
}

void CustomStatusReceiver::_handleGlobalPositionInt(const mavlink_message_t &message)
{
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(&message, &pos);
    
    // 更新连接状态
    if (!_connected) {
        _connected = true;
        emit connectedChanged();
        qDebug() << "CustomStatusReceiver: 开始接收位置数据";
    }
    
    _lastMessageTime = QDateTime::currentMSecsSinceEpoch();
    
    // 更新位置信息
    double newLat = pos.lat / 1e7;          // 转换为度
    double newLon = pos.lon / 1e7;          // 转换为度
    float newAlt = pos.relative_alt / 1000.0f;  // 相对高度，转换为米
    
    bool locationChanged = false;
    
    if (fabs(newLat - _latitude) > 1e-7 || fabs(newLon - _longitude) > 1e-7) {
        _latitude = newLat;
        _longitude = newLon;
        locationChanged = true;
    }
    
    if (fabs(newAlt - _altitude) > 0.1f) {
        _altitude = newAlt;
        locationChanged = true;
    }
    
    if (locationChanged) {
        emit positionChanged();
    }
}

void CustomStatusReceiver::_handleAttitude(const mavlink_message_t &message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);
    
    // 更新连接状态
    if (!_connected) {
        _connected = true;
        emit connectedChanged();
        qDebug() << "CustomStatusReceiver: 开始接收姿态数据";
    }
    
    _lastMessageTime = QDateTime::currentMSecsSinceEpoch();
    
    // 将弧度转换为度
    float newYaw = attitude.yaw * 180.0f / M_PI;
    float newPitch = attitude.pitch * 180.0f / M_PI;
    float newRoll = attitude.roll * 180.0f / M_PI;
    
    // 标准化偏航角到0-360度范围
    if (newYaw < 0) {
        newYaw += 360.0f;
    }
    
    bool attitudeChanged = false;
    
    if (fabs(newYaw - _yaw) > 0.1f || fabs(newPitch - _pitch) > 0.1f || fabs(newRoll - _roll) > 0.1f) {
        _yaw = newYaw;
        _pitch = newPitch;
        _roll = newRoll;
        attitudeChanged = true;
    }
    
    if (attitudeChanged) {
        emit attitudeDataChanged();
    }
}

void CustomStatusReceiver::_checkConnectionTimeout()
{
    if (!_connected) {
        return; // 如果本来就没连接，不需要检查
    }
    
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    qint64 timeSinceLastMessage = currentTime - _lastMessageTime;
    
    if (timeSinceLastMessage > CONNECTION_TIMEOUT_MS) {
        // 超时，断开连接
        _connected = false;
        emit connectedChanged();
        qDebug() << "CustomStatusReceiver: 连接超时，已断开连接。最后消息时间:" 
                 << timeSinceLastMessage << "ms前";
    }
} 