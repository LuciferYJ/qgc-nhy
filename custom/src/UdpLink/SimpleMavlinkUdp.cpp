#include "SimpleMavlinkUdp.h"
#include <QDebug>
#include <QDateTime>
#include <QtMath>
#include <QJsonParseError>

// 静态成员初始化
SimpleMavlinkUdp* SimpleMavlinkUdp::_instance = nullptr;

SimpleMavlinkUdp::SimpleMavlinkUdp(QObject *parent)
    : QObject(parent)
    , _socket(nullptr)
    , _listenPort(7777)
    , _connected(false)
    , _lastHeartbeatTime(0)
    , _timeoutTimer(nullptr)
    , _missionStatus()      // 使用默认构造函数
    , _locationStatus()     // 使用默认构造函数
{
    // 创建超时检测定时器
    _timeoutTimer = new QTimer(this);
    _timeoutTimer->setInterval(1000); // 每秒检查一次
    connect(_timeoutTimer, &QTimer::timeout, this, &SimpleMavlinkUdp::_checkConnectionTimeout);
    
    // qDebug() << "SimpleMavlinkUdp: JSON UDP通信初始化完成";
}

SimpleMavlinkUdp::~SimpleMavlinkUdp()
{
    stop();
    // qDebug() << "SimpleMavlinkUdp: 析构完成";
}

SimpleMavlinkUdp* SimpleMavlinkUdp::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine)
{
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    return SimpleMavlinkUdp::instance();
}

SimpleMavlinkUdp* SimpleMavlinkUdp::instance()
{
    if (!_instance) {
        _instance = new SimpleMavlinkUdp();
    }
    return _instance;
}

bool SimpleMavlinkUdp::start(quint16 listenPort)
{
    if (_socket) {
        // qDebug() << "SimpleMavlinkUdp: 已经启动，先停止再重新启动";
        stop();
    }
    
    _listenPort = listenPort;
    
    // 创建UDP套接字
    _socket = new QUdpSocket(this);
    
    // 绑定到指定端口
    if (!_socket->bind(QHostAddress::Any, _listenPort)) {
        // qWarning() << "SimpleMavlinkUdp: 无法绑定端口" << _listenPort << ":" << _socket->errorString();
        delete _socket;
        _socket = nullptr;
        return false;
    }
    
    // 连接数据接收信号
    connect(_socket, &QUdpSocket::readyRead, this, &SimpleMavlinkUdp::_readPendingDatagrams);
    
    // 启动超时检测
    _timeoutTimer->start();
    
    // qDebug() << "SimpleMavlinkUdp: 启动成功，监听端口:" << _listenPort;
    return true;
}

void SimpleMavlinkUdp::stop()
{
    // 停止超时检测
    if (_timeoutTimer) {
        _timeoutTimer->stop();
    }
    
    // 关闭套接字
    if (_socket) {
        _socket->close();
        delete _socket;
        _socket = nullptr;
    }
    
    // 更新连接状态
    if (_connected) {
        _connected = false;
        emit connectedChanged();
    }
    
    // qDebug() << "SimpleMavlinkUdp: 已停止";
}

// QML接口实现
bool SimpleMavlinkUdp::sendJson(int messageType, const QJsonObject& data)
{
    return _sendJsonMessage(static_cast<MessageType>(messageType), data);
}

bool SimpleMavlinkUdp::sendHeartbeat()
{
    HeartbeatData data;
    return sendHeartbeat(data);
}

bool SimpleMavlinkUdp::sendTakeoffCommand(const QString& missionUuid, const QString& action, float altitude)
{
    TakeoffCommand cmd;
    cmd.action = action;
    cmd.missionUuid = missionUuid;
    cmd.altitude = altitude;
    cmd.autoArm = true;
    
    return sendTakeoffCommand(cmd);
}

bool SimpleMavlinkUdp::sendMissionStatus(const QJsonObject& data)
{
    return _sendJsonMessage(MessageType::MISSION_STATUS, data);
}

bool SimpleMavlinkUdp::sendLocationStatus(const QJsonObject& data)
{
    return _sendJsonMessage(MessageType::LOCATION_STATUS, data);
}

// 结构体发送接口实现
bool SimpleMavlinkUdp::sendHeartbeat(const HeartbeatData& data)
{
    return _sendJsonMessage(MessageType::HEARTBEAT, data.toJson());
}

bool SimpleMavlinkUdp::sendTakeoffCommand(const TakeoffCommand& cmd)
{
    return _sendJsonMessage(MessageType::TAKEOFF_COMMAND, cmd.toJson());
}

bool SimpleMavlinkUdp::sendMissionStatus(const MissionStatus& status)
{
    return _sendJsonMessage(MessageType::MISSION_STATUS, status.toJson());
}

bool SimpleMavlinkUdp::sendLocationStatus(const LocationStatus& status)
{
    return _sendJsonMessage(MessageType::LOCATION_STATUS, status.toJson());
}

bool SimpleMavlinkUdp::sendAckResponse(const AckResponse& ack)
{
    return _sendJsonMessage(MessageType::ACK_RESPONSE, ack.toJson());
}

QString SimpleMavlinkUdp::_createJsonMessage(MessageType type, const QJsonObject& data)
{
    QJsonObject message;
    message["type"] = static_cast<int>(type);  // 直接使用数字类型
    message["timestamp"] = QDateTime::currentMSecsSinceEpoch();
    message["data"] = data;
    
    QJsonDocument doc(message);
    return doc.toJson(QJsonDocument::Compact);
}

bool SimpleMavlinkUdp::_sendJsonMessage(MessageType type, const QJsonObject& data)
{
    if (!_socket) {
        // qWarning() << "SimpleMavlinkUdp: UDP未启动，无法发送JSON消息";
        return false;
    }
    
    QString jsonStr = _createJsonMessage(type, data);
    QByteArray jsonData = jsonStr.toUtf8();
    
    // 发送到本地8888端口（假设无人机在此端口监听）
    qint64 bytesSent = _socket->writeDatagram(jsonData, QHostAddress::LocalHost, 8888);
    
    bool success = (bytesSent > 0);
    // qDebug() << "SimpleMavlinkUdp: 发送JSON消息类型" << static_cast<int>(type) << "结果:" << (success ? "成功" : "失败");
    // qDebug() << "内容:" << jsonStr;
    
    return success;
}

void SimpleMavlinkUdp::_readPendingDatagrams()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(_socket->pendingDatagramSize());
        
        QHostAddress sender;
        quint16 senderPort;
        
        _socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        
        // 解析JSON消息
        QJsonParseError parseError;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(datagram, &parseError);
        
        if (parseError.error == QJsonParseError::NoError && jsonDoc.isObject()) {
            // 成功解析为JSON消息
            QJsonObject jsonMessage = jsonDoc.object();
            _parseJsonMessage(jsonMessage);
        } else {
            // qWarning() << "SimpleMavlinkUdp: 收到无效的JSON数据:" << parseError.errorString();
            // qDebug() << "原始数据:" << datagram;
        }
    }
}

void SimpleMavlinkUdp::_parseJsonMessage(const QJsonObject& message)
{
    // 验证基本格式
    if (!message.contains("type") || !message.contains("data")) {
        // qWarning() << "SimpleMavlinkUdp: JSON消息格式错误，缺少type或data字段";
        return;
    }
    
    int typeInt = message["type"].toInt();
    MessageType msgType = static_cast<MessageType>(typeInt);  // 直接转换数字类型
    QJsonObject data = message["data"].toObject();
    
    // qDebug() << "SimpleMavlinkUdp: 收到JSON消息类型:" << typeInt;
    
    // 根据消息类型处理
    switch (msgType) {
    case MessageType::HEARTBEAT:
    {
        // 更新最后心跳时间
        _lastHeartbeatTime = QDateTime::currentMSecsSinceEpoch();
        
        // 接收到心跳消息认为连接正常
        if (!_connected) {
            _connected = true;
            emit connectedChanged();
            // qDebug() << "SimpleMavlinkUdp: 收到JSON心跳消息，连接已建立";
        }
        
        // 解析心跳数据并发送信号
        HeartbeatData heartbeat = HeartbeatData::fromJson(data);
        emit heartbeatReceived(heartbeat);
        break;
    }
    case MessageType::TAKEOFF_COMMAND:
    {
        TakeoffCommand cmd = TakeoffCommand::fromJson(data);
        emit takeoffCommandReceived(cmd);
        // qDebug() << "SimpleMavlinkUdp: 收到起飞命令 -" << cmd.action << "UUID:" << cmd.missionUuid;
        break;
    }
    case MessageType::MISSION_STATUS:
    {
        MissionStatus newStatus = MissionStatus::fromJson(data);
        if (_missionStatus != newStatus) {
            _missionStatus = newStatus;
            emit missionStatusChanged();
            emit missionStatusReceived(_missionStatus);
        }
        // qDebug() << "SimpleMavlinkUdp: 更新任务状态 - 状态:" << _missionStatus.missionState 
        //          << "时间:" << _missionStatus.flightTime << "距离:" << _missionStatus.remainingDistance 
        //          << "图片:" << _missionStatus.capturedImages;
        break;
    }
    case MessageType::LOCATION_STATUS:
    {
        LocationStatus newStatus = LocationStatus::fromJson(data);
        if (_locationStatus != newStatus) {
            _locationStatus = newStatus;
            emit locationStatusChanged();
            emit locationStatusReceived(_locationStatus);
        }
        // qDebug() << "SimpleMavlinkUdp: 更新定位状态 - 主状态:" << _locationStatus.locationStatus 
        //          << "视觉:" << _locationStatus.visualSubStatus << "卫星:" << _locationStatus.satelliteCount 
        //          << "精度:" << _locationStatus.accuracy;
        break;
    }
    case MessageType::ACK_RESPONSE:
    {
        AckResponse ack = AckResponse::fromJson(data);
        emit commandAckReceived(ack.success, ack.message);
        emit ackResponseReceived(ack);
        // qDebug() << "SimpleMavlinkUdp: 收到命令确认 -" << (ack.success ? "成功" : "失败") << ":" << ack.message;
        break;
    }
    case MessageType::ERROR:
        // qWarning() << "SimpleMavlinkUdp: 收到错误消息:" << data;
        break;
        
    default:
        // qDebug() << "SimpleMavlinkUdp: 未处理的JSON消息类型:" << typeInt;
        break;
    }
    
    // 发送通用JSON消息信号
    emit jsonReceived(static_cast<int>(msgType), data);
}

void SimpleMavlinkUdp::_checkConnectionTimeout()
{
    if (!_connected) {
        return; // 如果本来就没连接，不需要检查
    }
    
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    qint64 timeSinceLastMessage = currentTime - _lastHeartbeatTime;
    
    if (timeSinceLastMessage > CONNECTION_TIMEOUT_MS) {
        // 超时，断开连接
        _connected = false;
        emit connectedChanged();
        // qDebug() << "SimpleMavlinkUdp: 连接超时，已断开连接。最后消息时间:" 
        //          << timeSinceLastMessage << "ms前";
    }
} 