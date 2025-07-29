#include "SimpleMavlinkUdp.h"
#include <QDebug>
#include <QDateTime>
#include <QtMath>

// 静态成员初始化
SimpleMavlinkUdp* SimpleMavlinkUdp::_instance = nullptr;

SimpleMavlinkUdp::SimpleMavlinkUdp(QObject *parent)
    : QObject(parent)
    , _socket(nullptr)
    , _listenPort(7777)
    , _connected(false)
    , _lastHeartbeatTime(0)
    , _timeoutTimer(nullptr)
    , _missionState(0)
    , _flightTime(0)
    , _remainingDistance(0)
    , _capturedImages(0)
    , _locationStatus(0)
    , _visualSubStatus(0)
    , _satelliteCount(0)
    , _locationAccuracy(0.0f)
{
    // 初始化MAVLink状态
    memset(&_mavlinkStatus, 0, sizeof(_mavlinkStatus));
    memset(&_mavlinkMessage, 0, sizeof(_mavlinkMessage));
    
    // 创建超时检测定时器
    _timeoutTimer = new QTimer(this);
    _timeoutTimer->setInterval(1000); // 每秒检查一次
    connect(_timeoutTimer, &QTimer::timeout, this, &SimpleMavlinkUdp::_checkConnectionTimeout);
    
    qDebug() << "SimpleMavlinkUdp: 初始化完成";
}

SimpleMavlinkUdp::~SimpleMavlinkUdp()
{
    stop();
    qDebug() << "SimpleMavlinkUdp: 析构完成";
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
        qDebug() << "SimpleMavlinkUdp: 已经启动，先停止再重新启动";
        stop();
    }
    
    _listenPort = listenPort;
    
    // 创建UDP套接字
    _socket = new QUdpSocket(this);
    
    // 绑定到指定端口
    if (!_socket->bind(QHostAddress::Any, _listenPort)) {
        qWarning() << "SimpleMavlinkUdp: 无法绑定端口" << _listenPort << ":" << _socket->errorString();
        delete _socket;
        _socket = nullptr;
        return false;
    }
    
    // 连接数据接收信号
    connect(_socket, &QUdpSocket::readyRead, this, &SimpleMavlinkUdp::_readPendingDatagrams);
    
    // 启动超时检测
    _timeoutTimer->start();
    
    qDebug() << "SimpleMavlinkUdp: 启动成功，监听端口:" << _listenPort;
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
    
    qDebug() << "SimpleMavlinkUdp: 已停止";
}

bool SimpleMavlinkUdp::sendCommand(int command, float param1, float param2, float param3, float param4)
{
    if (!_socket) {
        qWarning() << "SimpleMavlinkUdp: UDP未启动，无法发送命令";
        return false;
    }
    
    // 构造MAVLink命令消息
    mavlink_message_t message;
    mavlink_command_long_t cmd;
    
    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = 1;      // 目标系统ID
    cmd.target_component = 1;   // 目标组件ID
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;
    
    // 编码MAVLink消息
    mavlink_msg_command_long_encode(1, 1, &message, &cmd);
    
    // 转换为字节数组
    QByteArray buffer(MAVLINK_MAX_PACKET_LEN, 0);
    int len = mavlink_msg_to_send_buffer((uint8_t*)buffer.data(), &message);
    buffer.resize(len);
    
    // 发送到本地8888端口（假设无人机在此端口监听）
    qint64 bytesSent = _socket->writeDatagram(buffer, QHostAddress::LocalHost, 8888);
    
    bool success = (bytesSent > 0);
    qDebug() << "SimpleMavlinkUdp: 发送命令" << command << "结果:" << (success ? "成功" : "失败");
    
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
        
        // 解析MAVLink消息
        for (int i = 0; i < datagram.size(); ++i) {
            uint8_t c = static_cast<uint8_t>(datagram[i]);
            
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &_mavlinkMessage, &_mavlinkStatus)) {
                // 成功解析到一个完整的MAVLink消息
                _parseMavlinkMessage(_mavlinkMessage);
            }
        }
    }
}

void SimpleMavlinkUdp::_parseMavlinkMessage(const mavlink_message_t &message)
{
    // 处理心跳消息
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        // 更新最后心跳时间（只有心跳消息才更新）
        _lastHeartbeatTime = QDateTime::currentMSecsSinceEpoch();
        
        // 接收到心跳消息认为连接正常
        if (!_connected) {
            _connected = true;
            emit connectedChanged();
            qDebug() << "SimpleMavlinkUdp: 收到心跳消息，连接已建立";
        }
        
        qDebug() << "SimpleMavlinkUdp: 收到心跳消息";
    }
    // 处理任务状态数据消息
    else if (message.msgid == MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        mavlink_named_value_int_t named_value;
        mavlink_msg_named_value_int_decode(&message, &named_value);
        
        // 将name转换为字符串（确保以null结尾）
        char name_str[11];  // 最多10个字符 + null终止符
        memcpy(name_str, named_value.name, 10);
        name_str[10] = '\0';
        
        QString name = QString::fromUtf8(name_str);
        
        if (name == "MISSION_ST") {
            if (_missionState != named_value.value) {
                _missionState = named_value.value;
                emit missionStateChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到任务状态数据" << "\"" << name << "\"" << "=" << named_value.value;
        } else if (name == "FLIGHT_TM") {
            if (_flightTime != named_value.value) {
                _flightTime = named_value.value;
                emit flightTimeChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到任务状态数据" << "\"" << name << "\"" << "=" << named_value.value;
        } else if (name == "REMAIN_DST") {
            if (_remainingDistance != named_value.value) {
                _remainingDistance = named_value.value;
                emit remainingDistanceChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到任务状态数据" << "\"" << name << "\"" << "=" << named_value.value;
        } else if (name == "IMG_COUNT") {
            if (_capturedImages != named_value.value) {
                _capturedImages = named_value.value;
                emit capturedImagesChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到任务状态数据" << "\"" << name << "\"" << "=" << named_value.value;
        } else if (name == "LOC_ST") {
            // 位运算解析定位状态
            int new_location_status = named_value.value & 0xFF;          // 低8位：主定位状态
            int new_visual_sub_status = (named_value.value >> 8) & 0xFF; // 次8位：视觉子状态
            
            if (_locationStatus != new_location_status) {
                _locationStatus = new_location_status;
                emit locationStatusChanged();
            }
            if (_visualSubStatus != new_visual_sub_status) {
                _visualSubStatus = new_visual_sub_status;
                emit visualSubStatusChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到定位状态数据" << "\"" << name << "\"" << "=" << named_value.value 
                     << "(主状态:" << new_location_status << ", 子状态:" << new_visual_sub_status << ")";
        } else if (name == "SAT_COUNT") {
            if (_satelliteCount != named_value.value) {
                _satelliteCount = named_value.value;
                emit satelliteCountChanged();
            }
            qDebug() << "SimpleMavlinkUdp: 收到定位状态数据" << "\"" << name << "\"" << "=" << named_value.value;
        }
    }
    // 处理定位精度数据消息（浮点型）
    else if (message.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) {
        mavlink_named_value_float_t named_value_float;
        mavlink_msg_named_value_float_decode(&message, &named_value_float);
        
        // 将name转换为字符串（确保以null结尾）
        char name_str[11];  // 最多10个字符 + null终止符
        memcpy(name_str, named_value_float.name, 10);
        name_str[10] = '\0';
        
        QString name = QString::fromUtf8(name_str);
        
        if (name == "LOC_ACCUR") {
            if (qAbs(_locationAccuracy - named_value_float.value) > 0.001f) {
                _locationAccuracy = named_value_float.value;
                emit locationAccuracyChanged();
            }
        }
        
        qDebug() << "SimpleMavlinkUdp: 收到定位精度数据" << "\"" << name << "\"" << "=" << named_value_float.value;
    }
    
    // 这里可以根据需要添加其他消息类型的处理
    // 注意：其他消息不会影响连接状态判断
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
        qDebug() << "SimpleMavlinkUdp: 连接超时，已断开连接。最后消息时间:" 
                 << timeSinceLastMessage << "ms前";
    }
} 