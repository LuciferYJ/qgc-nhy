/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "CustomCommandSender.h"
#include <QDebug>

// 单例静态变量
CustomCommandSender* CustomCommandSender::_instance = nullptr;

// 默认网络配置
const QString DEFAULT_REMOTE_HOST = "127.0.0.1";
const quint16 DEFAULT_REMOTE_PORT = 8888;  // 发送到远端8888端口
const quint16 DEFAULT_RECEIVE_PORT = 7777; // 接收7777端口的数据

// QML单例创建函数
CustomCommandSender* CustomCommandSender::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine)
{
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    
    return instance();
}

// 获取单例实例
CustomCommandSender* CustomCommandSender::instance()
{
    if (!_instance) {
        _instance = new CustomCommandSender();
    }
    return _instance;
}

CustomCommandSender::CustomCommandSender(QObject *parent) : 
    QObject(parent)
{
    // 创建UdpCommandLink时传入发送和接收配置
    _udpLink = UdpCommandLink::instance(DEFAULT_REMOTE_HOST, DEFAULT_REMOTE_PORT, DEFAULT_RECEIVE_PORT);
    
    // 确保UDP链接已初始化
    if (!_udpLink->initialize()) {
        qWarning() << "CustomCommandSender: 无法初始化UDP链接";
    }
    
    // 连接数据接收信号
    connect(_udpLink, &UdpCommandLink::dataReceived, this, &CustomCommandSender::dataReceived);
    
    qDebug() << "CustomCommandSender: 初始化完成";
    qDebug() << "发送目标:" << DEFAULT_REMOTE_HOST << ":" << DEFAULT_REMOTE_PORT;
    qDebug() << "接收端口:" << DEFAULT_RECEIVE_PORT;
}

CustomCommandSender::~CustomCommandSender()
{
    // UdpCommandLink是单例，这里不需要删除它
    qDebug() << "CustomCommandSender: 析构";
}

bool CustomCommandSender::sendCommandByUdp(int componentId, int command, bool showError, 
                                          float param1, float param2, float param3,
                                          float param4, float param5, float param6, float param7)
{
    if (!_udpLink) {
        qWarning() << "CustomCommandSender: UDP链接未初始化";
        return false;
    }
    
    qDebug() << "CustomCommandSender: 发送MAVLink UDP命令:" << command;
    
    // 构造MAVLink消息
    mavlink_message_t message;
    mavlink_command_long_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.target_system = _systemId;           // 系统ID，可以根据需要调整
    cmd.target_component = componentId;
    cmd.command = command;
    cmd.confirmation = 0;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;

    // 创建MAVLink消息
    mavlink_msg_command_long_encode(
        MAVLinkProtocol::instance()->getSystemId(),
        MAVLinkProtocol::getComponentId(),
        &message,
        &cmd);

    // 将MAVLink消息转换为字节数组
    QByteArray buffer(MAVLINK_MAX_PACKET_LEN, 0);
    int len = mavlink_msg_to_send_buffer((uint8_t*)buffer.data(), &message);
    buffer.resize(len);

    // 发送数据
    qint64 bytesSent = _udpLink->sendCommand(buffer);
    bool success = (bytesSent > 0);
    
    if (showError || success) {
        qDebug() << "CustomCommandSender: 发送MAVLink命令" << command 
                << "到组件" << componentId 
                << "结果:" << (success ? "成功" : "失败")
                << "发送字节数:" << bytesSent;
    }
    
    emit commandSent(success);
    return success;
} 
