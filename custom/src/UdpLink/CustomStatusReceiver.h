/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QObject>
#include <QByteArray>
#include "UdpCommandLink.h"
#include "QGCMAVLink.h"

// 前向声明
class QQmlEngine;
class QJSEngine;

/**
 * @brief 自定义状态接收器，用于接收和解析UDP封装的MAVLink位置消息
 * 提供给QML使用的位置数据
 * 采用单例模式，确保全局只有一个实例
 */
class CustomStatusReceiver : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON
    
    // 连接状态
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    
    // GPS位置信息
    Q_PROPERTY(double latitude READ latitude NOTIFY positionChanged)
    Q_PROPERTY(double longitude READ longitude NOTIFY positionChanged)
    Q_PROPERTY(float altitude READ altitude NOTIFY positionChanged)
    
public:
    explicit CustomStatusReceiver(QObject *parent = nullptr);
    ~CustomStatusReceiver();
    
    // QML单例创建函数
    static CustomStatusReceiver* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);
    
    // 获取单例实例
    static CustomStatusReceiver* instance();
    
    // 连接状态
    bool connected() const { return _connected; }
    
    // GPS位置信息
    double latitude() const { return _latitude; }
    double longitude() const { return _longitude; }
    float altitude() const { return _altitude; }
    
    /**
     * @brief 启动状态接收
     */
    Q_INVOKABLE void startReceiving();
    
    /**
     * @brief 停止状态接收
     */
    Q_INVOKABLE void stopReceiving();
    
signals:
    // 连接状态信号
    void connectedChanged();
    
    // 位置信息信号
    void positionChanged();
    
private slots:
    /**
     * @brief 处理接收到的UDP数据
     */
    void _handleReceivedData(const QByteArray &data);
    
private:
    /**
     * @brief 解析MAVLink消息
     */
    void _parseMavlinkMessage(const mavlink_message_t &message);
    
    /**
     * @brief 处理全球位置
     */
    void _handleGlobalPositionInt(const mavlink_message_t &message);
    
    // 单例相关
    static CustomStatusReceiver* _instance;
    
    // UDP连接
    UdpCommandLink* _udpLink;
    
    // 连接状态
    bool _connected;
    qint64 _lastMessageTime;
    
    // GPS位置信息
    double _latitude;
    double _longitude;
    float _altitude;
    
    // MAVLink解析
    mavlink_status_t _mavlinkStatus;
    mavlink_message_t _mavlinkMessage;
}; 