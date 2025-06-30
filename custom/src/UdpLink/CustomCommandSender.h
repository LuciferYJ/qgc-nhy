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
#include "MAVLinkProtocol.h"

// 前向声明
class QQmlEngine;
class QJSEngine;

/**
 * @brief 自定义命令发送器，用于在QML中直接发送命令而无需依赖Vehicle对象
 * 此类可以在无人机未连接时仍能发送UDP命令
 * 采用单例模式，确保全局只有一个实例
 * 功能纯粹，只负责发送MAVLink命令，网络配置由UdpCommandLink管理
 */
class CustomCommandSender : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON
    
public:
    explicit CustomCommandSender(QObject *parent = nullptr);
    ~CustomCommandSender();
    
    // QML单例创建函数
    static CustomCommandSender* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);
    
    // 获取单例实例
    static CustomCommandSender* instance();
    
    /**
     * @brief 发送MAVLink命令
     * @param componentId 目标组件ID
     * @param command 命令ID
     * @param showError 是否显示错误
     * @param param1-param7 命令参数
     * @return 是否成功发送
     */
    Q_INVOKABLE bool sendCommandByUdp(int componentId, int command, bool showError, 
                                     float param1 = 0, float param2 = 0, float param3 = 0,
                                     float param4 = 0, float param5 = 0, float param6 = 0, float param7 = 0);
    
signals:
    void commandSent(bool success);
    void dataReceived(const QByteArray &data);
    
private:
    UdpCommandLink* _udpLink;
    int _systemId = 255;  // 默认系统ID，可以根据需要修改
    
    // 单例相关
    static CustomCommandSender* _instance;
}; 