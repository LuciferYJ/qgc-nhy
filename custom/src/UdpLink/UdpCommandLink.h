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
#include <QUdpSocket>
#include <QHostAddress>
#include <QByteArray>
#include <QMutex>
#include <QQmlEngine>
#include <QJSEngine>

/**
 * @brief UDP通信类，用于发送和接收MAVLink命令
 * 支持双套接字模式：一个用于发送，一个用于接收
 */
class UdpCommandLink : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

public:
    /**
     * @brief QML单例创建函数
     */
    static UdpCommandLink* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);

    /**
     * @brief 创建或获取UdpCommandLink的单例实例
     * @param sendRemoteHost 发送目标主机地址
     * @param sendRemotePort 发送目标端口
     * @param receiveLocalPort 接收监听端口
     * @return UdpCommandLink实例
     */
    static UdpCommandLink* instance(const QString& sendRemoteHost = QString(), 
                                   quint16 sendRemotePort = 0, 
                                   quint16 receiveLocalPort = 0);

    /**
     * @brief 构造函数
     * @param sendRemoteHost 发送目标主机地址
     * @param sendRemotePort 发送目标端口
     * @param receiveLocalPort 接收监听端口
     * @param parent 父对象
     */
    explicit UdpCommandLink(const QString& sendRemoteHost = QString(), 
                           quint16 sendRemotePort = 0, 
                           quint16 receiveLocalPort = 0, 
                           QObject *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~UdpCommandLink();

    /**
     * @brief 通过UDP发送命令
     * @param data 要发送的数据
     * @return 成功发送的字节数，-1表示发送失败
     */
    qint64 sendCommand(const QByteArray &data);

    /**
     * @brief 设置发送目标地址
     * @param address 发送目标地址
     */
    void setSendRemoteAddress(const QString &address);

    /**
     * @brief 设置发送目标端口
     * @param port 发送目标端口
     */
    void setSendRemotePort(quint16 port);

    /**
     * @brief 设置接收监听端口
     * @param port 接收监听端口
     */
    void setReceiveLocalPort(quint16 port);

    /**
     * @brief 获取当前接收监听端口
     * @return 接收监听端口
     */
    quint16 getReceiveLocalPort() const { return _receiveLocalPort; }

    /**
     * @brief 获取当前发送目标端口
     * @return 发送目标端口
     */
    quint16 getSendRemotePort() const { return _sendRemotePort; }

    /**
     * @brief 初始化UDP连接（发送和接收套接字）
     * @return 是否成功初始化
     */
    bool initialize();

    /**
     * @brief 检查是否已初始化
     * @return 是否已初始化
     */
    bool isInitialized() const { return _isInitialized; }

    /**
     * @brief 静态方法：设置发送目标地址（供外部直接调用）
     * @param address 发送目标地址
     */
    Q_INVOKABLE static void setGlobalSendRemoteAddress(const QString &address);

    /**
     * @brief 静态方法：设置发送目标端口（供外部直接调用）
     * @param port 发送目标端口
     */
    Q_INVOKABLE static void setGlobalSendRemotePort(quint16 port);

    /**
     * @brief 静态方法：设置接收监听端口（供外部直接调用）
     * @param port 接收监听端口
     */
    Q_INVOKABLE static void setGlobalReceiveLocalPort(quint16 port);

    /**
     * @brief 静态方法：获取当前发送目标地址
     * @return 发送目标地址
     */
    Q_INVOKABLE static QString getGlobalSendRemoteAddress();

    /**
     * @brief 静态方法：获取当前发送目标端口
     * @return 发送目标端口
     */
    Q_INVOKABLE static quint16 getGlobalSendRemotePort();

    /**
     * @brief 静态方法：获取当前接收监听端口
     * @return 接收监听端口
     */
    Q_INVOKABLE static quint16 getGlobalReceiveLocalPort();

signals:
    /**
     * @brief 接收到数据时发出的信号
     * @param data 接收到的数据
     */
    void dataReceived(const QByteArray &data);

private slots:
    /**
     * @brief 处理接收套接字收到的UDP数据包
     */
    void _readPendingDatagrams();

private:
    // 禁止拷贝构造
    UdpCommandLink(const UdpCommandLink&) = delete;
    UdpCommandLink& operator=(const UdpCommandLink&) = delete;
    
    static UdpCommandLink* _instance; // 单例实例
    static QMutex _mutex;             // 互斥锁，保护单例创建
    
    QUdpSocket* _sendSocket;          // 发送套接字
    QUdpSocket* _receiveSocket;       // 接收套接字
    QHostAddress _sendRemoteAddress;  // 发送目标地址
    quint16 _sendRemotePort;          // 发送目标端口
    quint16 _receiveLocalPort;        // 接收监听端口
    bool _isInitialized;              // 是否已初始化
}; 