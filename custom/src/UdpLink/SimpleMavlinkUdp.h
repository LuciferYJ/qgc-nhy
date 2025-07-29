#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include "QGCMAVLink.h"

class QQmlEngine;
class QJSEngine;

/**
 * @brief 简化的MAVLink UDP通信类
 * 整合UDP收发、MAVLink解析和连接状态管理
 */
class SimpleMavlinkUdp : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON
    
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    
    // 任务状态数据属性
    Q_PROPERTY(int missionState READ missionState NOTIFY missionStateChanged)
    Q_PROPERTY(int flightTime READ flightTime NOTIFY flightTimeChanged)
    Q_PROPERTY(int remainingDistance READ remainingDistance NOTIFY remainingDistanceChanged)
    Q_PROPERTY(int capturedImages READ capturedImages NOTIFY capturedImagesChanged)
    
    // 定位状态数据属性
    Q_PROPERTY(int locationStatus READ locationStatus NOTIFY locationStatusChanged)
    Q_PROPERTY(int visualSubStatus READ visualSubStatus NOTIFY visualSubStatusChanged)
    Q_PROPERTY(int satelliteCount READ satelliteCount NOTIFY satelliteCountChanged)
    Q_PROPERTY(float locationAccuracy READ locationAccuracy NOTIFY locationAccuracyChanged)
    
public:
    explicit SimpleMavlinkUdp(QObject *parent = nullptr);
    ~SimpleMavlinkUdp();
    
    // QML单例创建函数
    static SimpleMavlinkUdp* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);
    static SimpleMavlinkUdp* instance();
    
    // 连接状态
    bool connected() const { return _connected; }
    
    // 任务状态数据访问器
    int missionState() const { return _missionState; }
    int flightTime() const { return _flightTime; }
    int remainingDistance() const { return _remainingDistance; }
    int capturedImages() const { return _capturedImages; }
    
    // 定位状态数据访问器
    int locationStatus() const { return _locationStatus; }
    int visualSubStatus() const { return _visualSubStatus; }
    int satelliteCount() const { return _satelliteCount; }
    float locationAccuracy() const { return _locationAccuracy; }
    
    // 启动/停止UDP通信
    Q_INVOKABLE bool start(quint16 listenPort = 7777);
    Q_INVOKABLE void stop();
    
    // 发送MAVLink命令（如果需要）
    Q_INVOKABLE bool sendCommand(int command, float param1 = 0, float param2 = 0, 
                                float param3 = 0, float param4 = 0);

signals:
    void connectedChanged();
    void missionStateChanged();
    void flightTimeChanged();
    void remainingDistanceChanged();
    void capturedImagesChanged();
    void locationStatusChanged();
    void visualSubStatusChanged();
    void satelliteCountChanged();
    void locationAccuracyChanged();
    
private slots:
    void _readPendingDatagrams();
    void _checkConnectionTimeout();
    
private:
    void _parseMavlinkMessage(const mavlink_message_t &message);
    
    // 网络
    QUdpSocket* _socket;
    quint16 _listenPort;
    
    // 连接状态
    bool _connected;
    qint64 _lastHeartbeatTime;
    QTimer* _timeoutTimer;
    
    // 任务状态数据
    int _missionState;
    int _flightTime;
    int _remainingDistance;
    int _capturedImages;
    
    // 定位状态数据
    int _locationStatus;
    int _visualSubStatus;
    int _satelliteCount;
    float _locationAccuracy;
    
    // MAVLink解析
    mavlink_status_t _mavlinkStatus;
    mavlink_message_t _mavlinkMessage;
    
    // 单例
    static SimpleMavlinkUdp* _instance;
    
    // 常量
    static const qint64 CONNECTION_TIMEOUT_MS = 3000; // 3秒超时
}; 