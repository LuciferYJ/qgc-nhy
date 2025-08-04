#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <QJsonObject>
#include <QJsonDocument>
#include "CustomUdpTypes.h"

class QQmlEngine;
class QJSEngine;

/**
 * @brief JSON UDP通信类
 * 整合UDP收发、JSON解析和连接状态管理
 */
class SimpleMavlinkUdp : public QObject
{
    Q_OBJECT
    
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    
    // 目标IP设置属性
    Q_PROPERTY(QString targetIP READ getTargetIP WRITE setTargetIP NOTIFY targetIPChanged)
    
    // 任务状态数据属性（为QML兼容性保持单独访问器）
    Q_PROPERTY(int missionState READ missionState NOTIFY missionStatusChanged)
    Q_PROPERTY(int flightTime READ flightTime NOTIFY missionStatusChanged)
    Q_PROPERTY(int remainingDistance READ remainingDistance NOTIFY missionStatusChanged)
    Q_PROPERTY(int capturedImages READ capturedImages NOTIFY missionStatusChanged)
    Q_PROPERTY(int flownDistance READ flownDistance NOTIFY missionStatusChanged)
    Q_PROPERTY(int totalDistance READ totalDistance NOTIFY missionStatusChanged)
    
    // 定位状态数据属性（为QML兼容性保持单独访问器）
    Q_PROPERTY(int locationStatus READ locationStatus NOTIFY locationStatusChanged)
    Q_PROPERTY(int visualSubStatus READ visualSubStatus NOTIFY locationStatusChanged)
    Q_PROPERTY(int satelliteCount READ satelliteCount NOTIFY locationStatusChanged)
    Q_PROPERTY(float locationAccuracy READ locationAccuracy NOTIFY locationStatusChanged)
    
public:
    explicit SimpleMavlinkUdp(QObject *parent = nullptr);
    ~SimpleMavlinkUdp();
    
    // QML单例创建函数
    static SimpleMavlinkUdp* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);
    static SimpleMavlinkUdp* instance();
    
    // 连接状态
    bool connected() const { return _connected; }
    
    // 任务状态数据访问器（QML兼容）
    int missionState() const { return _missionStatus.missionState; }
    int flightTime() const { return _missionStatus.flightTime; }
    int remainingDistance() const { return _missionStatus.remainingDistance; }
    int capturedImages() const { return _missionStatus.capturedImages; }
    int flownDistance() const { return _missionStatus.flownDistance; }
    int totalDistance() const { return _missionStatus.totalDistance; }
    
    // 定位状态数据访问器（QML兼容）
    int locationStatus() const { return _locationStatus.locationStatus; }
    int visualSubStatus() const { return _locationStatus.visualSubStatus; }
    int satelliteCount() const { return _locationStatus.satelliteCount; }
    float locationAccuracy() const { return _locationStatus.accuracy; }
    
    // 结构体访问器（C++使用）
    const MissionStatus& missionStatus() const { return _missionStatus; }
    const LocationStatus& locationStatusStruct() const { return _locationStatus; }
    
    // 启动/停止UDP通信
    Q_INVOKABLE bool start(quint16 listenPort = 7777);
    Q_INVOKABLE void stop();
    
    // 设置目标IP地址
    Q_INVOKABLE void setTargetIP(const QString& ip);
    Q_INVOKABLE QString getTargetIP() const;
    
    // JSON收发接口
    Q_INVOKABLE bool sendJson(int messageType, const QJsonObject& data);
    Q_INVOKABLE bool sendHeartbeat();
    Q_INVOKABLE bool sendTakeoffCommand(const QString& missionUuid, 
                                       const QString& action, 
                                       float altitude = 50.0);
    Q_INVOKABLE bool sendMissionStatus(const QJsonObject& data);
    Q_INVOKABLE bool sendLocationStatus(const QJsonObject& data);
    
    // 结构体发送接口（C++使用）
    bool sendHeartbeat(const HeartbeatData& data);
    bool sendTakeoffCommand(const TakeoffCommand& cmd);
    bool sendMissionStatus(const MissionStatus& status);
    bool sendLocationStatus(const LocationStatus& status);
    bool sendAckResponse(const AckResponse& ack);

signals:
    void connectedChanged();
    void missionStatusChanged();
    void locationStatusChanged();
    void targetIPChanged(); // Added for targetIP property
    
    // JSON消息信号
    void jsonReceived(int messageType, const QJsonObject& data);
    void commandAckReceived(bool success, const QString& message);
    
    // 结构体消息信号（C++使用）
    void heartbeatReceived(const HeartbeatData& data);
    void takeoffCommandReceived(const TakeoffCommand& cmd);
    void missionStatusReceived(const MissionStatus& status);
    void locationStatusReceived(const LocationStatus& status);
    void ackResponseReceived(const AckResponse& ack);
    
private slots:
    void _readPendingDatagrams();
    void _checkConnectionTimeout();
    
private:
    void _parseJsonMessage(const QJsonObject& message);
    QString _createJsonMessage(MessageType type, const QJsonObject& data);
    bool _sendJsonMessage(MessageType type, const QJsonObject& data);
    
    // 网络
    QUdpSocket* _socket;
    quint16 _listenPort;
    QString _targetIP;
    
    // 连接状态
    bool _connected;
    qint64 _lastHeartbeatTime;
    QTimer* _timeoutTimer;
    
    // 数据状态（使用结构体）
    MissionStatus _missionStatus;
    LocationStatus _locationStatus;
    
    // 单例
    static SimpleMavlinkUdp* _instance;
    
    // 常量
    static const qint64 CONNECTION_TIMEOUT_MS = 3000; // 3秒超时
}; 