#pragma once

#include <QtCore/QObject>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QtCore/QDateTime>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonArray>
#include <QtCore/QStandardPaths>
#include <QtCore/QDir>
#include <QtCore/QUuid>
#include <QDebug>

// 成果类型枚举
enum ResultType {
    RESULT_TYPE_AIRCRAFT = 0,    // 飞机
    RESULT_TYPE_VEHICLE = 1,     // 车
    RESULT_TYPE_BUILDING = 2     // 建筑物
};

// 成果数据结构
struct ResultData {
    int type;              // 成果类型 (0:飞机, 1:车, 2:建筑物)
    QString filePath;      // 文件路径
    double longitude;      // 经度
    double latitude;       // 纬度
    
    ResultData() : type(0), longitude(0.0), latitude(0.0) {}
    ResultData(int t, const QString &path, double lon, double lat) 
        : type(t), filePath(path), longitude(lon), latitude(lat) {}
};

class MissionDatabase : public QObject
{
    Q_OBJECT

public:
    explicit MissionDatabase(QObject *parent = nullptr);
    ~MissionDatabase();

    // 数据库初始化
    Q_INVOKABLE bool initDatabase();
    Q_INVOKABLE bool isConnected() const { return _isConnected; }

    // 航线表操作 (routes)
    struct RouteInfo {
        QString uuid;
        qint64 modifyTime;      // 修改时间戳
        QString name;           // 航线名称
        int waypointCount;      // 航点数
        double routeLength;     // 航线长度(米)
        int estimatedDuration;  // 预计时长(秒)
        QString waypoints;      // 航点数据(JSON格式)
    };

    Q_INVOKABLE bool addRoute(const QString &uuid, const QString &name, int waypointCount, 
                             double routeLength, int estimatedDuration, const QString &waypoints);
    Q_INVOKABLE bool updateRoute(const QString &uuid, const QString &name, int waypointCount, 
                               double routeLength, int estimatedDuration, const QString &waypoints);
    Q_INVOKABLE bool deleteRoute(const QString &uuid);
    Q_INVOKABLE QJsonObject getRoute(const QString &uuid);
    Q_INVOKABLE QJsonArray getAllRoutes();

    // 任务表操作 (missions)
    struct MissionInfo {
        QString uuid;
        QString routeUuid;      // 关联的航线UUID
        qint64 startTime;       // 开始时间戳
        qint64 endTime;         // 完成时间戳
        QString logFileName;    // 日志文件名
        QString routeBackup;    // 航线备份(JSON格式)
        QString resultUuid;     // 关联的成果UUID
        QString waypoints;      // 航点数据(JSON格式)
    };

    Q_INVOKABLE bool addMission(const QString &uuid, const QString &routeUuid, 
                               const QString &logFileName, const QString &waypoints);
    Q_INVOKABLE bool addMissionWithTime(const QString &uuid, const QString &routeUuid, 
                                       qint64 startTime, const QString &logFileName, const QString &waypoints);
    Q_INVOKABLE bool updateMissionResult(const QString &uuid, const QString &resultUuid);
    Q_INVOKABLE bool updateMissionEndTime(const QString &uuid, qint64 endTime);
    Q_INVOKABLE bool deleteMission(const QString &uuid);
    Q_INVOKABLE QJsonObject getMission(const QString &uuid);
    Q_INVOKABLE QJsonArray getAllMissions();
    Q_INVOKABLE QJsonArray getMissionsByRoute(const QString &routeUuid);

    // 成果表操作 (results)
    struct ResultInfo {
        QString uuid;
        QString missionUuid;    // 关联的任务UUID
        QString resultData;     // 成果数据(JSON格式，包含多个成果：图片路径和类别)
    };

    Q_INVOKABLE bool addResult(const QString &uuid, const QString &missionUuid, 
                              const QString &resultData);
    Q_INVOKABLE bool updateResult(const QString &uuid, const QString &resultData);
    Q_INVOKABLE bool deleteResult(const QString &uuid);
    Q_INVOKABLE QJsonObject getResult(const QString &uuid);
    Q_INVOKABLE QJsonArray getAllResults();
    Q_INVOKABLE QJsonArray getResultsByMission(const QString &missionUuid);

    // 工具函数
    Q_INVOKABLE QString generateUuid() { return QUuid::createUuid().toString(QUuid::WithoutBraces); }
    Q_INVOKABLE qint64 getCurrentTimestamp() { return QDateTime::currentSecsSinceEpoch(); }
    Q_INVOKABLE bool clearAllData();  // 清空所有数据表
    Q_INVOKABLE void checkTableStructure();  // 检查表结构
    
    // ==================== 当前航线UUID管理 ====================
    // 设置当前航线UUID
    Q_INVOKABLE void setCurrentRouteUuid(const QString &routeUuid);
    
    // 获取当前航线UUID
    Q_INVOKABLE QString getCurrentRouteUuid();
    
    // 清除当前航线UUID
    Q_INVOKABLE void clearCurrentRouteUuid(); 
    
    // 获取类型名称
    Q_INVOKABLE QString getResultTypeName(int type);
    
    // 获取所有类型名称
    Q_INVOKABLE QJsonObject getAllResultTypeNames();
    

signals:
    void databaseError(const QString &error);
    void databaseConnected();
    void databaseDisconnected();

private:
    bool _connectDatabase();
    void _disconnectDatabase();
    bool _createTables();
    QString _getDatabasePath();

    
    QSqlDatabase _database;
    bool _isConnected;
    QString _connectionName;
    
    // 当前航线UUID存储
    QString _currentRouteUuid;
    
    // 表创建SQL
    static const QString _createRoutesTableSQL;
    static const QString _createMissionsTableSQL;
    static const QString _createResultsTableSQL;
}; 