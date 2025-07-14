#include "MissionDatabase.h"
#include <QtCore/QDebug>
#include <QtCore/QFileInfo>

// 表创建SQL定义
const QString MissionDatabase::_createRoutesTableSQL = 
    "CREATE TABLE IF NOT EXISTS routes ("
    "uuid TEXT PRIMARY KEY, "
    "modify_time INTEGER NOT NULL, "
    "name TEXT NOT NULL, "
    "waypoint_count INTEGER NOT NULL, "
    "route_length REAL NOT NULL, "
    "estimated_duration INTEGER NOT NULL, "
    "waypoints TEXT NOT NULL"
    ")";

const QString MissionDatabase::_createMissionsTableSQL = 
    "CREATE TABLE IF NOT EXISTS missions ("
    "uuid TEXT PRIMARY KEY, "
    "route_uuid TEXT NOT NULL, "
    "start_time INTEGER NOT NULL, "
    "end_time INTEGER DEFAULT 0, "
    "log_file_name TEXT, "
    "route_backup TEXT NOT NULL, "
    "result_uuid TEXT, "
    "waypoints TEXT NOT NULL, "
    "FOREIGN KEY (route_uuid) REFERENCES routes(uuid), "
    "FOREIGN KEY (result_uuid) REFERENCES results(uuid)"
    ")";

const QString MissionDatabase::_createResultsTableSQL = 
    "CREATE TABLE IF NOT EXISTS results ("
    "uuid TEXT PRIMARY KEY, "
    "mission_uuid TEXT NOT NULL, "
    "result_data TEXT NOT NULL, "
    "FOREIGN KEY (mission_uuid) REFERENCES missions(uuid)"
    ")";

MissionDatabase::MissionDatabase(QObject *parent)
    : QObject(parent)
    , _isConnected(false)
    , _connectionName(QUuid::createUuid().toString())
{
    qDebug() << "MissionDatabase 初始化";
}

MissionDatabase::~MissionDatabase()
{
    _disconnectDatabase();
}

bool MissionDatabase::initDatabase()
{
    if (!_connectDatabase()) {
        emit databaseError("无法连接到数据库");
        return false;
    }
    
    if (!_createTables()) {
        emit databaseError("无法创建数据表");
        return false;
    }
    
    // 执行数据库架构迁移
    if (!migrateDatabaseSchema()) {
        emit databaseError("数据库架构迁移失败");
        return false;
    }
    
    qDebug() << "任务数据库初始化成功:" << _getDatabasePath();
    emit databaseConnected();
    return true;
}

bool MissionDatabase::_connectDatabase()
{
    _database = QSqlDatabase::addDatabase("QSQLITE", _connectionName);
    _database.setDatabaseName(_getDatabasePath());
    
    if (!_database.open()) {
        qWarning() << "无法打开数据库:" << _database.lastError().text();
        return false;
    }
    
    // 启用外键约束
    QSqlQuery query(_database);
    if (!query.exec("PRAGMA foreign_keys=ON")) {
        qWarning() << "启用外键约束失败:" << query.lastError().text();
    } else {
        qDebug() << "外键约束已启用";
    }
    
    _isConnected = true;
    return true;
}

void MissionDatabase::_disconnectDatabase()
{
    if (_isConnected) {
        _database.close();
        QSqlDatabase::removeDatabase(_connectionName);
        _isConnected = false;
        emit databaseDisconnected();
    }
}

bool MissionDatabase::_createTables()
{
    QSqlQuery query(_database);
    
    // 创建航线表
    if (!query.exec(_createRoutesTableSQL)) {
        qWarning() << "创建航线表失败:" << query.lastError().text();
        return false;
    }
    
    // 创建任务表
    if (!query.exec(_createMissionsTableSQL)) {
        qWarning() << "创建任务表失败:" << query.lastError().text();
        return false;
    }
    
    // 创建成果表
    if (!query.exec(_createResultsTableSQL)) {
        qWarning() << "创建成果表失败:" << query.lastError().text();
        return false;
    }
    
    // 创建索引提高查询性能
    query.exec("CREATE INDEX IF NOT EXISTS idx_missions_route_uuid ON missions(route_uuid)");
    query.exec("CREATE INDEX IF NOT EXISTS idx_results_mission_uuid ON results(mission_uuid)");
    query.exec("CREATE INDEX IF NOT EXISTS idx_missions_start_time ON missions(start_time)");
    
    return true;
}

QString MissionDatabase::_getDatabasePath()
{
    QString cacheDir;
#if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
    cacheDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
#else
    cacheDir = QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation);
#endif
    
    cacheDir += "/QGCCustomMission";
    
    // 确保目录存在
    QDir dir;
    if (!dir.mkpath(cacheDir)) {
        qWarning() << "无法创建数据库目录:" << cacheDir;
        return QString();
    }
    
    return cacheDir + "/mission_database.db";
}

// ==================== 航线表操作 ====================

bool MissionDatabase::addRoute(const QString &uuid, const QString &name, int waypointCount, 
                              double routeLength, int estimatedDuration, const QString &waypoints)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("INSERT INTO routes (uuid, modify_time, name, waypoint_count, route_length, estimated_duration, waypoints) "
                  "VALUES (?, ?, ?, ?, ?, ?, ?)");
    query.addBindValue(uuid);
    query.addBindValue(getCurrentTimestamp());
    query.addBindValue(name);
    query.addBindValue(waypointCount);
    query.addBindValue(routeLength);
    query.addBindValue(estimatedDuration);
    query.addBindValue(waypoints);
    
    if (!query.exec()) {
        qWarning() << "添加航线失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功添加航线:" << name << "UUID:" << uuid;
    return true;
}

bool MissionDatabase::updateRoute(const QString &uuid, const QString &name, int waypointCount, 
                                 double routeLength, int estimatedDuration, const QString &waypoints)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("UPDATE routes SET modify_time=?, name=?, waypoint_count=?, route_length=?, estimated_duration=?, waypoints=? "
                  "WHERE uuid=?");
    query.addBindValue(getCurrentTimestamp());
    query.addBindValue(name);
    query.addBindValue(waypointCount);
    query.addBindValue(routeLength);
    query.addBindValue(estimatedDuration);
    query.addBindValue(waypoints);
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "更新航线失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功更新航线:" << name << "UUID:" << uuid;
    return true;
}

bool MissionDatabase::deleteRoute(const QString &uuid)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("DELETE FROM routes WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "删除航线失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功删除航线 UUID:" << uuid;
    return true;
}

QJsonObject MissionDatabase::getRoute(const QString &uuid)
{
    QJsonObject result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    query.prepare("SELECT * FROM routes WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec() || !query.next()) {
        qWarning() << "查询航线失败:" << query.lastError().text();
        return result;
    }
    
    result["uuid"] = query.value("uuid").toString();
    result["modify_time"] = query.value("modify_time").toLongLong();
    result["name"] = query.value("name").toString();
    result["waypoint_count"] = query.value("waypoint_count").toInt();
    result["route_length"] = query.value("route_length").toDouble();
    result["estimated_duration"] = query.value("estimated_duration").toInt();
    result["waypoints"] = query.value("waypoints").toString();
    
    return result;
}

QJsonArray MissionDatabase::getAllRoutes()
{
    QJsonArray result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    if (!query.exec("SELECT * FROM routes ORDER BY modify_time DESC")) {
        qWarning() << "查询所有航线失败:" << query.lastError().text();
        return result;
    }
    
    while (query.next()) {
        QJsonObject route;
        route["uuid"] = query.value("uuid").toString();
        route["modify_time"] = query.value("modify_time").toLongLong();
        route["name"] = query.value("name").toString();
        route["waypoint_count"] = query.value("waypoint_count").toInt();
        route["route_length"] = query.value("route_length").toDouble();
        route["estimated_duration"] = query.value("estimated_duration").toInt();
        route["waypoints"] = query.value("waypoints").toString();
        result.append(route);
    }
    
    return result;
}

// ==================== 任务表操作 ====================

bool MissionDatabase::addMission(const QString &uuid, const QString &routeUuid, 
                                 const QString &logFileName, const QString &routeBackup, const QString &waypoints)
{
    return addMissionWithTime(uuid, routeUuid, getCurrentTimestamp(), logFileName, routeBackup, waypoints);
}

bool MissionDatabase::addMissionWithTime(const QString &uuid, const QString &routeUuid, 
                                         qint64 startTime, const QString &logFileName, const QString &routeBackup, const QString &waypoints)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("INSERT INTO missions (uuid, route_uuid, start_time, log_file_name, route_backup, waypoints) "
                  "VALUES (?, ?, ?, ?, ?, ?)");
    query.addBindValue(uuid);
    query.addBindValue(routeUuid);
    query.addBindValue(startTime);
    query.addBindValue(logFileName);
    query.addBindValue(routeBackup);
    query.addBindValue(waypoints);
    
    if (!query.exec()) {
        qWarning() << "添加任务失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功添加任务 UUID:" << uuid << "关联航线:" << routeUuid << "开始时间:" << startTime;
    return true;
}

bool MissionDatabase::updateMissionEndTime(const QString &uuid, qint64 endTime)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("UPDATE missions SET end_time=? WHERE uuid=?");
    query.addBindValue(endTime);
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "更新任务完成时间失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功更新任务完成时间 UUID:" << uuid;
    return true;
}

bool MissionDatabase::updateMissionResult(const QString &uuid, const QString &resultUuid)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("UPDATE missions SET result_uuid=? WHERE uuid=?");
    query.addBindValue(resultUuid);
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "更新任务成果关联失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功更新任务成果关联 UUID:" << uuid << "成果UUID:" << resultUuid;
    return true;
}

bool MissionDatabase::deleteMission(const QString &uuid)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("DELETE FROM missions WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "删除任务失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功删除任务 UUID:" << uuid;
    return true;
}

QJsonObject MissionDatabase::getMission(const QString &uuid)
{
    QJsonObject result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    query.prepare("SELECT * FROM missions WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec() || !query.next()) {
        qWarning() << "查询任务失败:" << query.lastError().text();
        return result;
    }
    
    result["uuid"] = query.value("uuid").toString();
    result["route_uuid"] = query.value("route_uuid").toString();
    result["start_time"] = query.value("start_time").toLongLong();
    result["end_time"] = query.value("end_time").toLongLong();
    result["log_file_name"] = query.value("log_file_name").toString();
    result["route_backup"] = query.value("route_backup").toString();
    result["result_uuid"] = query.value("result_uuid").toString();
    result["waypoints"] = query.value("waypoints").toString();
    
    return result;
}

QJsonArray MissionDatabase::getAllMissions()
{
    QJsonArray result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    if (!query.exec("SELECT * FROM missions ORDER BY start_time DESC")) {
        qWarning() << "查询所有任务失败:" << query.lastError().text();
        return result;
    }
    
    while (query.next()) {
        QJsonObject mission;
        mission["uuid"] = query.value("uuid").toString();
        mission["route_uuid"] = query.value("route_uuid").toString();
        mission["start_time"] = query.value("start_time").toLongLong();
        mission["end_time"] = query.value("end_time").toLongLong();
        mission["log_file_name"] = query.value("log_file_name").toString();
        mission["route_backup"] = query.value("route_backup").toString();
        mission["result_uuid"] = query.value("result_uuid").toString();
        mission["waypoints"] = query.value("waypoints").toString();
        result.append(mission);
    }
    
    return result;
}

QJsonArray MissionDatabase::getMissionsByRoute(const QString &routeUuid)
{
    QJsonArray result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    query.prepare("SELECT * FROM missions WHERE route_uuid=? ORDER BY start_time DESC");
    query.addBindValue(routeUuid);
    
    if (!query.exec()) {
        qWarning() << "查询航线任务失败:" << query.lastError().text();
        return result;
    }
    
    while (query.next()) {
        QJsonObject mission;
        mission["uuid"] = query.value("uuid").toString();
        mission["route_uuid"] = query.value("route_uuid").toString();
        mission["start_time"] = query.value("start_time").toLongLong();
        mission["end_time"] = query.value("end_time").toLongLong();
        mission["log_file_name"] = query.value("log_file_name").toString();
        mission["route_backup"] = query.value("route_backup").toString();
        mission["result_uuid"] = query.value("result_uuid").toString();
        mission["waypoints"] = query.value("waypoints").toString();
        result.append(mission);
    }
    
    return result;
}

// ==================== 成果表操作 ====================

bool MissionDatabase::addResult(const QString &uuid, const QString &missionUuid, 
                               const QString &resultData)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("INSERT INTO results (uuid, mission_uuid, result_data) VALUES (?, ?, ?)");
    query.addBindValue(uuid);
    query.addBindValue(missionUuid);
    query.addBindValue(resultData);
    
    if (!query.exec()) {
        qWarning() << "添加成果失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功添加成果 UUID:" << uuid << "关联任务:" << missionUuid;
    return true;
}

bool MissionDatabase::updateResult(const QString &uuid, const QString &resultData)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("UPDATE results SET result_data=? WHERE uuid=?");
    query.addBindValue(resultData);
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "更新成果失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功更新成果 UUID:" << uuid;
    return true;
}

bool MissionDatabase::deleteResult(const QString &uuid)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("DELETE FROM results WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec()) {
        qWarning() << "删除成果失败:" << query.lastError().text();
        return false;
    }
    
    qDebug() << "成功删除成果 UUID:" << uuid;
    return true;
}

QJsonObject MissionDatabase::getResult(const QString &uuid)
{
    QJsonObject result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    query.prepare("SELECT * FROM results WHERE uuid=?");
    query.addBindValue(uuid);
    
    if (!query.exec() || !query.next()) {
        qWarning() << "查询成果失败:" << query.lastError().text();
        return result;
    }
    
    result["uuid"] = query.value("uuid").toString();
    result["mission_uuid"] = query.value("mission_uuid").toString();
    result["result_data"] = query.value("result_data").toString();
    
    return result;
}

QJsonArray MissionDatabase::getAllResults()
{
    QJsonArray result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    if (!query.exec("SELECT * FROM results")) {
        qWarning() << "查询所有成果失败:" << query.lastError().text();
        return result;
    }
    
    while (query.next()) {
        QJsonObject resultObj;
        resultObj["uuid"] = query.value("uuid").toString();
        resultObj["mission_uuid"] = query.value("mission_uuid").toString();
        resultObj["result_data"] = query.value("result_data").toString();
        result.append(resultObj);
    }
    
    return result;
}

QJsonArray MissionDatabase::getResultsByMission(const QString &missionUuid)
{
    QJsonArray result;
    if (!_isConnected) return result;
    
    QSqlQuery query(_database);
    query.prepare("SELECT * FROM results WHERE mission_uuid=?");
    query.addBindValue(missionUuid);
    
    if (!query.exec()) {
        qWarning() << "查询任务成果失败:" << query.lastError().text();
        return result;
    }
    
    while (query.next()) {
        QJsonObject resultObj;
        resultObj["uuid"] = query.value("uuid").toString();
        resultObj["mission_uuid"] = query.value("mission_uuid").toString();
        resultObj["result_data"] = query.value("result_data").toString();
        result.append(resultObj);
    }
    
    return result;
}




// 清空所有数据表
bool MissionDatabase::clearAllData()
{
    if (!_isConnected) {
        qWarning() << "数据库未连接，无法清空数据";
        return false;
    }
    
    qDebug() << "开始清空所有数据表...";
    
    QSqlQuery query(_database);
    
    // 由于外键约束，需要按顺序删除
    // 1. 先删除成果表（results）
    if (!query.exec("DELETE FROM results")) {
        qWarning() << "清空成果表失败:" << query.lastError().text();
        return false;
    }
    qDebug() << "成果表已清空";
    
    // 2. 再删除任务表（missions）
    if (!query.exec("DELETE FROM missions")) {
        qWarning() << "清空任务表失败:" << query.lastError().text();
        return false;
    }
    qDebug() << "任务表已清空";
    
    // 3. 最后删除航线表（routes）
    if (!query.exec("DELETE FROM routes")) {
        qWarning() << "清空航线表失败:" << query.lastError().text();
        return false;
    }
    qDebug() << "航线表已清空";
    
    // 重置自增序列（如果有的话）
    query.exec("DELETE FROM sqlite_sequence WHERE name IN ('routes', 'missions', 'results')");
    
    // 执行 VACUUM 来优化数据库文件大小
    if (!query.exec("VACUUM")) {
        qWarning() << "数据库优化失败:" << query.lastError().text();
        // 这不是致命错误，继续执行
    }
    
    qDebug() << "数据库清空完成，所有数据已删除";
    return true;
}

// 检查表结构
void MissionDatabase::checkTableStructure()
{
    if (!_isConnected) {
        qWarning() << "数据库未连接，无法检查表结构";
        return;
    }
    
    QSqlQuery query(_database);
    
    // 检查 missions 表结构
    if (query.exec("PRAGMA table_info(missions)")) {
        qDebug() << "=== missions 表结构 ===";
        while (query.next()) {
            qDebug() << "列:" << query.value("name").toString() 
                     << "类型:" << query.value("type").toString()
                     << "非空:" << query.value("notnull").toBool()
                     << "默认值:" << query.value("dflt_value").toString()
                     << "主键:" << query.value("pk").toBool();
        }
    }
    
    // 检查 routes 表结构
    if (query.exec("PRAGMA table_info(routes)")) {
        qDebug() << "=== routes 表结构 ===";
        while (query.next()) {
            qDebug() << "列:" << query.value("name").toString() 
                     << "类型:" << query.value("type").toString()
                     << "非空:" << query.value("notnull").toBool()
                     << "默认值:" << query.value("dflt_value").toString()
                     << "主键:" << query.value("pk").toBool();
        }
    }
    
    // 检查 results 表结构
    if (query.exec("PRAGMA table_info(results)")) {
        qDebug() << "=== results 表结构 ===";
        while (query.next()) {
            qDebug() << "列:" << query.value("name").toString() 
                     << "类型:" << query.value("type").toString()
                     << "非空:" << query.value("notnull").toBool()
                     << "默认值:" << query.value("dflt_value").toString()
                     << "主键:" << query.value("pk").toBool();
                 }
     }
}

// 迁移数据库架构
bool MissionDatabase::migrateDatabaseSchema()
{
    if (!_isConnected) {
        qWarning() << "数据库未连接，无法迁移架构";
        return false;
    }
    
    qDebug() << "开始检查数据库架构...";
    
    // 这里可以添加将来需要的数据库架构升级逻辑
    // 目前数据库表结构已经完整，暂时不需要迁移操作
    
    qDebug() << "数据库架构检查完成";
    return true;
}

// ==================== 航点数据处理工具函数 ====================

QString MissionDatabase::createWaypointsJson(const QJsonArray &waypoints)
{
    QJsonDocument doc(waypoints);
    return doc.toJson(QJsonDocument::Compact);
}

QJsonArray MissionDatabase::parseWaypointsJson(const QString &waypointsJson)
{
    if (waypointsJson.isEmpty()) {
        return QJsonArray();
    }
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(waypointsJson.toUtf8(), &error);
    
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "解析航点JSON失败:" << error.errorString();
        return QJsonArray();
    }
    
    return doc.array();
}

QString MissionDatabase::createWaypointJson(double longitude, double latitude, double altitude, int type)
{
    QJsonObject waypoint;
    waypoint["longitude"] = longitude;
    waypoint["latitude"] = latitude;
    waypoint["altitude"] = altitude;
    waypoint["type"] = type;
    
    QJsonDocument doc(waypoint);
    return doc.toJson(QJsonDocument::Compact);
}

// ==================== 成果数据处理工具函数 ====================

QString MissionDatabase::createResultsJson(const QJsonArray &results)
{
    QJsonDocument doc(results);
    return doc.toJson(QJsonDocument::Compact);
}

QJsonArray MissionDatabase::parseResultsJson(const QString &resultsJson)
{
    if (resultsJson.isEmpty()) {
        return QJsonArray();
    }
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(resultsJson.toUtf8(), &error);
    
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "解析成果JSON失败:" << error.errorString();
        return QJsonArray();
    }
    
    return doc.array();
}

QString MissionDatabase::createResultJson(const QString &imagePath, const QString &category)
{
    QJsonObject result;
    result["image_path"] = imagePath;
    result["category"] = category;
    
    QJsonDocument doc(result);
    return doc.toJson(QJsonDocument::Compact);
}

