#include "MissionDatabase.h"
#include <QtCore/QDebug>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonDocument>
#include <QtCore/QMetaObject>
#include <QtCore/QCryptographicHash>
#include <QtPositioning/QGeoCoordinate>
#include <cmath>

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

bool MissionDatabase::addRoute(const QString &routeName, const QVariant& visualItems)
{
    if (!_isConnected) return false;
    
    // 生成新的UUID
    QString newUuid = QUuid::createUuid().toString();
    
    // visualItems是一个QObject*（QQmlListModel），需要通过方法访问
    QObject* visualItemsObj = visualItems.value<QObject*>();
    if (!visualItemsObj) {
        return false;
    }
    
    // 获取count属性
    QVariant countVariant = visualItemsObj->property("count");
    int itemCount = countVariant.toInt();
    
    QJsonArray waypointsArray;
    int waypointCount = 0;
    double totalDistance = 0.0;
    QObject* previousItem = nullptr;
    
    // 遍历所有visualItems
    for (int i = 0; i < itemCount; i++) {
        // 直接获取QObject*类型的item
        QObject* item = nullptr;
        
        // QmlObjectListModel::get(int)返回QObject*，不是QVariant
        if (!QMetaObject::invokeMethod(visualItemsObj, "get", Q_RETURN_ARG(QObject*, item), Q_ARG(int, i)) || !item) {
            continue;
        }
        
        // 检查是否指定坐标且不是第一个项目（MissionSettingsItem）
        QVariant specifiesCoordinate = item->property("specifiesCoordinate");
        QVariant sequenceNumber = item->property("sequenceNumber");
        
        if (specifiesCoordinate.toBool() && sequenceNumber.toInt() > 0) {
            
            QJsonObject waypointObj;
            
            // 获取坐标数据 - 使用QGeoCoordinate类型
            QVariant coordinateVariant = item->property("coordinate");
            QGeoCoordinate coordinate = coordinateVariant.value<QGeoCoordinate>();
            
            if (!coordinate.isValid()) {
                continue;
            }
            
            waypointObj["latitude"] = coordinate.latitude();
            waypointObj["longitude"] = coordinate.longitude();
            waypointObj["altitude"] = coordinate.altitude();
            
            // 获取基本属性
            waypointObj["sequence"] = sequenceNumber.toInt();
            waypointObj["command"] = item->property("command").toInt();
            
            // 获取高度信息
            QVariant altitudeVariant = item->property("altitude");
            QObject* altitudeObj = altitudeVariant.value<QObject*>();
            if (altitudeObj) {
                waypointObj["altitude"] = altitudeObj->property("rawValue").toDouble();
            }
            
            // 获取MAVLink参数
            QVariant missionItemVariant = item->property("missionItem");
            QObject* missionItem = missionItemVariant.value<QObject*>();
            if (missionItem) {
                // 通过QMetaObject调用方法获取参数
                QVariant param1, param2, param3, param4, param5, param6, param7;
                QMetaObject::invokeMethod(missionItem, "param1", Q_RETURN_ARG(QVariant, param1));
                QMetaObject::invokeMethod(missionItem, "param2", Q_RETURN_ARG(QVariant, param2));
                QMetaObject::invokeMethod(missionItem, "param3", Q_RETURN_ARG(QVariant, param3));
                QMetaObject::invokeMethod(missionItem, "param4", Q_RETURN_ARG(QVariant, param4));
                QMetaObject::invokeMethod(missionItem, "param5", Q_RETURN_ARG(QVariant, param5));
                QMetaObject::invokeMethod(missionItem, "param6", Q_RETURN_ARG(QVariant, param6));
                QMetaObject::invokeMethod(missionItem, "param7", Q_RETURN_ARG(QVariant, param7));
                
                waypointObj["param1"] = param1.toDouble();
                waypointObj["param2"] = param2.toDouble();
                waypointObj["param3"] = param3.toDouble();
                waypointObj["param4"] = param4.toDouble();
                waypointObj["param5"] = param5.toDouble();
                waypointObj["param6"] = param6.toDouble();
                waypointObj["param7"] = param7.toDouble();
                
                // 获取frame和autocontinue
                QVariant frame, autoContinue;
                QMetaObject::invokeMethod(missionItem, "frame", Q_RETURN_ARG(QVariant, frame));
                QMetaObject::invokeMethod(missionItem, "autoContinue", Q_RETURN_ARG(QVariant, autoContinue));
                waypointObj["frame"] = frame.toInt();
                waypointObj["autocontinue"] = autoContinue.toBool();
            } else {
                // 默认值
                waypointObj["param1"] = 0.0;
                waypointObj["param2"] = 0.0;
                waypointObj["param3"] = 0.0;
                waypointObj["param4"] = 0.0;
                waypointObj["param5"] = waypointObj["latitude"].toDouble();
                waypointObj["param6"] = waypointObj["longitude"].toDouble();
                waypointObj["param7"] = waypointObj["altitude"].toDouble();
                waypointObj["frame"] = 3;
                waypointObj["autocontinue"] = true;
            }
            
            // 检查是否为起飞项目
            QVariant isTakeoffItem = item->property("isTakeoffItem");
            if (isTakeoffItem.toBool()) {
                waypointObj["itemType"] = "TakeoffMissionItem";
                
                // 获取launch坐标
                QVariant launchCoordinate = item->property("launchCoordinate");
                QObject* launchCoord = launchCoordinate.value<QObject*>();
                if (launchCoord) {
                    waypointObj["launchLatitude"] = launchCoord->property("latitude").toDouble();
                    waypointObj["launchLongitude"] = launchCoord->property("longitude").toDouble();
                    waypointObj["launchAltitude"] = launchCoord->property("altitude").toDouble();
                }
                
                QVariant launchTakeoffAtSameLocation = item->property("launchTakeoffAtSameLocation");
                waypointObj["launchTakeoffAtSameLocation"] = launchTakeoffAtSameLocation.toBool();
            } else {
                waypointObj["itemType"] = "SimpleMissionItem";
            }
            
            // 获取高度模式
            QVariant altitudeMode = item->property("altitudeMode");
            if (altitudeMode.isValid()) {
                waypointObj["altitudeMode"] = altitudeMode.toInt();
            }
            
            waypointsArray.append(waypointObj);
            waypointCount++;
            
            // 计算距离（使用QGeoCoordinate内置的距离计算）
            if (previousItem) {
                QVariant prevCoordinateVariant = previousItem->property("coordinate");
                QGeoCoordinate prevCoordinate = prevCoordinateVariant.value<QGeoCoordinate>();
                if (prevCoordinate.isValid() && coordinate.isValid()) {
                    // 使用QGeoCoordinate的distanceTo方法计算精确的地理距离
                    double segmentDistance = prevCoordinate.distanceTo(coordinate);
                    totalDistance += segmentDistance;
                }
            }
            
            previousItem = item;
        }
    }
    
    // 估算飞行时间（假设平均速度为15 m/s）
    double defaultSpeed = 15.0;
    int estimatedDuration = totalDistance > 0 ? static_cast<int>(ceil(totalDistance / defaultSpeed)) : 0;
    
        // 生成航点JSON字符串
    QString waypointsJson = QJsonDocument(waypointsArray).toJson(QJsonDocument::Compact);
    
    // 调用现有的addRoute方法
    bool success = addRoute(newUuid, routeName, waypointCount, totalDistance, estimatedDuration, waypointsJson);
    
    if (success) {
        // 设置为当前航线
        setCurrentRouteUuid(newUuid);
    }
    
    return success;
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



QString MissionDatabase::checkRouteUnique(const QVariant& visualItems)
{
    if (!_isConnected) return QString(); // 如果数据库未连接，返回空
    
    qDebug() << "=== checkRouteUnique: 开始检查航线唯一性 ===";
    
    // visualItems是一个QObject*（QQmlListModel），需要通过方法访问
    QObject* visualItemsObj = visualItems.value<QObject*>();
    if (!visualItemsObj) {
        qDebug() << "checkRouteUnique: visualItemsObj为空";
        return QString();
    }
    
    // 获取count属性
    QVariant countVariant = visualItemsObj->property("count");
    int itemCount = countVariant.toInt();
    qDebug() << "checkRouteUnique: visualItems count =" << itemCount;
    
    // 创建规范化的航点数组用于哈希比较
    QJsonArray normalizedWaypoints;
    
    // 遍历所有visualItems，使用与addRoute相同的逻辑
    for (int i = 0; i < itemCount; i++) {
        // 直接获取QObject*类型的item
        QObject* item = nullptr;
        
        if (!QMetaObject::invokeMethod(visualItemsObj, "get", Q_RETURN_ARG(QObject*, item), Q_ARG(int, i)) || !item) {
            continue;
        }
        
        // 检查是否指定坐标且不是第一个项目（MissionSettingsItem）
        QVariant specifiesCoordinate = item->property("specifiesCoordinate");
        QVariant sequenceNumber = item->property("sequenceNumber");
        
        qDebug() << "checkRouteUnique: item" << i << "specifiesCoordinate:" << specifiesCoordinate.toBool() 
                 << "sequenceNumber:" << sequenceNumber.toInt();
        
        if (specifiesCoordinate.toBool() && sequenceNumber.toInt() > 0) {
            // 获取坐标数据 - 使用QGeoCoordinate类型
            QVariant coordinateVariant = item->property("coordinate");
            QGeoCoordinate coordinate = coordinateVariant.value<QGeoCoordinate>();
            
            if (!coordinate.isValid()) {
                qDebug() << "checkRouteUnique: item" << i << "坐标无效";
                continue;
            }
            
            QJsonObject normalizedWaypoint;
            
            // 获取altitude - 使用与addRoute相同的逻辑
            double altitudeValue = coordinate.altitude();
            
            // 获取高度信息（与addRoute保持一致）
            QVariant altitudeVariant = item->property("altitude");
            QObject* altitudeObj = altitudeVariant.value<QObject*>();
            if (altitudeObj) {
                altitudeValue = altitudeObj->property("rawValue").toDouble();
                qDebug() << "checkRouteUnique: item" << i << "使用altitude.rawValue:" << altitudeValue;
            } else {
                qDebug() << "checkRouteUnique: item" << i << "使用coordinate.altitude():" << altitudeValue;
            }
            
            // 使用与addRoute相同的字段名和精度，但规范化为字符串进行哈希比较
            normalizedWaypoint["latitude"] = QString::number(coordinate.latitude(), 'f', 8);
            normalizedWaypoint["longitude"] = QString::number(coordinate.longitude(), 'f', 8);
            normalizedWaypoint["altitude"] = QString::number(altitudeValue, 'f', 2);
            normalizedWaypoint["sequence"] = sequenceNumber.toInt();
            normalizedWaypoint["command"] = item->property("command").toInt();
            
            qDebug() << "checkRouteUnique: item" << i << "最终altitude:" << altitudeValue 
                     << "坐标:" << coordinate.latitude() << coordinate.longitude()
                     << "命令:" << item->property("command").toInt();
            
            // 获取MAVLink参数（使用与addRoute相同的逻辑）
            QVariant missionItemVariant = item->property("missionItem");
            QObject* missionItem = missionItemVariant.value<QObject*>();
            if (missionItem) {
                QVariant param1, param2, param3, param4;
                QMetaObject::invokeMethod(missionItem, "param1", Q_RETURN_ARG(QVariant, param1));
                QMetaObject::invokeMethod(missionItem, "param2", Q_RETURN_ARG(QVariant, param2));
                QMetaObject::invokeMethod(missionItem, "param3", Q_RETURN_ARG(QVariant, param3));
                QMetaObject::invokeMethod(missionItem, "param4", Q_RETURN_ARG(QVariant, param4));
                
                // 规范化参数（精度限制，避免浮点误差）
                normalizedWaypoint["param1"] = QString::number(param1.toDouble(), 'f', 6);
                normalizedWaypoint["param2"] = QString::number(param2.toDouble(), 'f', 6);
                normalizedWaypoint["param3"] = QString::number(param3.toDouble(), 'f', 6);
                normalizedWaypoint["param4"] = QString::number(param4.toDouble(), 'f', 6);
                
                qDebug() << "checkRouteUnique: item" << i << "参数:" << param1.toDouble() << param2.toDouble() << param3.toDouble() << param4.toDouble();
            } else {
                // 默认值（与addRoute保持一致）
                normalizedWaypoint["param1"] = "0.000000";
                normalizedWaypoint["param2"] = "0.000000";
                normalizedWaypoint["param3"] = "0.000000";
                normalizedWaypoint["param4"] = "0.000000";
                qDebug() << "checkRouteUnique: item" << i << "使用默认参数";
            }
            
            normalizedWaypoints.append(normalizedWaypoint);
        }
    }
    
    // 生成规范化的JSON字符串并计算哈希
    QString normalizedJson = QJsonDocument(normalizedWaypoints).toJson(QJsonDocument::Compact);
    QByteArray inputHash = QCryptographicHash::hash(normalizedJson.toUtf8(), QCryptographicHash::Sha256);
    QString inputHashHex = inputHash.toHex();
    
    qDebug() << "checkRouteUnique: 输入数据JSON:" << normalizedJson;
    qDebug() << "checkRouteUnique: 输入数据哈希:" << inputHashHex;
    
    // 获取数据库中所有航线的航点数据
    QSqlQuery query(_database);
    if (!query.exec("SELECT uuid, waypoints FROM routes")) {
        qDebug() << "checkRouteUnique: 数据库查询失败";
        return QString(); // 查询失败时返回空
    }
    
    int routeIndex = 0;
    // 逐一比较每条航线的哈希值
    while (query.next()) {
        QString routeUuid = query.value("uuid").toString();
        QString existingWaypoints = query.value("waypoints").toString();
        
        qDebug() << "checkRouteUnique: 检查数据库航线" << routeIndex << "UUID:" << routeUuid;
        
        // 解析现有航线的JSON数据并生成相同格式的规范化哈希
        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(existingWaypoints.toUtf8(), &error);
        if (error.error != QJsonParseError::NoError) {
            qDebug() << "checkRouteUnique: 数据库航线" << routeIndex << "JSON解析失败:" << error.errorString();
            continue;
        }
        
        if (!doc.isArray()) {
            qDebug() << "checkRouteUnique: 数据库航线" << routeIndex << "不是数组格式";
            continue;
        }
        
        QJsonArray existingArray = doc.array();
        QJsonArray existingNormalized;
        
        qDebug() << "checkRouteUnique: 数据库航线" << routeIndex << "原始数据:" << existingWaypoints;
        
        // 对现有航线生成相同的规范化格式
        for (const QJsonValue& value : existingArray) {
            if (!value.isObject()) continue;
            
            QJsonObject waypoint = value.toObject();
            QJsonObject normalized;
            
            // 使用相同的字段名和精度
            if (waypoint.contains("latitude")) normalized["latitude"] = QString::number(waypoint["latitude"].toDouble(), 'f', 8);
            if (waypoint.contains("longitude")) normalized["longitude"] = QString::number(waypoint["longitude"].toDouble(), 'f', 8);
            if (waypoint.contains("altitude")) normalized["altitude"] = QString::number(waypoint["altitude"].toDouble(), 'f', 2);
            if (waypoint.contains("sequence")) normalized["sequence"] = waypoint["sequence"].toInt();
            if (waypoint.contains("command")) normalized["command"] = waypoint["command"].toInt();
            
            // 规范化参数
            normalized["param1"] = QString::number(waypoint["param1"].toDouble(), 'f', 6);
            normalized["param2"] = QString::number(waypoint["param2"].toDouble(), 'f', 6);
            normalized["param3"] = QString::number(waypoint["param3"].toDouble(), 'f', 6);
            normalized["param4"] = QString::number(waypoint["param4"].toDouble(), 'f', 6);
            
            existingNormalized.append(normalized);
        }
        
        // 计算现有航线的哈希值
        QString existingNormalizedJson = QJsonDocument(existingNormalized).toJson(QJsonDocument::Compact);
        QByteArray existingHash = QCryptographicHash::hash(existingNormalizedJson.toUtf8(), QCryptographicHash::Sha256);
        QString existingHashHex = existingHash.toHex();
        
        qDebug() << "checkRouteUnique: 数据库航线" << routeIndex << "规范化JSON:" << existingNormalizedJson;
        qDebug() << "checkRouteUnique: 数据库航线" << routeIndex << "哈希:" << existingHashHex;
        qDebug() << "checkRouteUnique: 哈希是否匹配:" << (inputHashHex == existingHashHex ? "YES" : "NO");
        
        if (inputHashHex == existingHashHex) {
            qDebug() << "checkRouteUnique: 找到匹配的航线 UUID:" << routeUuid;
            return routeUuid; // 发现重复，返回匹配的航线UUID
        }
        
        routeIndex++;
    }
    
    qDebug() << "checkRouteUnique: 未找到匹配的航线，返回空字符串";
    return QString(); // 未发现重复，返回空字符串
}

void MissionDatabase::handleDownloadedRoute(const QVariant& visualItems)
{
    if (!_isConnected) {
        qWarning() << "MissionDatabase::handleDownloadedRoute: 数据库未连接";
        return;
    }
    
    qDebug() << "MissionDatabase::handleDownloadedRoute: 开始处理下载的航线";
    
    // 检查下载的航线是否与数据库中的航线重复
    QString matchedRouteUuid = checkRouteUnique(visualItems);
    
    if (!matchedRouteUuid.isEmpty()) {
        // 发现重复航线，设置为当前航线UUID
        setCurrentRouteUuid(matchedRouteUuid);
        qDebug() << "检测到重复航线，设置当前航线UUID:" << matchedRouteUuid;
        
        // 获取航线信息用于显示
        QJsonObject routeInfo = getRoute(matchedRouteUuid);
        if (!routeInfo.isEmpty()) {
            QString routeName = routeInfo["name"].toString();
            qDebug() << "关联到现有航线:" << routeName;
        }
    } else {
        // 未发现重复，需要用户输入新航线名称
        qDebug() << "检测到新航线，发射needNewRouteName信号";
        emit needNewRouteName(QVariant(visualItems));
    }
}

// ==================== 任务表操作 ====================

bool MissionDatabase::addMission(const QString &uuid, const QString &routeUuid, 
                                 const QString &logFileName, const QString &waypoints)
{
    return addMissionWithTime(uuid, routeUuid, getCurrentTimestamp(), logFileName, waypoints);
}

bool MissionDatabase::addMissionWithTime(const QString &uuid, const QString &routeUuid, 
                                         qint64 startTime, const QString &logFileName, const QString &waypoints)
{
    if (!_isConnected) return false;
    
    QSqlQuery query(_database);
    query.prepare("INSERT INTO missions (uuid, route_uuid, start_time, log_file_name, waypoints) "
                  "VALUES (?, ?, ?, ?, ?)");
    query.addBindValue(uuid);
    query.addBindValue(routeUuid);
    query.addBindValue(startTime);
    query.addBindValue(logFileName);
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

QString MissionDatabase::getResultTypeName(int type)
{
    switch (type) {
        case RESULT_TYPE_AIRCRAFT:
            return "飞机";
        case RESULT_TYPE_VEHICLE:
            return "车";
        case RESULT_TYPE_BUILDING:
            return "建筑物";
        default:
            return "未知类型";
    }
}

QJsonObject MissionDatabase::getAllResultTypeNames()
{
    QJsonObject typeNames;
    typeNames["0"] = "飞机";
    typeNames["1"] = "车";
    typeNames["2"] = "建筑物";
    return typeNames;
}


// ==================== 当前航线UUID管理 ====================

void MissionDatabase::setCurrentRouteUuid(const QString &routeUuid)
{
    _currentRouteUuid = routeUuid;
    qDebug() << "设置当前航线UUID:" << routeUuid;
}

QString MissionDatabase::getCurrentRouteUuid()
{
    qDebug() << "获取当前航线UUID:" << _currentRouteUuid;
    return _currentRouteUuid;
}

void MissionDatabase::clearCurrentRouteUuid()
{
    qDebug() << "清除当前航线UUID";
    _currentRouteUuid.clear();
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





 