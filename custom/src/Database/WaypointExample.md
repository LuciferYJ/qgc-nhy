# 航点数据库使用示例

## 数据库表结构

### routes表（航线表）
```sql
CREATE TABLE routes (
    uuid TEXT PRIMARY KEY,           -- 航线唯一标识
    modify_time INTEGER NOT NULL,    -- 修改时间戳
    name TEXT NOT NULL,              -- 航线名称
    waypoint_count INTEGER NOT NULL, -- 航点数量
    route_length REAL NOT NULL,      -- 航线长度
    estimated_duration INTEGER NOT NULL, -- 预计飞行时长
    waypoints TEXT NOT NULL          -- 航点数据(JSON格式)
)
```

### missions表（任务表）
```sql
CREATE TABLE missions (
    uuid TEXT PRIMARY KEY,           -- 任务唯一标识
    route_uuid TEXT NOT NULL,        -- 关联的航线UUID
    start_time INTEGER NOT NULL,     -- 任务开始时间戳
    end_time INTEGER DEFAULT 0,      -- 任务结束时间戳
    log_file_name TEXT,              -- 关联的日志文件名
    result_uuid TEXT,                -- 关联的成果UUID
    waypoints TEXT NOT NULL          -- 航点数据(JSON格式)
)
```

### results表（成果表）
```sql
CREATE TABLE results (
    uuid TEXT PRIMARY KEY,           -- 成果唯一标识
    mission_uuid TEXT NOT NULL,      -- 关联的任务UUID
    result_data TEXT NOT NULL        -- 成果数据(JSON格式，包含多个成果：图片路径和类别)
)
```

## 航点数据格式

### 单个航点JSON格式
```json
{
    "longitude": 116.3974,  // 经度
    "latitude": 39.9093,    // 纬度
    "altitude": 100.0,      // 高度
    "type": 1               // 航点类型(int)
}
```

### 航点数组JSON格式
```json
[
    {
        "longitude": 116.3974,
        "latitude": 39.9093,
        "altitude": 100.0,
        "type": 1
    },
    {
        "longitude": 116.3984,
        "latitude": 39.9103,
        "altitude": 120.0,
        "type": 2
    }
]
```

### 成果数据JSON格式
```json
[
    {
        "image_path": "/path/to/image1.jpg",
        "category": "目标A"
    },
    {
        "image_path": "/path/to/image2.jpg", 
        "category": "目标B"
    }
]
```

## 使用示例

### 1. 创建航点数据
```javascript
// 在QML中使用
var waypoints = [
    {
        "longitude": 116.3974,
        "latitude": 39.9093,
        "altitude": 100.0,
        "type": 1
    },
    {
        "longitude": 116.3984,
        "latitude": 39.9103,
        "altitude": 120.0,
        "type": 2
    }
];

// 转换为JSON字符串
var waypointsJson = JSON.stringify(waypoints);
```

### 2. 添加航线
```javascript
// 在QML中调用数据库方法
var routeUuid = missionDatabase.generateUuid();
var success = missionDatabase.addRoute(
    routeUuid,           // uuid
    "测试航线1",         // name
    2,                   // waypoint_count
    1500.0,             // route_length
    300,                // estimated_duration
    waypointsJson       // waypoints
);

// 添加任务
var missionUuid = missionDatabase.generateUuid();
var success = missionDatabase.addMission(
    missionUuid,         // uuid
    routeUuid,           // route_uuid
    "flight_log.txt",    // log_file_name
                    // route_backup 字段已移除
    waypointsJson        // waypoints
);
```

### 3. 查询航线
```javascript
// 获取单个航线
var route = missionDatabase.getRoute(routeUuid);
if (route.uuid) {
    console.log("航线名称:", route.name);
    console.log("航点数量:", route.waypoint_count);
    
    // 解析航点数据
    var waypoints = JSON.parse(route.waypoints);
    for (var i = 0; i < waypoints.length; i++) {
        var wp = waypoints[i];
        console.log("航点" + (i+1) + ":", wp.longitude, wp.latitude, wp.altitude, wp.type);
    }
}
```

### 4. 更新航线
```javascript
// 修改航点数据
waypoints[0].altitude = 150.0;  // 修改第一个航点的高度
var newWaypointsJson = JSON.stringify(waypoints);

// 更新航线
var success = missionDatabase.updateRoute(
    routeUuid,
    "测试航线1(已修改)",
    2,
    1500.0,
    300,
    newWaypointsJson
);
```

### 5. 使用工具函数
```javascript
// 创建单个航点JSON
var waypointJson = missionDatabase.createWaypointJson(116.3974, 39.9093, 100.0, 1);

// 解析航点JSON数组
var waypointsArray = missionDatabase.parseWaypointsJson(route.waypoints);

// 创建航点JSON数组
var waypointsJson = missionDatabase.createWaypointsJson(waypointsArray);

// 创建单个成果JSON
var resultJson = missionDatabase.createResultJson("/path/to/image.jpg", "目标A");

// 创建成果数组
var results = [
    {"image_path": "/path/to/image1.jpg", "category": "目标A"},
    {"image_path": "/path/to/image2.jpg", "category": "目标B"}
];
var resultsJson = missionDatabase.createResultsJson(results);

// 解析成果JSON数组
var resultsArray = missionDatabase.parseResultsJson(resultsJson);
```

## 航点类型说明

根据您的需求，可以定义不同的航点类型：
- `1`: 普通航点
- `2`: 起飞点
- `3`: 降落点
- `4`: 拍照点
- `5`: 悬停点
- 等等...

## 注意事项

1. 航点数据以JSON格式存储，便于扩展和解析
2. `waypoint_count`字段应与实际航点数量保持一致
3. 坐标系统建议使用WGS84（GPS坐标系）
4. 高度单位建议使用米
5. 数据库会自动处理架构迁移，为现有表添加waypoints字段 