/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

// Custom builds can override this file to add custom guided actions.

import QtQml
import QtQuick
import Custom.UdpLink 1.0

QtObject {
    // 获取应用程序根对象的引用
    property var _applicationWindow: null
    
    // 消息类型常量 (与CustomUdpTypes.h中的MessageType枚举对应)
    readonly property int msgHeartbeat: 0
    readonly property int msgTakeoffCommand: 1
    readonly property int msgSetHomeCommand: 2
    readonly property int msgMissionStatus: 3
    readonly property int msgLocationStatus: 4
    readonly property int msgAckResponse: 5
    readonly property int msgError: 6
    
    Component.onCompleted: {
        // 尝试获取应用程序根窗口
        var obj = parent
        while (obj && obj.parent) {
            obj = obj.parent
        }
        _applicationWindow = obj
        
        // 连接JSON消息接收信号
        SimpleMavlinkUdp.jsonReceived.connect(_onJsonReceived)
        SimpleMavlinkUdp.commandAckReceived.connect(_onCommandAckReceived)
    }
    
    // 用于存储当前操作的上下文信息
    property var _currentActionData: null
    property var _currentMapIndicator: null
    readonly property int actionCustomTakeoff: 10000 + 0  // customActionStart = 10000
    readonly property int actionCustomHome: 10000 + 1
    
    // 起飞/结束状态控制 - 独立的按钮状态
    property int _missionState: SimpleMavlinkUdp.missionState
    property bool _isTakeoffMode: true  // 独立状态：true=起飞模式，false=结束模式
    property bool _canTakeoff: (_missionState === 1)    // 只有就绪状态才能起飞
    
    // 地图点击模式控制
    property bool homePositionMapClickMode: false
    property var activeHomePositionDialog: null

    // 动态按钮文字和标题
    property string customButtonTitle: _isTakeoffMode ? qsTr("我的起飞") : qsTr("结束任务")
    readonly property string customHomeTitle: qsTr("设置Home")

    property string customButtonMessage: _isTakeoffMode ? 
        qsTr("执行自定义起飞逻辑，无需检查载具状态") : 
        qsTr("结束当前任务并返回待机状态")
    readonly property string customHomeMessage: qsTr("执行自定义设置Home逻辑")

    // UUID生成函数 - 标准UUID v4格式
    function generateMissionUuid() {
        // 生成标准UUID格式：xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
        function randomHex() {
            return Math.floor(Math.random() * 16).toString(16);
        }
        
        function randomHex4() {
            return randomHex() + randomHex() + randomHex() + randomHex();
        }
        
        function randomHex8() {
            return randomHex4() + randomHex4();
        }
        
        // UUID v4 格式：xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
        var uuid = randomHex8() + "-" +                    // 8位
                   randomHex4() + "-" +                    // 4位  
                   "4" + randomHex() + randomHex() + randomHex() + "-" +  // 4xxx (版本4)
                   (8 + Math.floor(Math.random() * 4)).toString(16) + randomHex() + randomHex() + randomHex() + "-" +  // yxxx (变体位)
                   randomHex8() + randomHex4();            // 12位
        
        return uuid;
    }
    
    // JSON消息接收处理
    function _onJsonReceived(messageType, data) {
        console.log("[JSON] 收到消息类型:", messageType, "数据:", JSON.stringify(data))
        
        // 可以根据需要处理特定的JSON消息
        if (messageType === msgAckResponse) {
            // 命令确认响应已经通过commandAckReceived信号处理
        }
    }
    
    function _onCommandAckReceived(success, message) {
        console.log("[命令确认]", success ? "成功" : "失败", ":", message)
        
        if (success) {
            // 命令执行成功，可以根据需要更新UI状态
        } else {
            // 命令执行失败，显示错误信息
            console.error("[错误] 命令执行失败:", message)
        }
    }

    function customConfirmAction(actionCode, actionData, mapIndicator, confirmDialog) {
        // 保存当前操作的上下文信息
        _currentActionData = actionData
        _currentMapIndicator = mapIndicator

        
        switch (actionCode) {
        case actionCustomTakeoff:
            confirmDialog.hideTrigger = true
            confirmDialog.title = customButtonTitle
            confirmDialog.message = customButtonMessage
            break
        case actionCustomHome:
            confirmDialog.hideTrigger = true  // 隐藏滑动确认，直接执行
            confirmDialog.title = customHomeTitle
            confirmDialog.message = customHomeMessage
            break
        default:
            return false // false = action not handled here
        }

        return true // true = action handled here
    }

    function customExecuteAction(actionCode, actionData, sliderOutputValue, optionChecked) {
        switch (actionCode) {
        case actionCustomTakeoff:
            _handleMyCustomTakeoff()
            break
        case actionCustomHome:
            _handleMyCustomHome()
            break
        default:
            return false // false = action not handled here
        }

        return true // true = action handled here
    }
    
    // 处理地图点击事件（供FlyViewMap调用）
    function handleMapClickForHomePosition(coordinate) {
        if (homePositionMapClickMode && activeHomePositionDialog) {
            // 验证坐标有效性
            if (!coordinate || !coordinate.isValid) {
                console.error("[错误] 无效的地图坐标")
                return false
            }
            
            activeHomePositionDialog.setCoordinateFromMap(coordinate)
            return true
        }
        return false
    }
    
    function _handleMyCustomHome() {
        // 防止重复创建对话框
        if (activeHomePositionDialog) {
            return
        }
        
        // 直接显示坐标输入对话框，无需滑动确认
        _showHomePositionDialog()
    }
    
    function _showHomePositionDialog() {
        
        // 创建对话框组件
        var component = Qt.createComponent("qrc:/Custom/HomePositionInputDialog.qml")
        
        if (component.status === Component.Ready) {
            
            // 使用应用程序根窗口作为父对象
            var parentObject = _applicationWindow
            
            var dialog = component.createObject(parentObject, {
                // 设置初始坐标（可以使用当前地图中心或默认值）
                latitude: _currentActionData && _currentActionData.coordinate ? _currentActionData.coordinate.latitude : 39.9042,
                longitude: _currentActionData && _currentActionData.coordinate ? _currentActionData.coordinate.longitude : 116.4074,
                altitude: 50.0
            })
            
            if (dialog) {
                // 设置活动对话框和地图点击模式
                activeHomePositionDialog = dialog
                homePositionMapClickMode = true
                
                // 连接信号
                dialog.accepted.connect(function(lat, lon, alt) {
                    _executeSetHome(lat, lon, alt)
                    _cleanupHomePositionDialog(dialog)
                })
                
                dialog.rejected.connect(function() {
                    _cleanupHomePositionDialog(dialog)
                })

            } else {
                console.error("无法创建对话框对象")
                // 清理状态，防止状态不一致
                homePositionMapClickMode = false
                activeHomePositionDialog = null
            }
        } else if (component.status === Component.Loading) {
            // 添加超时保护，防止无限等待
            var timeoutTimer = Qt.createQmlObject(
                'import QtQuick; Timer { interval: 5000; repeat: false }',
                _applicationWindow
            )
            
            var statusHandler = function() {
                if (component.status === Component.Ready) {
                    timeoutTimer.stop()
                    // 检查是否仍需要创建对话框（用户可能已取消）
                    if (!activeHomePositionDialog) {
                        _showHomePositionDialog()
                    }
                } else if (component.status === Component.Error) {
                    timeoutTimer.stop()
                    console.error("组件加载失败:", component.errorString())
                    // 清理状态
                    homePositionMapClickMode = false
                    activeHomePositionDialog = null
                }
            }
            
            component.statusChanged.connect(statusHandler)
            
            // 超时处理
            timeoutTimer.timeout.connect(function() {
                console.error("组件加载超时")
                component.statusChanged.disconnect(statusHandler)
                homePositionMapClickMode = false
                activeHomePositionDialog = null
                timeoutTimer.destroy()
            })
            
            timeoutTimer.start()
        } else {
            console.error("无法加载对话框组件:", component.errorString())
        }
    }
    
    function _cleanupHomePositionDialog(dialog) {
        // 清理地图点击模式
        homePositionMapClickMode = false
        activeHomePositionDialog = null
        
        // 销毁对话框
        if (dialog) {
            dialog.destroy()
        }
    }
    
    function _handleMyCustomTakeoff() {
        // 基于当前按钮模式确定操作
        var action = _isTakeoffMode ? "takeoff" : "land"
        var missionUuid = generateMissionUuid()
        var altitude = 50.0
        
        console.log("[INFO] 执行" + action + "命令，任务UUID:", missionUuid)
        
        // 使用新的JSON接口发送起飞命令
        var success = SimpleMavlinkUdp.sendTakeoffCommand(missionUuid, action, altitude)
        
        if (success) {
            console.log("[INFO]", action, "命令发送成功，任务UUID:", missionUuid)
            // 切换按钮状态：起飞后变成结束模式，结束后变成起飞模式
            _isTakeoffMode = !_isTakeoffMode
        } else {
            console.error("[ERROR] 自定义", action, "命令发送失败")
        }
    }

    function _executeSetHome(latitude, longitude, altitude) {
        // 验证坐标有效性
        if (isNaN(latitude) || isNaN(longitude) || isNaN(altitude)) {
            console.error("[错误] 坐标包含无效值:", latitude, longitude, altitude)
            return
        }
        
        if (latitude < -90 || latitude > 90) {
            console.error("[错误] 纬度超出范围:", latitude)
            return
        }
        
        if (longitude < -180 || longitude > 180) {
            console.error("[错误] 经度超出范围:", longitude)
            return
        }
        
        // 使用JSON接口发送设置Home位置的命令
        var commandData = {
            "coordinate": {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude
            },
            "use_current_position": false
        }
        
        var success = SimpleMavlinkUdp.sendJson(msgSetHomeCommand, commandData)
        
        if (success) {
            console.log("[成功] Home位置设置命令发送成功:", latitude, longitude, altitude)
        } else {
            console.error("[失败] Home位置设置命令发送失败")
        }
    }

}