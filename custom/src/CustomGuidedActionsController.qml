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
    
    Component.onCompleted: {
        // 尝试获取应用程序根窗口
        var obj = parent
        while (obj && obj.parent) {
            obj = obj.parent
        }
        _applicationWindow = obj
    }
    // 用于存储当前操作的上下文信息
    property var _currentActionData: null
    property var _currentMapIndicator: null
    readonly property int actionCustomTakeoff: 10000 + 0  // customActionStart = 10000
    readonly property int actionCustomHome: 10000 + 1
    
    // 起飞/结束状态控制
    property bool _isTakeoffMode: true  // true=起飞模式, false=结束模式
    property bool _canTakeoff: false    // 是否允许起飞，默认false，真实数据来自UDP接收
    
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
    
    // 更新起飞许可状态（供UDP接收器调用）
    function updateTakeoffPermission(canTakeoff) {
        var oldValue = _canTakeoff
        _canTakeoff = canTakeoff
        
        if (oldValue !== canTakeoff) {
            console.log("[INFO] 起飞许可状态更新:", canTakeoff ? "允许" : "禁止")
        }
    }
    
    // 重置起飞/结束状态到初始状态（起飞模式）
    function resetTakeoffState() {
        _isTakeoffMode = true
        _canTakeoff = false  // 重置时同时重置起飞许可
        console.log("[INFO] 起飞状态已重置为起飞模式，起飞许可重置为禁止")
    }
    
    function _handleMyCustomTakeoff() {
        // 根据当前状态决定param1的值
        var param1Value = _isTakeoffMode ? 1.0 : 0.0
        var actionName = _isTakeoffMode ? "起飞" : "结束"
        
        console.log("[INFO] 执行" + actionName + "命令，param1=" + param1Value)
        
        // 发送自定义MAVLink命令
        var success = CustomCommandSender.sendCommandByUdp(
            0,      // compId (目标组件ID)
            31010,  // MAV_CMD_USER_1
            true,   // showError
            param1Value,  // param1: 1.0=起飞, 0.0=结束
            0, 0, 0, 0, 0, 0  // 其他参数
        )
        
        if (success) {
            // 命令发送成功，切换状态
            _isTakeoffMode = !_isTakeoffMode
            console.log("[INFO] " + actionName + "命令发送成功，切换到" + (_isTakeoffMode ? "起飞" : "结束") + "模式")
        } else {
            console.error("[UDP] 自定义" + actionName + "命令发送失败")
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
        
        // 通过UDP发送设置Home位置的MAVLink命令
        var success = CustomCommandSender.sendCommandByUdp(
                0,      // compId (目标组件ID)
                31011,  // MAV_CMD_USER_2
                true,   // showError
                1.0,    // param1 = 1.0 表示使用自定义坐标
                latitude, longitude, altitude, 0, 0, 0  // 坐标参数
            )
        
        if (!success) {
            console.error("[失败] Home位置设置命令发送失败")
        }
    }

}