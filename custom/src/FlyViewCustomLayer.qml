/****************************************************************************
 *
 * (c) 2009-2019 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 * @file
 *   @author Gus Grubba <gus@auterion.com>
 */

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.Palette
import QGroundControl.ScreenTools

import Custom.Widgets
import Custom.Database 1.0

Item {
    property var parentToolInsets                       // These insets tell you what screen real estate is available for positioning the controls in your overlay
    property var totalToolInsets:   _totalToolInsets    // The insets updated for the custom overlay additions
    property var mapControl

    readonly property string noGPS:         qsTr("NO GPS")
    readonly property real   indicatorValueWidth:   ScreenTools.defaultFontPixelWidth * 7

    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property real   _indicatorDiameter:     ScreenTools.defaultFontPixelWidth * 18
    property real   _indicatorsHeight:      ScreenTools.defaultFontPixelHeight
    property var    _sepColor:              qgcPal.globalTheme === QGCPalette.Light ? Qt.rgba(0,0,0,0.5) : Qt.rgba(1,1,1,0.5)
    property color  _indicatorsColor:       qgcPal.text
    property bool   _isVehicleGps:          _activeVehicle ? _activeVehicle.gps.count.rawValue > 1 && _activeVehicle.gps.hdop.rawValue < 1.4 : false
    property string _altitude:              _activeVehicle ? (isNaN(_activeVehicle.altitudeRelative.value) ? "0.0" : _activeVehicle.altitudeRelative.value.toFixed(1)) + ' ' + _activeVehicle.altitudeRelative.units : "0.0"
    property string _distanceStr:           isNaN(_distance) ? "0" : _distance.toFixed(0) + ' ' + QGroundControl.unitsConversion.appSettingsHorizontalDistanceUnitsString
    property real   _heading:               _activeVehicle   ? _activeVehicle.heading.rawValue : 0
    property real   _distance:              _activeVehicle ? _activeVehicle.distanceToHome.rawValue : 0
    property string _messageTitle:          ""
    property string _messageText:           ""
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75

    // ==================== 任务记录相关属性 ====================
    property string _currentMissionUuid:    ""              // 当前任务UUID
    property string _currentRouteUuid:      ""              // 当前关联的航线UUID
    property bool   _missionRecordingActive: false          // 任务记录是否激活
    property bool   _vehicleArmed:          _activeVehicle ? _activeVehicle.armed : false
    property bool   _vehicleFlying:         _activeVehicle ? _activeVehicle.flying : false
    property string _vehicleFlightMode:     _activeVehicle ? _activeVehicle.flightMode : ""
    property string _vehicleMissionMode:    _activeVehicle ? _activeVehicle.missionFlightMode : ""
    property bool   _vehicleInMissionMode:  _vehicleFlightMode === _vehicleMissionMode
    property bool   _missionActive:         _vehicleArmed && _vehicleFlying && _vehicleInMissionMode

    // 添加调试信息，检查组件是否正确加载
    Component.onCompleted: {
        console.log("=== FlyViewCustomLayer: Component.onCompleted ===")
        console.log("MissionDatabase可用:", typeof MissionDatabase !== 'undefined')
    }

    // ==================== 任务记录核心逻辑 ====================
    
    // 监听任务状态变化
    Connections {
        target: _activeVehicle
        function onArmedChanged() { _checkMissionState() }
        function onFlyingChanged() { _checkMissionState() }
        function onFlightModeChanged() { _checkMissionState() }
    }
    
    function _checkMissionState() {
        var newMissionActive = _vehicleArmed && _vehicleFlying && _vehicleInMissionMode
        console.log("检查任务状态:", newMissionActive ? "激活" : "停止")
        if (newMissionActive && !_missionRecordingActive) {
            // 任务开始
            _startMissionRecording()
        } else if (!newMissionActive && _missionRecordingActive) {
            // 任务结束
            _stopMissionRecording()
        }
    }


    // 开始任务记录
    function _startMissionRecording() {
        console.log("=== 开始任务记录 ===")
        
        if (!_activeVehicle) {
            console.log("无活动载具，无法开始任务记录")
            return
        }

        try {
            // 1. 生成任务UUID
            _currentMissionUuid = MissionDatabase.generateUuid()
            console.log("生成任务UUID:", _currentMissionUuid)

            // 2. 从PlanView获取当前routeUuid
            _currentRouteUuid = _getCurrentRouteUuid()
            if (!_currentRouteUuid) {
                console.log("无法获取当前路线UUID")
                return
            }
            console.log("获取到路线UUID:", _currentRouteUuid)

            // 3. 从数据库查询航线信息
            var routeInfo = MissionDatabase.getRoute(_currentRouteUuid)
            if (!routeInfo || !routeInfo.waypoints) {
                console.log("无法从数据库获取航线信息")
                return
            }

            // 4. 生成日志文件名（假数据）
            var logFileName = _generateLogFileName()

            // 5. 创建任务记录
            var success = MissionDatabase.addMission(
                _currentMissionUuid,
                _currentRouteUuid,
                logFileName,
                routeInfo.waypoints   // waypoints: 航点数据
            )

            if (success) {
                _missionRecordingActive = true
                console.log("任务记录创建成功")
                console.log("- 任务UUID:", _currentMissionUuid)
                console.log("- 航线UUID:", _currentRouteUuid)
                console.log("- 日志文件:", logFileName)
            } else {
                console.log("任务记录创建失败")
                _currentMissionUuid = ""
                _currentRouteUuid = ""
            }

        } catch (error) {
            console.log("开始任务记录时发生错误:", error)
            _currentMissionUuid = ""
            _currentRouteUuid = ""
        }
    }

    // 停止任务记录
    function _stopMissionRecording() {
        console.log("=== 停止任务记录 ===")
        
        if (!_missionRecordingActive || !_currentMissionUuid) {
            console.log("任务记录未激活或UUID为空")
            return
        }

        try {
            // 更新任务完成时间
            var success = MissionDatabase.updateMissionEndTime(
                _currentMissionUuid,
                MissionDatabase.getCurrentTimestamp()
            )

            if (success) {
                console.log("任务记录更新成功")
                console.log("- 任务UUID:", _currentMissionUuid)
                console.log("- 完成时间:", new Date().toLocaleString())
            } else {
                console.log("任务记录更新失败")
            }

        } catch (error) {
            console.log("停止任务记录时发生错误:", error)
        } finally {
            // 清理状态
            _missionRecordingActive = false
            _currentMissionUuid = ""
            _currentRouteUuid = ""
        }
    }

    // 从PlanView获取当前routeUuid
    function _getCurrentRouteUuid() {
        console.log("=== _getCurrentRouteUuid 被调用 ===")
        
        // 直接从MissionDatabase获取当前航线UUID
        var routeUuid = MissionDatabase.getCurrentRouteUuid()
        
        if (routeUuid && routeUuid !== "") {
            console.log("从MissionDatabase获取到UUID:", routeUuid)
            // 更新本地缓存
            _currentRouteUuid = routeUuid
            return routeUuid
        }
        
        console.log("MissionDatabase中没有当前航线UUID")
        return null
    }


    // 生成日志文件名（假数据）
    function _generateLogFileName() {
        var now = new Date()
        var timestamp = now.toISOString().slice(0, 19).replace(/[-:]/g, '').replace('T', '_')
        return "flight_log_" + timestamp + ".ulg"
    }

    // ==================== 原有代码继续 ====================

    function secondsToHHMMSS(timeS) {
        var sec_num = parseInt(timeS, 10);
        var hours   = Math.floor(sec_num / 3600);
        var minutes = Math.floor((sec_num - (hours * 3600)) / 60);
        var seconds = sec_num - (hours * 3600) - (minutes * 60);
        if (hours   < 10) {hours   = "0"+hours;}
        if (minutes < 10) {minutes = "0"+minutes;}
        if (seconds < 10) {seconds = "0"+seconds;}
        return hours+':'+minutes+':'+seconds;
    }

    QGCToolInsets {
        id:                     _totalToolInsets
        leftEdgeTopInset:       parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:    exampleRectangle.leftEdgeCenterInset
        leftEdgeBottomInset:    parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      parentToolInsets.rightEdgeTopInset
        rightEdgeCenterInset:   parentToolInsets.rightEdgeCenterInset
        rightEdgeBottomInset:   parentToolInsets.rightEdgeBottomInset  // 使用父级边界值，因为customStatusPanel已隐藏
        topEdgeLeftInset:       parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:     parentToolInsets.topEdgeCenterInset  // 使用父级边界值，因为compassArrowIndicator已隐藏
        topEdgeRightInset:      parentToolInsets.topEdgeRightInset
        bottomEdgeLeftInset:    parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  parentToolInsets.bottomEdgeCenterInset
        bottomEdgeRightInset:   parentToolInsets.bottomEdgeRightInset  // 使用父级边界值，因为attitudeIndicator已隐藏
    }

    // This is an example of how you can use parent tool insets to position an element on the custom fly view layer
    // - we use parent topEdgeLeftInset to position the widget below the toolstrip
    // - we use parent bottomEdgeLeftInset to dodge the virtual joystick if enabled
    // - we use the parent leftEdgeTopInset to size our element to the same width as the ToolStripAction
    // - we export the width of this element as the leftEdgeCenterInset so that the map will recenter if the vehicle flys behind this element
    Rectangle {
        id: exampleRectangle
        visible: false // to see this example, set this to true. To view insets, enable the insets viewer FlyView.qml
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.topMargin: parentToolInsets.topEdgeLeftInset + _toolsMargin
        anchors.bottomMargin: parentToolInsets.bottomEdgeLeftInset + _toolsMargin
        anchors.leftMargin: _toolsMargin
        width: parentToolInsets.leftEdgeTopInset - _toolsMargin
        color: 'red'

        property real leftEdgeCenterInset: visible ? x + width : 0
    }

    //-------------------------------------------------------------------------
    //-- Heading Indicator
    Rectangle {
        id:                         compassBar
        height:                     ScreenTools.defaultFontPixelHeight * 1.5
        width:                      ScreenTools.defaultFontPixelWidth  * 50
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       _toolsMargin
        color:                      "#DEDEDE"
        radius:                     2
        clip:                       true
        anchors.horizontalCenter:   parent.horizontalCenter
        visible:                    false  // 隐藏罗盘方向条
        Repeater {
            model: 720
            QGCLabel {
                function _normalize(degrees) {
                    var a = degrees % 360
                    if (a < 0) a += 360
                    return a
                }
                property int _startAngle: modelData + 180 + _heading
                property int _angle: _normalize(_startAngle)
                anchors.verticalCenter: parent.verticalCenter
                x:              visible ? ((modelData * (compassBar.width / 360)) - (width * 0.5)) : 0
                visible:        _angle % 45 == 0
                color:          "#75505565"
                font.pointSize: ScreenTools.smallFontPointSize
                text: {
                    switch(_angle) {
                    case 0:     return "N"
                    case 45:    return "NE"
                    case 90:    return "E"
                    case 135:   return "SE"
                    case 180:   return "S"
                    case 225:   return "SW"
                    case 270:   return "W"
                    case 315:   return "NW"
                    }
                    return ""
                }
            }
        }
    }
    Rectangle {
        id:                         headingIndicator
        height:                     ScreenTools.defaultFontPixelHeight
        width:                      ScreenTools.defaultFontPixelWidth * 4
        color:                      qgcPal.windowShadeDark
        anchors.top:                compassBar.top
        anchors.topMargin:          -headingIndicator.height / 2
        anchors.horizontalCenter:   parent.horizontalCenter
        visible:                    false  // 隐藏航向数字指示器
        QGCLabel {
            text:                   _heading
            color:                  qgcPal.text
            font.pointSize:         ScreenTools.smallFontPointSize
            anchors.centerIn:       parent
        }
    }
    Image {
        id:                         compassArrowIndicator
        height:                     _indicatorsHeight
        width:                      height
        source:                     "/custom/img/compass_pointer.svg"
        fillMode:                   Image.PreserveAspectFit
        sourceSize.height:          height
        anchors.top:                compassBar.bottom
        anchors.topMargin:          -height / 2
        anchors.horizontalCenter:   parent.horizontalCenter
        visible:                    false  // 隐藏罗盘箭头指示器
    }

    // 自定义状态面板 - 位于罗盘左侧
    Loader {
        id:                     customStatusPanel
        anchors.bottom:         compassBackground.bottom
        anchors.right:          compassBackground.left
        anchors.rightMargin:    _toolsMargin
        source:                 "qrc:/Custom/qml/QGroundControl/FlightDisplay/CustomStatusPanel.qml"
        visible:                false  // 隐藏自定义状态面板
    }



    Rectangle {
        id:                     compassBackground
        anchors.bottom:         attitudeIndicator.bottom
        anchors.right:          attitudeIndicator.left
        anchors.rightMargin:    -attitudeIndicator.width / 2
        width:                  -anchors.rightMargin + compassBezel.width + (_toolsMargin * 2)
        height:                 attitudeIndicator.height * 0.75
        radius:                 2
        color:                  qgcPal.window
        visible:                false  // 隐藏罗盘背景

        Rectangle {
            id:                     compassBezel
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin:     _toolsMargin
            anchors.left:           parent.left
            width:                  height
            height:                 parent.height - (northLabelBackground.height / 2) - (headingLabelBackground.height / 2)
            radius:                 height / 2
            border.color:           qgcPal.text
            border.width:           1
            color:                  Qt.rgba(0,0,0,0)
        }

        Rectangle {
            id:                         northLabelBackground
            anchors.top:                compassBezel.top
            anchors.topMargin:          -height / 2
            anchors.horizontalCenter:   compassBezel.horizontalCenter
            width:                      northLabel.contentWidth * 1.5
            height:                     northLabel.contentHeight * 1.5
            radius:                     ScreenTools.defaultFontPixelWidth  * 0.25
            color:                      qgcPal.windowShade

            QGCLabel {
                id:                 northLabel
                anchors.centerIn:   parent
                text:               "N"
                color:              qgcPal.text
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }

        Image {
            id:                 headingNeedle
            anchors.centerIn:   compassBezel
            height:             compassBezel.height * 0.75
            width:              height
            source:             "/custom/img/compass_needle.svg"
            fillMode:           Image.PreserveAspectFit
            sourceSize.height:  height
            transform: [
                Rotation {
                    origin.x:   headingNeedle.width  / 2
                    origin.y:   headingNeedle.height / 2
                    angle:      _heading
                }]
        }

        Rectangle {
            id:                         headingLabelBackground
            anchors.top:                compassBezel.bottom
            anchors.topMargin:          -height / 2
            anchors.horizontalCenter:   compassBezel.horizontalCenter
            width:                      headingLabel.contentWidth * 1.5
            height:                     headingLabel.contentHeight * 1.5
            radius:                     ScreenTools.defaultFontPixelWidth  * 0.25
            color:                      qgcPal.windowShade

            QGCLabel {
                id:                 headingLabel
                anchors.centerIn:   parent
                text:               _heading
                color:              qgcPal.text
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }
    }

    Rectangle {
        id:                     attitudeIndicator
        anchors.bottomMargin:   _toolsMargin + parentToolInsets.bottomEdgeRightInset
        anchors.rightMargin:    _toolsMargin
        anchors.bottom:         parent.bottom
        anchors.right:          parent.right
        height:                 ScreenTools.defaultFontPixelHeight * 6
        width:                  height
        radius:                 height * 0.5
        color:                  qgcPal.windowShade
        visible:                false  // 隐藏姿态指示器

        CustomAttitudeWidget {
            size:               parent.height * 0.95
            vehicle:            _activeVehicle
            showHeading:        false
            anchors.centerIn:   parent
        }
    }

    // ==================== 任务记录状态显示 ====================
    Rectangle {
        id:                     missionRecordingStatus
        anchors.top:            parent.top
        anchors.topMargin:      _toolsMargin + parentToolInsets.topEdgeRightInset
        anchors.right:          parent.right
        anchors.rightMargin:    _toolsMargin
        width:                  ScreenTools.defaultFontPixelWidth * 25
        height:                 ScreenTools.defaultFontPixelHeight * 8
        radius:                 ScreenTools.defaultFontPixelWidth * 0.5
        color:                  qgcPal.windowShade
        border.color:           _missionRecordingActive ? "green" : "gray"
        border.width:           2
        visible:                _activeVehicle !== null

        Column {
            anchors.fill:       parent
            anchors.margins:    ScreenTools.defaultFontPixelWidth * 0.5
            spacing:            ScreenTools.defaultFontPixelHeight * 0.25

            // 标题
            QGCLabel {
                text:               "任务记录状态"
                font.bold:          true
                font.pointSize:     ScreenTools.smallFontPointSize
                color:              qgcPal.text
                anchors.horizontalCenter: parent.horizontalCenter
            }

            // 分隔线
            Rectangle {
                width:              parent.width
                height:             1
                color:              qgcPal.text
                opacity:            0.3
            }

            // 记录状态
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                QGCLabel {
                    text:           "记录状态:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           _missionRecordingActive ? "激活" : "停止"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          _missionRecordingActive ? "green" : "red"
                    font.bold:      true
                }
            }

            // 任务状态
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                QGCLabel {
                    text:           "任务状态:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           _missionActive ? "执行中" : "未执行"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          _missionActive ? "green" : "gray"
                }
            }

            // 载具状态
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                QGCLabel {
                    text:           "载具状态:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           _vehicleArmed ? (_vehicleFlying ? "飞行中" : "已解锁") : "未解锁"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          _vehicleArmed ? (_vehicleFlying ? "green" : "orange") : "gray"
                }
            }

            // 飞行模式
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                QGCLabel {
                    text:           "飞行模式:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           _vehicleFlightMode || "未知"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          _vehicleInMissionMode ? "green" : "gray"
                }
            }

            // 任务UUID（如果有）
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                visible:            _currentMissionUuid !== ""
                QGCLabel {
                    text:           "任务ID:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           _currentMissionUuid.substring(0, 8) + "..."
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
            }

            // 调试信息
            Row {
                spacing:            ScreenTools.defaultFontPixelWidth * 0.5
                visible:            true // 设置为true以显示调试信息
                QGCLabel {
                    text:           "调试:"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                }
                QGCLabel {
                    text:           "A:" + (_vehicleArmed ? "1" : "0") + 
                                   " F:" + (_vehicleFlying ? "1" : "0") + 
                                   " M:" + (_vehicleInMissionMode ? "1" : "0")
                    font.pointSize: ScreenTools.smallFontPointSize
                    color:          qgcPal.text
                    font.family:    "monospace"
                }
            }
        }
    }
}
