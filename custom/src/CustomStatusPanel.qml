/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.ScreenTools
import QGroundControl.Palette
import Custom.UdpLink

Rectangle {
    id: statusPanel
    
    // 属性
    property var activeVehicle: QGroundControl.multiVehicleManager.activeVehicle
    property bool expanded: false
    
    // 状态数据 - 使用activeVehicle的数据
    property real latitude: activeVehicle ? activeVehicle.coordinate.latitude : 0
    property real longitude: activeVehicle ? activeVehicle.coordinate.longitude : 0
    property real altitude: activeVehicle ? activeVehicle.altitudeRelative.rawValue : 0
    
    // 姿态数据 - 使用activeVehicle的数据
    property real yaw: activeVehicle ? activeVehicle.heading.rawValue : 0
    property real pitch: activeVehicle ? activeVehicle.pitch.rawValue : 0
    property real roll: activeVehicle ? activeVehicle.roll.rawValue : 0
    
    // 尺寸 - 改为横向扩充
    property real collapsedWidth: ScreenTools.defaultFontPixelWidth * 3
    property real expandedWidth: ScreenTools.defaultFontPixelWidth * 35  // 增加宽度以容纳更多数据
    
    width: expanded ? expandedWidth : collapsedWidth
    height: ScreenTools.defaultFontPixelHeight * 4  // 减少高度，因为改为横向布局
    radius: ScreenTools.defaultFontPixelWidth * 0.25
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    
    // 动画
    Behavior on width {
        NumberAnimation {
            duration: 250
            easing.type: Easing.OutCubic
        }
    }
    
    QGCPalette { id: qgcPal; colorGroupEnabled: enabled }
    
    // 组件初始化时启动状态接收器
    Component.onCompleted: {
        statusReceiver.startReceiving()
    }
    
    // 组件销毁时停止状态接收器
    Component.onDestruction: {
        statusReceiver.stopReceiving()
    }
    
    // 收缩状态指示器
    Rectangle {
        id: collapsedIndicator
        anchors.centerIn: parent
        width: parent.width * 0.5
        height: ScreenTools.defaultFontPixelHeight * 0.6
        radius: width * 0.3
        // 根据数据源改变颜色：UDP连接=绿色，QGC连接=橙色，无连接=灰色
        color: statusReceiver.connected ? "#4CAF50" : 
               (activeVehicle ? "#FF9800" : "#9E9E9E")
        visible: !expanded
        
        QGCLabel {
            anchors.centerIn: parent
            text: "⋯"
            color: qgcPal.window
            font.pointSize: ScreenTools.smallFontPointSize
        }
    }
    
    // 展开状态内容
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5
        spacing: ScreenTools.defaultFontPixelHeight * 0.1
        visible: expanded
        
        // 标题行
        RowLayout {
            Layout.fillWidth: true
            
            QGCLabel {
                text: "状态信息 " + (statusReceiver.connected ? "(UDP)" : "(QGC)")
                font.pointSize: ScreenTools.smallFontPointSize
                font.bold: true
                color: qgcPal.text
                Layout.fillWidth: true
            }
            
            // 修复叉号按钮的样式和位置
            Rectangle {
                Layout.preferredWidth: ScreenTools.defaultFontPixelHeight * 0.8
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 0.8
                radius: width * 0.2
                color: mouseArea.pressed ? qgcPal.buttonHighlight : qgcPal.button
                border.width: 1
                border.color: qgcPal.text
                
                QGCLabel {
                    anchors.centerIn: parent
                    text: "×"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.buttonText
                }
                
                MouseArea {
                    id: mouseArea
                    anchors.fill: parent
                    onClicked: statusPanel.expanded = false
                }
            }
        }
        
        // 横向数据布局
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: ScreenTools.defaultFontPixelWidth * 0.8
            
            // 位置信息组
            ColumnLayout {
                spacing: 2
                
                QGCLabel {
                    text: "位置"
                    font.pointSize: ScreenTools.smallFontPointSize * 0.9
                    font.bold: true
                    color: qgcPal.text
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                }
                
                QGCLabel {
                    text: "纬度: " + ((statusReceiver.connected || activeVehicle) ? latitude.toFixed(4) + "°" : "---°")
                    font.pointSize: ScreenTools.smallFontPointSize * 0.8
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                }
                
                QGCLabel {
                    text: "经度: " + ((statusReceiver.connected || activeVehicle) ? longitude.toFixed(4) + "°" : "---°")
                    font.pointSize: ScreenTools.smallFontPointSize * 0.8
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                }
            }
            
            // 分隔线
            Rectangle {
                width: 1
                Layout.fillHeight: true
                color: qgcPal.text
                opacity: 0.3
            }
            
            // 高度信息组
            ColumnLayout {
                spacing: 2
                
                QGCLabel {
                    text: "高度"
                    font.pointSize: ScreenTools.smallFontPointSize * 0.9
                    font.bold: true
                    color: qgcPal.text
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                }
                
                QGCLabel {
                    text: ((statusReceiver.connected || activeVehicle) ? altitude.toFixed(1) + "m" : "---m")
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                }
            }
            
            // 分隔线
            Rectangle {
                width: 1
                Layout.fillHeight: true
                color: qgcPal.text
                opacity: 0.3
            }
            
            // 姿态信息组
            ColumnLayout {
                spacing: 2
                
                QGCLabel {
                    text: "姿态"
                    font.pointSize: ScreenTools.smallFontPointSize * 0.9
                    font.bold: true
                    color: qgcPal.text
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                }
                
                QGCLabel {
                    text: "Yaw: " + ((statusReceiver.connected || activeVehicle) ? yaw.toFixed(1) + "°" : "---°")
                    font.pointSize: ScreenTools.smallFontPointSize * 0.8
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                }
                
                QGCLabel {
                    text: "Pitch: " + ((statusReceiver.connected || activeVehicle) ? pitch.toFixed(1) + "°" : "---°")
                    font.pointSize: ScreenTools.smallFontPointSize * 0.8
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                }
                
                QGCLabel {
                    text: "Roll: " + ((statusReceiver.connected || activeVehicle) ? roll.toFixed(1) + "°" : "---°")
                    font.pointSize: ScreenTools.smallFontPointSize * 0.8
                    color: statusReceiver.connected ? "#4CAF50" : qgcPal.text
                }
            }
        }
        
        // 状态指示
        Rectangle {
            Layout.fillWidth: true
            height: ScreenTools.defaultFontPixelHeight * 0.6
            radius: height * 0.3
            color: {
                if (statusReceiver.connected) {
                    return "#4CAF50"  // UDP连接 - 绿色
                } else if (activeVehicle) {
                    return activeVehicle.armed ? "#4CAF50" : "#FF9800"  // QGC连接 - 根据解锁状态
                } else {
                    return "#9E9E9E"  // 无连接 - 灰色
                }
            }
            
            QGCLabel {
                anchors.centerIn: parent
                text: {
                    if (statusReceiver.connected) {
                        return "UDP已连接"
                    } else if (activeVehicle) {
                        return activeVehicle.armed ? "已解锁" : "未解锁"
                    } else {
                        return "未连接"
                    }
                }
                font.pointSize: ScreenTools.smallFontPointSize * 0.8
                color: "white"
            }
        }
    }
    
    // 点击区域
    MouseArea {
        anchors.fill: parent
        enabled: !expanded
        onClicked: statusPanel.expanded = true
    }
    
    // 监听数据变化
    Connections {
        target: statusReceiver
        function onConnectedChanged() {
            // 连接状态变化时自动更新UI
        }
        
        function onPositionChanged() {
            // 位置数据变化时自动更新UI
        }
        
        function onAttitudeDataChanged() {
            // 姿态数据变化时自动更新UI
        }
    }
} 