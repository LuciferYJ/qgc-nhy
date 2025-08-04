/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlightDisplay
import QGroundControl.ScreenTools
import QGroundControl.Palette
import Custom.UdpLink

// Custom版本：隐藏遥测值显示，添加自定义状态面板
ColumnLayout {
    spacing: ScreenTools.defaultFontPixelHeight * 0.5
    
    // 第一行：TelemetryValuesBar 和自定义状态面板水平对齐
RowLayout {
        Layout.fillWidth: true
        spacing: 0
        
        // TelemetryValuesBar - 左边
    TelemetryValuesBar {
            id: telemetryBar
            Layout.alignment: Qt.AlignBottom | Qt.AlignLeft
            extraWidth: instrumentPanel.extraValuesWidth
            settingsGroup: factValueGrid.telemetryBarSettingsGroup
            specificVehicleForCard: null
            visible: true
        }
        
        // 固定宽度的空白间隔
        Item {
            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 20  // 固定间距
            Layout.preferredHeight: 1
        }
        
        // 自定义状态面板 - 右边
        Rectangle {
            id: statusPanel
            Layout.alignment: Qt.AlignBottom | Qt.AlignRight
            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 45  // 固定宽度，确保内容完全显示
            height: statusColumn.height + ScreenTools.defaultFontPixelHeight * 1
            color: Qt.rgba(qgcPal.window.r, qgcPal.window.g, qgcPal.window.b, 0.8)
            border.width: 0
            radius: ScreenTools.defaultFontPixelWidth * 0.5
            
            // 数据来源：复用现有UDP数据
            property bool isConnected: SimpleMavlinkUdp.connected
            property int missionStateIndex: SimpleMavlinkUdp.missionState
            property int locationStatusIndex: SimpleMavlinkUdp.locationStatus
            property int flightTime: SimpleMavlinkUdp.flightTime
            property int flownDistance: SimpleMavlinkUdp.flownDistance
            property int totalDistance: SimpleMavlinkUdp.totalDistance
            
            // 调试：数据变化监听
            onIsConnectedChanged: console.log("连接状态变化:", isConnected)
            onMissionStateIndexChanged: console.log("任务状态变化:", missionStateIndex)
            onFlightTimeChanged: console.log("飞行时间变化:", flightTime)
            onFlownDistanceChanged: console.log("已飞行距离变化:", flownDistance)
            onTotalDistanceChanged: console.log("总距离变化:", totalDistance)
            
            // 启动时检查初始值
            Component.onCompleted: {
                console.log("statusPanel初始化完成")
                
                // 启动UDP监听
                if (!SimpleMavlinkUdp.start(7777)) {
                    console.log("ERROR: SimpleMavlinkUdp启动失败")
                } else {
                    console.log("SimpleMavlinkUdp已启动，监听端口7777")
                }
                
                console.log("初始连接状态:", isConnected)
                console.log("初始任务状态:", missionStateIndex)
                console.log("初始飞行时间:", flightTime)
                console.log("初始已飞行距离:", flownDistance)
                console.log("初始总距离:", totalDistance)
            }
            
            // 状态映射
            property var missionStates: ["空闲", "就绪", "起飞", "巡航", "返航", "降落"]
            property var locationStates: ["无", "GPS", "ORB"]
            
            property string connectionStatus: isConnected ? "正常" : "断开"
            property string missionStatus: (missionStateIndex >= 0 && missionStateIndex < missionStates.length) ? 
                                          missionStates[missionStateIndex] : "未知"
            property string locationStatus: (locationStatusIndex >= 0 && locationStatusIndex < locationStates.length) ? 
                                           locationStates[locationStatusIndex] : "未知"

            QGCPalette { id: qgcPal }
            
            // 格式化飞行时间
            function formatTime(seconds) {
                var hours = Math.floor(seconds / 3600)
                var minutes = Math.floor((seconds % 3600) / 60)
                var secs = seconds % 60
                return hours.toString().padStart(2, '0') + ":" + 
                       minutes.toString().padStart(2, '0') + ":" + 
                       secs.toString().padStart(2, '0')
            }
            
            // 格式化距离
            function formatDistance(meters) {
                if (meters >= 1000) {
                    return (meters / 1000).toFixed(1) + "km"
                } else {
                    return meters + "m"
                }
            }
            
            Column {
                id:                 statusColumn
                anchors.centerIn:   parent
                spacing:            ScreenTools.defaultFontPixelHeight * 0.3
                
                // 第一行：任务机状态 + 模式 + 信号源
                Row {
                    spacing: ScreenTools.defaultFontPixelWidth * 0.8  // 减少间距以适应固定宽度
                    
                    // 任务机状态
                    Row {
                        spacing: ScreenTools.defaultFontPixelWidth * 0.3  // 减少间距
                        
                        QGCLabel {
                            text: "任务机："
                            font.pointSize: ScreenTools.defaultFontPointSize
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        
                        QGCLabel {
                            text: statusPanel.connectionStatus
                            font.pointSize: ScreenTools.defaultFontPointSize
                            font.bold: true
                            color: statusPanel.isConnected ? qgcPal.colorGreen : qgcPal.colorRed
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                    
                    // 分隔符
                    Rectangle {
                        width: 1
                        height: parent.height * 0.8
                        color: qgcPal.text
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    
                    // 模式状态
                    Row {
                        spacing: ScreenTools.defaultFontPixelWidth * 0.3  // 减少间距
                        
                        QGCLabel {
                            text: "模式："
                            font.pointSize: ScreenTools.defaultFontPointSize
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        
                        QGCLabel {
                            text: statusPanel.missionStatus
                            font.pointSize: ScreenTools.defaultFontPointSize
                            font.bold: true
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                    
                    // 分隔符
                    Rectangle {
                        width: 1
                        height: parent.height * 0.8
                        color: qgcPal.text
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    
                    // 信号源状态
                    Row {
                        spacing: ScreenTools.defaultFontPixelWidth * 0.3  // 减少间距
                        
                        QGCLabel {
                            text: "信号源："
                            font.pointSize: ScreenTools.defaultFontPointSize
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        
                        QGCLabel {
                            text: statusPanel.locationStatus
                            font.pointSize: ScreenTools.defaultFontPointSize
                            font.bold: true
                            color: statusPanel.locationStatus === "无" ? qgcPal.colorRed : qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
                
                // 第二行：巡检进度 + 飞行时间
                Row {
                    spacing: ScreenTools.defaultFontPixelWidth * 1.5  // 减少间距以适应固定宽度
                    
                    // 巡检进度
                    Row {
                        spacing: ScreenTools.defaultFontPixelWidth * 0.3  // 减少间距
                        
                        QGCLabel {
                            text: "巡检进度："
                            font.pointSize: ScreenTools.defaultFontPointSize
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        
                        QGCLabel {
                            text: statusPanel.formatDistance(statusPanel.flownDistance) + "/" + statusPanel.formatDistance(statusPanel.totalDistance)
                            font.pointSize: ScreenTools.defaultFontPointSize
                            font.bold: true
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                    
                    // 分隔符
                    Rectangle {
                        width: 1
                        height: parent.height * 0.8
                        color: qgcPal.text
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    
                    // 飞行时间
                    Row {
                        spacing: ScreenTools.defaultFontPixelWidth * 0.3  // 减少间距
                        
                        QGCLabel {
                            text: "飞行时间："
                            font.pointSize: ScreenTools.defaultFontPointSize
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        
                        QGCLabel {
                            text: statusPanel.formatTime(statusPanel.flightTime)
                            font.pointSize: ScreenTools.defaultFontPointSize
                            font.bold: true
                            font.family: "monospace"
                            color: qgcPal.text
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
            }
        }
    }
    
    // FlyViewInstrumentPanel - 恢复Layout定位
    FlyViewInstrumentPanel {
        id: instrumentPanel
        Layout.alignment: Qt.AlignBottom | Qt.AlignRight
        visible: QGroundControl.corePlugin.options.flyView.showInstrumentPanel && _showSingleVehicleUI
    }
} 