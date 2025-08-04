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
import QGroundControl.ScreenTools
import QGroundControl.Palette
import Custom.UdpLink

//-------------------------------------------------------------------------
//-- Flight Display Status Panel (Right Bottom)
Rectangle {
    id:                 root
    width:              statusColumn.width + ScreenTools.defaultFontPixelWidth * 1.5
    height:             statusColumn.height + ScreenTools.defaultFontPixelHeight * 1
    color:              Qt.rgba(qgcPal.window.r, qgcPal.window.g, qgcPal.window.b, 0.8)
    border.color:       qgcPal.text
    border.width:       1
    radius:             ScreenTools.defaultFontPixelWidth * 0.5
    
    // 数据来源：复用现有UDP数据
    property bool isConnected: SimpleMavlinkUdp.connected
    property int missionStateIndex: SimpleMavlinkUdp.missionState
    property int locationStatusIndex: SimpleMavlinkUdp.locationStatus
    property int flightTime: SimpleMavlinkUdp.flightTime
    property int flownDistance: SimpleMavlinkUdp.flownDistance
    property int totalDistance: SimpleMavlinkUdp.totalDistance
    
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
            spacing: ScreenTools.defaultFontPixelWidth * 1
            
            // 任务机状态
            Row {
                spacing: ScreenTools.defaultFontPixelWidth * 0.5
                
                QGCLabel {
                    text: "任务机："
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
                
                QGCLabel {
                    text: connectionStatus
                    font.pointSize: ScreenTools.smallFontPointSize
                    font.bold: true
                    color: isConnected ? qgcPal.colorGreen : qgcPal.colorRed
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
                spacing: ScreenTools.defaultFontPixelWidth * 0.5
                
                QGCLabel {
                    text: "模式："
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
                
                QGCLabel {
                    text: missionStatus
                    font.pointSize: ScreenTools.smallFontPointSize
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
                spacing: ScreenTools.defaultFontPixelWidth * 0.5
                
                QGCLabel {
                    text: "信号源："
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
                
                QGCLabel {
                    text: locationStatus
                    font.pointSize: ScreenTools.smallFontPointSize
                    font.bold: true
                    color: locationStatus === "无" ? qgcPal.colorRed : qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }
        
        // 第二行：巡检进度 + 飞行时间
        Row {
            spacing: ScreenTools.defaultFontPixelWidth * 2
            
            // 巡检进度
            Row {
                spacing: ScreenTools.defaultFontPixelWidth * 0.5
                
                QGCLabel {
                    text: "巡检进度："
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
                
                QGCLabel {
                    text: formatDistance(flownDistance) + "/" + formatDistance(totalDistance)
                    font.pointSize: ScreenTools.smallFontPointSize
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
                spacing: ScreenTools.defaultFontPixelWidth * 0.5
                
                QGCLabel {
                    text: "飞行时间："
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
                
                QGCLabel {
                    text: formatTime(flightTime)
                    font.pointSize: ScreenTools.smallFontPointSize
                    font.bold: true
                    font.family: "monospace"
                    color: qgcPal.text
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }
    }
} 