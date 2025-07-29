import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0
import Custom.UdpLink

Rectangle {
    id: root
    width: ScreenTools.defaultFontPixelWidth * 32
    height: contentColumn.height + ScreenTools.defaultFontPixelHeight * 1.4
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    radius: ScreenTools.defaultFontPixelWidth * 0.5

    // 从SimpleMavlinkUdp获取真实数据
    property int missionStateIndex: SimpleMavlinkUdp.missionState
    property int flightTime: SimpleMavlinkUdp.flightTime
    property int remainingDistance: SimpleMavlinkUdp.remainingDistance
    property int capturedImages: SimpleMavlinkUdp.capturedImages
    
    // 状态映射：将数字索引转换为中文状态
    property var missionStates: ["空闲", "就绪", "起飞", "巡航", "返航", "降落"]
    property string missionStatus: (missionStateIndex >= 0 && missionStateIndex < missionStates.length) ? 
                                  missionStates[missionStateIndex] : "未知"
    
    // 其他状态数据（可以后续扩展）
    property real altitude: 120.8     // m - 可以后续通过UDP传输

    QGCPalette { id: qgcPal }

    function formatTime(seconds) {
        var hours = Math.floor(seconds / 3600)
        var minutes = Math.floor((seconds % 3600) / 60)
        var secs = seconds % 60
        return hours.toString().padStart(2, '0') + ":" + 
               minutes.toString().padStart(2, '0') + ":" + 
               secs.toString().padStart(2, '0')
    }

    function formatDistance(meters) {
        if (meters >= 1000) {
            return (meters / 1000).toFixed(1) + " km"
        } else {
            return meters + " m"
        }
    }

    Column {
        id: contentColumn
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: ScreenTools.defaultFontPixelHeight * 0.8
        spacing: ScreenTools.defaultFontPixelHeight * 0.6

        // 标题栏
        Rectangle {
            width: parent.width
            height: ScreenTools.defaultFontPixelHeight * 2.5
            color: qgcPal.windowShade
            radius: ScreenTools.defaultFontPixelWidth * 0.3
            border.color: qgcPal.text
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5

                Rectangle {
                    width: ScreenTools.defaultFontPixelHeight * 1.2
                    height: width
                    radius: width / 2
                    color: qgcPal.window
                    border.color: qgcPal.text
                    border.width: 2

                    Rectangle {
                        anchors.centerIn: parent
                        width: parent.width * 0.6
                        height: width
                        radius: width / 2
                        color: qgcPal.text
                    }
                }

                QGCLabel {
                    text: "任务状态："
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: qgcPal.text
                    Layout.fillWidth: true
                }

                QGCLabel {
                    text: missionStatus
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: qgcPal.text
                }
            }
        }

        // 主要信息网格
        Grid {
            width: parent.width
            columns: 2
            columnSpacing: ScreenTools.defaultFontPixelWidth * 3
            rowSpacing: ScreenTools.defaultFontPixelHeight * 0.8

            // 飞行时间
            Rectangle {
                width: (parent.width - parent.columnSpacing) / 2
                height: ScreenTools.defaultFontPixelHeight * 3
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.3
                border.color: qgcPal.text
                border.width: 1

                Column {
                    anchors.centerIn: parent
                    spacing: ScreenTools.defaultFontPixelHeight * 0.2

                    QGCLabel {
                        text: "飞行时间"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: formatTime(flightTime)
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        font.family: "monospace"
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }

            // 剩余航程
            Rectangle {
                width: (parent.width - parent.columnSpacing) / 2
                height: ScreenTools.defaultFontPixelHeight * 3
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.3
                border.color: qgcPal.text
                border.width: 1

                Column {
                    anchors.centerIn: parent
                    spacing: ScreenTools.defaultFontPixelHeight * 0.2

                    QGCLabel {
                        text: "剩余航程"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: formatDistance(remainingDistance)
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        color: remainingDistance < 500 ? qgcPal.colorOrange : qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }

            // 采集图片
            Rectangle {
                width: (parent.width - parent.columnSpacing) / 2
                height: ScreenTools.defaultFontPixelHeight * 3
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.3
                border.color: qgcPal.text
                border.width: 1

                Column {
                    anchors.centerIn: parent
                    spacing: ScreenTools.defaultFontPixelHeight * 0.2

                    QGCLabel {
                        text: "采集图片"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: capturedImages + " 张"
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }


        }


    }

    // 模拟数据更新
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            // 模拟实时数据更新（实际使用时这些会通过UDP接收）
            altitude = 120.8 + (Math.random() - 0.5) * 10
        }
    }
} 