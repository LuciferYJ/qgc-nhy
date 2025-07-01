import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0

Rectangle {
    id: root
    width: ScreenTools.defaultFontPixelWidth * 32
    height: contentColumn.height + ScreenTools.defaultFontPixelHeight * 1.4
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    radius: ScreenTools.defaultFontPixelWidth * 0.5

    property string missionStatus: "巡航"
    property int flightTime: 2456
    property int remainingDistance: 1200
    property int capturedImages: 78
    property real currentSpeed: 15.5  // m/s
    property real batteryLevel: 68.5  // %
    property real altitude: 120.8     // m

    QGCPalette { id: qgcPal }

    function getMissionStatusColor() {
        switch(missionStatus) {
            case "空闲": return qgcPal.colorGrey
            case "就绪": return qgcPal.colorBlue
            case "起飞": return qgcPal.colorRed
            case "巡航": return qgcPal.colorGreen
            case "返航": return qgcPal.colorGreen
            case "降落": return qgcPal.colorRed
            default: return qgcPal.colorGrey
        }
    }

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
            color: getMissionStatusColor()
            radius: ScreenTools.defaultFontPixelWidth * 0.3

            RowLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5

                Rectangle {
                    width: ScreenTools.defaultFontPixelHeight * 1.2
                    height: width
                    radius: width / 2
                    color: "white"
                    border.color: qgcPal.text
                    border.width: 2

                    Rectangle {
                        anchors.centerIn: parent
                        width: parent.width * 0.6
                        height: width
                        radius: width / 2
                        color: getMissionStatusColor()
                    }
                }

                QGCLabel {
                    text: "任务状态："
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: "white"
                    Layout.fillWidth: true
                }

                QGCLabel {
                    text: missionStatus
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: "white"
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

            // 当前速度
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
                        text: "当前速度"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: currentSpeed.toFixed(1) + " m/s"
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }
        }

        // 进度条区域
        Rectangle {
            width: parent.width
            height: ScreenTools.defaultFontPixelHeight * 2.5
            color: qgcPal.windowShade
            radius: ScreenTools.defaultFontPixelWidth * 0.3
            border.color: qgcPal.text
            border.width: 1

            Column {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.8
                spacing: ScreenTools.defaultFontPixelHeight * 0.3

                QGCLabel {
                    text: "电池电量"
                    font.pointSize: ScreenTools.smallFontPointSize
                    color: qgcPal.text
                }

                Rectangle {
                    width: parent.width
                    height: ScreenTools.defaultFontPixelHeight * 0.8
                    color: qgcPal.window
                    border.color: qgcPal.text
                    border.width: 1
                    radius: height / 2

                    Rectangle {
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        width: parent.width * (batteryLevel / 100)
                        color: batteryLevel > 30 ? qgcPal.colorGreen : 
                               batteryLevel > 15 ? qgcPal.colorOrange : qgcPal.colorRed
                        radius: parent.radius
                    }

                    QGCLabel {
                        text: batteryLevel.toFixed(1) + "%"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.centerIn: parent
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
            // 模拟实时数据更新
            flightTime += 1
            if (remainingDistance > 10) {
                remainingDistance -= Math.floor(Math.random() * 5 + 1)
            }
            
            currentSpeed = 15.5 + (Math.random() - 0.5) * 3
            batteryLevel = Math.max(0, batteryLevel - 0.01)
            altitude = 120.8 + (Math.random() - 0.5) * 10
            
            if (missionStatus === "巡航" && Math.random() > 0.95) {
                capturedImages += 1
            }
        }
    }
} 