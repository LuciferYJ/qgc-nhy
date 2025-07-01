import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QGroundControl 1.0
import QGroundControl.Controls 1.0
import QGroundControl.ScreenTools 1.0
import QGroundControl.Palette 1.0

Rectangle {
    id: root
    width: ScreenTools.defaultFontPixelWidth * 28
    height: contentColumn.height + ScreenTools.defaultFontPixelHeight * 1.4
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    radius: ScreenTools.defaultFontPixelWidth * 0.5

    property string locationStatus: "无定位"
    property string visualStatus: "未初始化"
    property int satelliteCount: 0
    property real accuracy: 0.0
    property real latitude: 0.0
    property real longitude: 0.0

    QGCPalette { id: qgcPal }

    function getLocationStatusColor() {
        switch(locationStatus) {
            case "无定位": return qgcPal.colorRed
            case "卫导定位": return qgcPal.colorGreen
            case "视觉定位": return qgcPal.colorGreen
            default: return qgcPal.colorGrey
        }
    }

    Column {
        id: contentColumn
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: ScreenTools.defaultFontPixelHeight * 0.8
        spacing: ScreenTools.defaultFontPixelHeight * 0.6


        // 主要信息区域
        Rectangle {
            width: parent.width
            height: ScreenTools.defaultFontPixelHeight * 1.8
            color: qgcPal.windowShade
            radius: ScreenTools.defaultFontPixelWidth * 0.3
            border.color: qgcPal.text
            border.width: 1

            Column {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.8
                spacing: ScreenTools.defaultFontPixelHeight * 0.1

                // 定位状态行
                RowLayout {
                    width: parent.width
                    spacing: ScreenTools.defaultFontPixelWidth

                    QGCLabel {
                        text: "定位状态："
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                    }

                    Rectangle {
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.2
                        radius: height / 2
                        color: getLocationStatusColor()

                        QGCLabel {
                            anchors.centerIn: parent
                            text: locationStatus
                            font.pointSize: ScreenTools.smallFontPointSize
                            font.bold: true
                            color: qgcPal.text
                        }
                    }
                }
            }
        }

        // 详细信息网格
        Grid {
            width: parent.width
            columns: 2
            columnSpacing: ScreenTools.defaultFontPixelWidth * 2
            rowSpacing: ScreenTools.defaultFontPixelHeight * 0.8
            visible: locationStatus !== "无定位"

            // 卫星数量（仅卫导航定位时显示）
            Rectangle {
                width: (parent.width - parent.columnSpacing) / 2
                height: ScreenTools.defaultFontPixelHeight * 3
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.3
                border.color: qgcPal.text
                border.width: 1
                visible: locationStatus === "卫导定位"

                Column {
                    anchors.centerIn: parent
                    spacing: ScreenTools.defaultFontPixelHeight * 0.2

                    QGCLabel {
                        text: "卫星数量"
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: satelliteCount + " 颗"
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        color: satelliteCount >= 8 ? qgcPal.colorGreen : 
                               satelliteCount >= 5 ? qgcPal.colorYellow : qgcPal.colorRed
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }

            // 视觉定位额外状态（仅在视觉定位时显示）
            Rectangle {
                width: (parent.width - parent.columnSpacing) / 2
                height: ScreenTools.defaultFontPixelHeight * 3
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.3
                border.color: qgcPal.text
                border.width: 1
                visible: locationStatus === "视觉定位"

                Column {
                    anchors.centerIn: parent
                    spacing: ScreenTools.defaultFontPixelHeight * 0.2

                    QGCLabel {
                        text: "视觉状态："
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: visualStatus
                        font.pointSize: ScreenTools.defaultFontPointSize
                        font.bold: true
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }
            }

            // 定位精度
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
                        text: "定位精度："
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    QGCLabel {
                        text: accuracy > 0 ? accuracy.toFixed(2) + " m" : "---"
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
            // 模拟实时数据更新
            if (locationStatus === "卫导定位") {
                accuracy = accuracy + (Math.random() - 0.5) * 0.2
                if (accuracy < 0.5) accuracy = 0.5
                if (accuracy > 5.0) accuracy = 5.0
            } else if (locationStatus === "视觉定位") {
                accuracy = accuracy + (Math.random() - 0.5) * 0.05
                if (accuracy < 0.1) accuracy = 0.1
                if (accuracy > 1.0) accuracy = 1.0
            }
        }
    }
} 