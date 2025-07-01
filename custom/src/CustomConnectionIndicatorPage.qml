/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

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
    width: ScreenTools.defaultFontPixelWidth * 30
    height: contentColumn.height + ScreenTools.defaultFontPixelHeight * 1.4
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    radius: ScreenTools.defaultFontPixelWidth * 0.5

    property var statusReceiver: CustomStatusReceiver
    property string connectionStatus: statusReceiver.connected ? "已连接" : "未连接"
    property string localAddress: "127.0.0.1:7777"
    property string remoteAddress: "127.0.0.1:8888"

    QGCPalette { id: qgcPal }

    function getConnectionStatusColor() {
        return statusReceiver.connected ? qgcPal.colorGreen : qgcPal.colorRed
    }

    function formatDataSize(bytes) {
        if (bytes >= 1024 * 1024) {
            return (bytes / (1024 * 1024)).toFixed(2) + " MB"
        } else if (bytes >= 1024) {
            return (bytes / 1024).toFixed(1) + " KB"
        } else {
            return bytes + " B"
        }
    }

    function formatRate(bytesPerSec) {
        if (bytesPerSec >= 1024) {
            return (bytesPerSec / 1024).toFixed(1) + " KB/s"
        } else {
            return bytesPerSec.toFixed(0) + " B/s"
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
            color: getConnectionStatusColor()
            radius: ScreenTools.defaultFontPixelWidth * 0.3

            RowLayout {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5

                QGCColoredImage {
                    Layout.preferredWidth: ScreenTools.defaultFontPixelHeight * 1.2
                    Layout.preferredHeight: Layout.preferredWidth
                    source: "/res/connect.svg"
                    fillMode: Image.PreserveAspectFit
                    color: "white"
                }

                QGCLabel {
                    text: "连接状态："
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: "white"
                    Layout.fillWidth: true
                }

                QGCLabel {
                    text: connectionStatus
                    font.pointSize: ScreenTools.mediumFontPointSize
                    font.bold: true
                    color: "white"
                }
            }
        }

        // 主要信息区域
        Rectangle {
            width: parent.width
            height: ScreenTools.defaultFontPixelHeight * 3
            color: qgcPal.windowShade
            radius: ScreenTools.defaultFontPixelWidth * 0.3
            border.color: qgcPal.text
            border.width: 1

            Column {
                anchors.fill: parent
                anchors.margins: ScreenTools.defaultFontPixelWidth * 0.8
                spacing: ScreenTools.defaultFontPixelHeight * 0.3

                // 本地地址
                RowLayout {
                    width: parent.width
                    spacing: ScreenTools.defaultFontPixelWidth

                    QGCLabel {
                        text: "本地地址："
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                    }

                    QGCLabel {
                        text: localAddress
                        font.pointSize: ScreenTools.smallFontPointSize
                        font.bold: true
                        font.family: "monospace"
                        color: qgcPal.text
                    }
                }

                // 远程地址
                RowLayout {
                    width: parent.width
                    spacing: ScreenTools.defaultFontPixelWidth

                    QGCLabel {
                        text: "远程地址："
                        font.pointSize: ScreenTools.smallFontPointSize
                        color: qgcPal.text
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                    }

                    QGCLabel {
                        text: remoteAddress
                        font.pointSize: ScreenTools.smallFontPointSize
                        font.bold: true
                        font.family: "monospace"
                        color: qgcPal.text
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
            // 模拟统计数据更新
            if (statusReceiver.connected) {
                packetsReceived += Math.floor(Math.random() * 10 + 5)
                packetsSent += Math.floor(Math.random() * 3 + 1)
                dataRate = 50 + (Math.random() - 0.5) * 40
                if (dataRate < 0) dataRate = 0
            } else {
                dataRate = 0
            }
        }
    }
}
