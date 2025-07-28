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
    width: ScreenTools.defaultFontPixelWidth * 24
    height: ScreenTools.defaultFontPixelHeight * 4
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 1
    radius: ScreenTools.defaultFontPixelWidth * 0.5

    // 直接使用CustomStatusReceiver的连接状态
    property bool isConnected: CustomStatusReceiver.connected
    property string connectionStatus: isConnected ? "正常" : "断开"

    QGCPalette { id: qgcPal }

    function getConnectionStatusColor() {
        return isConnected ? qgcPal.text : qgcPal.colorRed
    }

    // 简化的连接状态显示
    RowLayout {
        anchors.fill: parent
        anchors.margins: ScreenTools.defaultFontPixelWidth * 0.8
        spacing: ScreenTools.defaultFontPixelWidth * 0.8

        // 连接状态指示灯
        Rectangle {
            Layout.preferredWidth: ScreenTools.defaultFontPixelHeight * 1.2
            Layout.preferredHeight: Layout.preferredWidth
            radius: Layout.preferredWidth / 2
            color: getConnectionStatusColor()
            border.color: qgcPal.text
            border.width: 1

            // 连接时的闪烁效果
            SequentialAnimation on opacity {
                running: isConnected
                loops: Animation.Infinite
                NumberAnimation { to: 0.3; duration: 800 }
                NumberAnimation { to: 1.0; duration: 800 }
            }
        }

        // 连接状态文字
        QGCLabel {
            text: "连接状态"
            font.pointSize: ScreenTools.smallFontPointSize
            color: qgcPal.text
            Layout.fillWidth: true
        }

        QGCLabel {
            text: connectionStatus
            font.pointSize: ScreenTools.smallFontPointSize
            font.bold: true
            color: getConnectionStatusColor()
        }
    }

    // 组件完成时初始化
    Component.onCompleted: {
        console.log("[INFO] 无人机连接详情页已初始化")
        // 状态管理由CustomStatusReceiver负责，这里只负责显示
    }
}
