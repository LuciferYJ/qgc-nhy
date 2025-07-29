/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
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
//-- Custom Connection Status Indicator
Item {
    id:             control
    width:          connectionRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property bool showIndicator: true
    property bool isConnected: SimpleMavlinkUdp.connected  // 使用简化的UDP类

    QGCPalette { id: qgcPal }

    Row {
        id:             connectionRow
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        spacing:        ScreenTools.defaultFontPixelWidth * 0.25

        QGCColoredImage {
            id:                 connectionIcon
            width:              height
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            sourceSize.height:  height
            source:             "/res/connect.svg"
            fillMode:           Image.PreserveAspectFit
            color:              isConnected ? qgcPal.text : qgcPal.colorRed
        }
    }

    MouseArea {
        anchors.fill:   parent
        onClicked:      mainWindow.showIndicatorDrawer(connectionIndicatorPage, control)
    }

    Component {
        id: connectionIndicatorPage

        CustomConnectionIndicatorPage {
            // 页面直接使用SimpleMavlinkUdp，无需传递参数
        }
    }

    // 组件完成时初始化
    Component.onCompleted: {
        console.log("[INFO] 无人机连接指示器已初始化，使用SimpleMavlinkUdp管理连接状态")
        SimpleMavlinkUdp.start(7777)
        console.log("[INFO] 已启动UDP MAVLink接收器，监听端口7777")
    }
}
