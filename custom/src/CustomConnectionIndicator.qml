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
    property var  statusReceiver: CustomStatusReceiver

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
            color:              statusReceiver.connected ? qgcPal.colorGreen : qgcPal.colorRed
        }

        QGCLabel {
            id:                 statusLabel
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            verticalAlignment:  Text.AlignVCenter
            text:               statusReceiver.connected ? "正常" : "断线"
            font.pointSize:     ScreenTools.smallFontPointSize
            color:              statusReceiver.connected ? qgcPal.colorGreen : qgcPal.colorRed
        }
    }

    MouseArea {
        anchors.fill:   parent
        onClicked:      mainWindow.showIndicatorDrawer(connectionIndicatorPage, control)
    }

    Component {
        id: connectionIndicatorPage

        CustomConnectionIndicatorPage { }
    }
}
