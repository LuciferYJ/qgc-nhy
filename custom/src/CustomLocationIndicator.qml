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

//-------------------------------------------------------------------------
//-- Custom Location Status Indicator
Item {
    id:             control
    width:          locationRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property bool showIndicator: true
    property string locationStatus: "无定位"  // 无定位，卫导航定位，视觉定位
    property string visualStatus: "未初始化"  // 未初始化，单应，ORB SLAM
    
    // 模拟数据，后续通过UDP接收真实数据
    property int satelliteCount: 0
    property real accuracy: 0.0  // 定位精度（米）
    property real latitude: 0.0
    property real longitude: 0.0

    QGCPalette { id: qgcPal }

    function getLocationStatusColor() {
        switch(locationStatus) {
            case "无定位": return qgcPal.colorRed
            case "卫导定位": return qgcPal.colorGreen
            case "视觉定位": return qgcPal.colorBlue
            default: return qgcPal.colorGrey
        }
    }

    function getLocationStatusText() {
        switch(locationStatus) {
            case "无定位": return "无定位"
            case "卫导定位": return "卫导"
            case "视觉定位": return "视觉"
            default: return "未知"
        }
    }

    Row {
        id:             locationRow
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        spacing:        ScreenTools.defaultFontPixelWidth * 0.25

        QGCColoredImage {
            id:                 locationIcon
            width:              height
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            sourceSize.height:  height
            source:             "/res/locate.svg"
            fillMode:           Image.PreserveAspectFit
            color:              getLocationStatusColor()
        }

        QGCLabel {
            id:                 statusLabel
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            verticalAlignment:  Text.AlignVCenter
            text:               getLocationStatusText()
            font.pointSize:     ScreenTools.smallFontPointSize
            color:              getLocationStatusColor()
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: mainWindow.showIndicatorDrawer(locationIndicatorPage, control)
    }

    Component {
        id: locationIndicatorPage

        CustomLocationIndicatorPage {
            locationStatus: control.locationStatus
            visualStatus: control.visualStatus
            satelliteCount: control.satelliteCount
            accuracy: control.accuracy
            latitude: control.latitude
            longitude: control.longitude
        }
    }

    // 模拟数据更新定时器（用于演示）
    Timer {
        interval: 4000
        running: true
        repeat: true
        onTriggered: {
            // 模拟状态变化
            var statuses = ["无定位", "卫导定位", "视觉定位"]
            var currentIndex = statuses.indexOf(locationStatus)
            var nextIndex = (currentIndex + 1) % statuses.length
            locationStatus = statuses[nextIndex]
            
            // 根据定位状态模拟相应数据
            if (locationStatus === "卫导定位") {
                satelliteCount = Math.floor(Math.random() * 8 + 8)  // 8-15颗卫星
                accuracy = Math.random() * 2 + 1  // 1-3米精度
                latitude = 39.9042 + (Math.random() - 0.5) * 0.01
                longitude = 116.4074 + (Math.random() - 0.5) * 0.01
            } else if (locationStatus === "视觉定位") {
                var visualStatuses = ["未初始化", "单应", "ORB SLAM"]
                visualStatus = visualStatuses[Math.floor(Math.random() * visualStatuses.length)]
                satelliteCount = 0
                accuracy = Math.random() * 0.5 + 0.1  // 0.1-0.6米精度
                latitude = 39.9042 + (Math.random() - 0.5) * 0.001
                longitude = 116.4074 + (Math.random() - 0.5) * 0.001
            } else {
                satelliteCount = 0
                accuracy = 0.0
                latitude = 0.0
                longitude = 0.0
            }
            
            console.log("Location status updated:", locationStatus, "Visual:", visualStatus, "Satellites:", satelliteCount, "Accuracy:", accuracy.toFixed(2))
        }
    }
} 