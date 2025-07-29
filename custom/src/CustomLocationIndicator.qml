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
//-- Custom Location Status Indicator
Item {
    id:             control
    width:          locationRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property bool showIndicator: true
    
    // 从SimpleMavlinkUdp获取真实数据
    property int locationStatusIndex: SimpleMavlinkUdp.locationStatus
    property int visualSubStatusIndex: SimpleMavlinkUdp.visualSubStatus
    property int satelliteCount: SimpleMavlinkUdp.satelliteCount
    property real accuracy: SimpleMavlinkUdp.locationAccuracy
    
    // 状态映射：将数字索引转换为中文状态
    property var locationStates: ["无定位", "卫导定位", "视觉定位"]
    property var visualStates: ["未初始化", "单应", "ORB SLAM"]
    
    property string locationStatus: (locationStatusIndex >= 0 && locationStatusIndex < locationStates.length) ? 
                                   locationStates[locationStatusIndex] : "未知"
    property string visualStatus: (visualSubStatusIndex >= 0 && visualSubStatusIndex < visualStates.length) ? 
                                 visualStates[visualSubStatusIndex] : "未知"

    QGCPalette { id: qgcPal }

    function getLocationStatusColor() {
        switch(locationStatus) {
            case "无定位": return qgcPal.colorRed
            default: return qgcPal.colorGrey
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
            height:             parent.height * 0.7
            anchors.verticalCenter: parent.verticalCenter
            sourceSize.height:  height
            source:             "/res/locate.svg"
            fillMode:           Image.PreserveAspectFit
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
            // 详情页直接使用SimpleMavlinkUdp数据，无需传递参数
        }
    }

    // 组件完成时初始化
    Component.onCompleted: {
        console.log("定位状态指示器已初始化，使用SimpleMavlinkUdp提供的真实数据")
    }
} 