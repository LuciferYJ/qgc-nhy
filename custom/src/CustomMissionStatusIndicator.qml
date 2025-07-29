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
//-- Custom Mission Status Indicator
Item {
    id:             control
    width:          missionRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property bool showIndicator: true
    
    // 从SimpleMavlinkUdp获取真实数据
    property int missionStateIndex: SimpleMavlinkUdp.missionState
    property int flightTime: SimpleMavlinkUdp.flightTime
    property int remainingDistance: SimpleMavlinkUdp.remainingDistance
    property int capturedImages: SimpleMavlinkUdp.capturedImages
    
    // 状态映射：将数字索引转换为中文状态
    property var missionStates: ["空闲", "就绪", "起飞", "巡航", "返航", "降落"]
    property string missionStatus: (missionStateIndex >= 0 && missionStateIndex < missionStates.length) ? 
                                  missionStates[missionStateIndex] : "未知"

    QGCPalette { id: qgcPal }

    Row {
        id:             missionRow
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        spacing:        ScreenTools.defaultFontPixelWidth * 0.25

        Rectangle {
            id:                 missionIcon
            width:              ScreenTools.defaultFontPixelHeight * 0.8
            height:             width
            radius:             width / 2
            anchors.verticalCenter: parent.verticalCenter
            color:              qgcPal.text
            border.color:       qgcPal.text
            border.width:       1
        }

        QGCLabel {
            id:                 missionLabel
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            verticalAlignment:  Text.AlignVCenter
            text:               missionStatus
            font.pointSize:     ScreenTools.smallFontPointSize
            color:              qgcPal.text
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: mainWindow.showIndicatorDrawer(missionStatusIndicatorPage, control)
    }

    Component {
        id: missionStatusIndicatorPage

        CustomMissionStatusIndicatorPage {
            // 详情页直接使用SimpleMavlinkUdp数据，无需传递参数
        }
    }

    // 组件完成时初始化
    Component.onCompleted: {
        console.log("任务状态指示器已初始化，使用SimpleMavlinkUdp提供的真实数据")
    }
} 