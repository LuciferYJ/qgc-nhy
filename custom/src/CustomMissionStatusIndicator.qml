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
//-- Custom Mission Status Indicator
Item {
    id:             control
    width:          missionRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property bool showIndicator: true
    property string missionStatus: "空闲"  // 空闲，就绪，起飞，巡航，返航，降落
    
    // 模拟数据，后续通过UDP接收真实数据
    property int flightTime: 1235  // 飞行时间（秒）
    property int remainingDistance: 2800  // 剩余航程（米）
    property int capturedImages: 45  // 采集图片数量

    QGCPalette { id: qgcPal }

    function getMissionStatusColor() {
        switch(missionStatus) {
            case "空闲": return qgcPal.colorGrey
            case "就绪": return qgcPal.colorBlue
            case "起飞": return qgcPal.colorOrange
            case "巡航": return qgcPal.colorGreen
            case "返航": return qgcPal.colorYellow
            case "降落": return qgcPal.colorRed
            default: return qgcPal.colorGrey
        }
    }

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
            color:              getMissionStatusColor()
            border.color:       qgcPal.text
            border.width:       1
        }

        QGCLabel {
            id:                 missionLabel
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            verticalAlignment:  Text.AlignVCenter
            text:               "任务: " + missionStatus
            font.pointSize:     ScreenTools.smallFontPointSize
            color:              getMissionStatusColor()
        }
    }

    MouseArea {
        anchors.fill: parent
        onClicked: mainWindow.showIndicatorDrawer(missionStatusIndicatorPage, control)
    }

    Component {
        id: missionStatusIndicatorPage

        CustomMissionStatusIndicatorPage {
            missionStatus: control.missionStatus
            flightTime: control.flightTime
            remainingDistance: control.remainingDistance
            capturedImages: control.capturedImages
        }
    }

    // 模拟数据更新定时器（用于演示）
    Timer {
        interval: 3000
        running: true
        repeat: true
        onTriggered: {
            // 模拟状态变化
            var statuses = ["空闲", "就绪", "起飞", "巡航", "返航", "降落"]
            var currentIndex = statuses.indexOf(missionStatus)
            var nextIndex = (currentIndex + 1) % statuses.length
            missionStatus = statuses[nextIndex]
            
            // 模拟数据变化
            flightTime += 30
            if (remainingDistance > 100) {
                remainingDistance -= Math.floor(Math.random() * 100 + 50)
            }
            if (missionStatus === "巡航" && Math.random() > 0.7) {
                capturedImages += Math.floor(Math.random() * 3 + 1)
            }
            
            // console.log("Mission status updated:", missionStatus, "Flight time:", flightTime, "Remaining:", remainingDistance, "Images:", capturedImages)
        }
    }
} 