/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.Palette
import QGroundControl.ScreenTools

Rectangle {
    id: root
    
    property real latitude: 0.0
    property real longitude: 0.0
    property real altitude: 0.0
    
    signal accepted(real lat, real lon, real alt)
    signal rejected()
    
    width: ScreenTools.defaultFontPixelWidth * 20
    height: ScreenTools.defaultFontPixelHeight * 13
    color: qgcPal.window
    border.color: qgcPal.text
    border.width: 2
    radius: ScreenTools.defaultFontPixelWidth * 0.5
    
    // 确保对话框可见
    visible: true
    
    property var qgcPal: QGCPalette { colorGroupEnabled: enabled }
    
    // 响应式位置设置 - 与左侧菜单栏保持同一高度
    anchors.horizontalCenter: parent ? parent.horizontalCenter : undefined
    y: parent ? 80 : 80  
    
    // 地图坐标接收接口
    function setCoordinateFromMap(coordinate) {
        if (coordinate && coordinate.isValid) {
            latitudeField.text = coordinate.latitude.toFixed(6)
            longitudeField.text = coordinate.longitude.toFixed(6)
        }
    }
    
    // 简化的标题栏
    Rectangle {
        id: titleBar
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: ScreenTools.defaultFontPixelHeight * 2
        color: qgcPal.windowShade
        radius: parent.radius
        
        QGCLabel {
            anchors.centerIn: parent
            text: qsTr("设置Home位置")
            font.pointSize: ScreenTools.defaultFontPointSize
        }
    }
    
    // 简化的内容区域
    ColumnLayout {
        anchors.top: titleBar.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: buttonRow.top
        anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5
        spacing: ScreenTools.defaultFontPixelHeight * 0.3
        
        // 纬度输入
        RowLayout {
            Layout.fillWidth: true
            QGCLabel {
                text: qsTr("纬度:")
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 4
            }
            QGCTextField {
                id: latitudeField
                Layout.fillWidth: true
                text: root.latitude.toFixed(6)
                placeholderText: qsTr("39.904200")
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                validator: DoubleValidator {
                    bottom: -90.0
                    top: 90.0
                    decimals: 6
                }
            }
        }
        
        // 经度输入
        RowLayout {
            Layout.fillWidth: true
            QGCLabel {
                text: qsTr("经度:")
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 4
            }
            QGCTextField {
                id: longitudeField
                Layout.fillWidth: true
                text: root.longitude.toFixed(6)
                placeholderText: qsTr("116.407400")
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                validator: DoubleValidator {
                    bottom: -180.0
                    top: 180.0
                    decimals: 6
                }
            }
        }
        
        // 海拔输入
        RowLayout {
            Layout.fillWidth: true
            QGCLabel {
                text: qsTr("海拔:")
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 4
            }
            QGCTextField {
                id: altitudeField
                Layout.fillWidth: true
                text: root.altitude.toFixed(1)
                placeholderText: qsTr("50.0")
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                validator: DoubleValidator {
                    bottom: -1000.0
                    top: 10000.0
                    decimals: 1
                }
            }
        }
        
        // 地图选择提示
        QGCLabel {
            Layout.fillWidth: true
            text: qsTr("💡 直接点击地图上的任意位置即可获取坐标")
            font.pointSize: ScreenTools.smallFontPointSize
            color: qgcPal.colorOrange
            wrapMode: Text.WordWrap
            horizontalAlignment: Text.AlignHCenter
        }
    }
    
    // 简化的按钮行
    RowLayout {
        id: buttonRow
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: ScreenTools.defaultFontPixelWidth * 0.5
        spacing: ScreenTools.defaultFontPixelWidth * 0.5
        
        QGCButton {
            Layout.fillWidth: true
            text: qsTr("取消")
            onClicked: root.rejected()
        }
        
        QGCButton {
            Layout.fillWidth: true
            text: qsTr("确定")
            primary: true
            enabled: latitudeField.acceptableInput && 
                    longitudeField.acceptableInput && 
                    altitudeField.acceptableInput
            onClicked: {
                var lat = parseFloat(latitudeField.text)
                var lon = parseFloat(longitudeField.text)
                var alt = parseFloat(altitudeField.text)
                root.accepted(lat, lon, alt)
            }
        }
    }
} 