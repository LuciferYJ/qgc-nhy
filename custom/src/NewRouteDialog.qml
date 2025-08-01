import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

import QGroundControl
import QGroundControl.Controls
import QGroundControl.ScreenTools
import QGroundControl.Palette
import Custom.Database 1.0

QGCPopupDialog {
    id: newRouteDialog
    
    property alias routeName: routeNameField.text
    property var visualItems: null
    
    function saveRoute() {
        if (routeNameField.text.trim() === "") {
            mainWindow.showMessageDialog(qsTr("错误"), qsTr("请输入航线名称"))
            return
        }
        
        if (visualItems && MissionDatabase) {
            var success = MissionDatabase.addRoute(routeNameField.text.trim(), visualItems)
            if (success) {
                console.log("新航线保存成功:", routeNameField.text.trim())
                accept()
            } else {
                mainWindow.showMessageDialog(qsTr("错误"), qsTr("保存航线失败"))
            }
        }
    }
    
    function reset() {
        routeNameField.text = ""
        visualItems = null
    }
    
    onAccepted: {
        saveRoute()
    }
    
    onRejected: {
        reset()
    }
    
    title: qsTr("保存新航线")
    buttons: Dialog.Save | Dialog.Cancel
    modal: true
    
    ColumnLayout {
        spacing: ScreenTools.defaultFontPixelHeight
        
        QGCLabel {
            Layout.fillWidth: true
            text: qsTr("检测到新的航线，请输入航线名称：")
            wrapMode: Text.WordWrap
        }
        
        QGCTextField {
            id: routeNameField
            Layout.fillWidth: true
            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 30
            placeholderText: qsTr("输入航线名称...")
            
            Keys.onReturnPressed: {
                if (text.trim() !== "") {
                    newRouteDialog.saveRoute()
                }
            }
            
            Component.onCompleted: {
                forceActiveFocus()
            }
        }
        
        QGCLabel {
            Layout.fillWidth: true
            text: qsTr("提示：航线名称将用于标识此航线")
            font.pointSize: ScreenTools.smallFontPointSize
            color: qgcPal.colorGrey
            wrapMode: Text.WordWrap
        }
    }
} 