/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

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
    id:         saveRouteDialog
    title:      qsTr("保存航线")
    buttons:    Dialog.Save | Dialog.Cancel
    
    property var visualItems: null
    property string routeName: ""
    
    Component.onCompleted: {
        // 生成默认航线名称
        routeName = "飞行任务" + new Date().getTime().toString().slice(-8)
    }
    
    function saveRoute() {
        if (routeNameField.text.trim() === "") {
            mainWindow.showMessageDialog(qsTr("错误"), qsTr("请输入航线名称"))
            return
        }
        
        var finalRouteName = routeNameField.text.trim()
        console.log("开始保存航线:", finalRouteName)
        
        // 调用MissionDatabase保存航线
        if (MissionDatabase && visualItems) {
            var success = MissionDatabase.addRoute(finalRouteName, visualItems)
            
            if (success) {
                var currentUuid = MissionDatabase.getCurrentRouteUuid()
                console.log("航线保存成功！名称:", finalRouteName, "UUID:", currentUuid)
                
                mainWindow.showMessageDialog(
                    qsTr("保存成功"),
                    qsTr("新航线已创建！\n\n") +
                    qsTr("航线名称: %1\n").arg(finalRouteName) +
                    qsTr("UUID: %1").arg(currentUuid)
                )
                
                accept() // 关闭对话框
            } else {
                console.log("航线保存失败！")
                mainWindow.showMessageDialog(
                    qsTr("保存失败"),
                    qsTr("航线保存失败！\n\n请检查数据库连接状态。")
                )
            }
        } else {
            console.error("MissionDatabase不可用或visualItems为空")
            mainWindow.showMessageDialog(qsTr("错误"), qsTr("数据库连接异常"))
        }
    }
    
    // 对话框内容
    ColumnLayout {
        spacing: ScreenTools.defaultFontPixelHeight
        
        QGCLabel {
            Layout.fillWidth:   true
            text:               qsTr("请输入航线名称：")
            font.pointSize:     ScreenTools.defaultFontPointSize
        }
        
        QGCTextField {
            id:                     routeNameField
            Layout.fillWidth:       true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 2
            text:                   routeName
            placeholderText:        qsTr("请输入航线名称")
            selectByMouse:          true
            
            // 回车键保存
            Keys.onReturnPressed: {
                saveRoute()
            }
            
            // 自动选中文本便于编辑
            Component.onCompleted: {
                selectAll()
                forceActiveFocus()
            }
        }
        
        QGCLabel {
            Layout.fillWidth:   true
            text:               qsTr("提示：航线将保存到本地数据库中")
            color:              qgcPal.colorGrey
            font.pointSize:     ScreenTools.smallFontPointSize
            wrapMode:           Text.WordWrap
        }
    }
    
    // 处理对话框按钮
    onAccepted: {
        saveRoute()
    }
    
    onRejected: {
        console.log("用户取消保存航线")
    }
} 