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
    id:         missionListDialog
    title:      qsTr("任务列表")
    buttons:    Dialog.Close
    
    property var missionListModel: []
    property var qgcPal: QGroundControl.globalPalette
    
    Component.onCompleted: {
        console.log("MissionListDialog 创建完成")
        loadMissionList()
    }
    
    function loadMissionList() {
        console.log("开始加载任务数据...")
        if (typeof MissionDatabase !== 'undefined' && MissionDatabase) {
            console.log("MissionDatabase 可用，检查连接状态...")
            console.log("数据库连接状态:", MissionDatabase.isConnected())
            
            // 如果数据库未连接，尝试初始化
            if (!MissionDatabase.isConnected()) {
                console.log("数据库未连接，尝试初始化...")
                var initSuccess = MissionDatabase.initDatabase()
                console.log("数据库初始化结果:", initSuccess)
                
                if (initSuccess) {
                    console.log("数据库初始化成功")
                    // 不再自动创建测试数据
                }
            }
            
            try {
                var missions = MissionDatabase.getAllMissions()
                console.log("获取到", missions.length, "条任务数据")
                
                // 同时检查航线数据
                var routes = MissionDatabase.getAllRoutes()
                console.log("获取到", routes.length, "条航线数据")
                
                // 如果没有数据，不自动创建测试数据
                if (missions.length === 0 && routes.length === 0) {
                    console.log("没有找到任何数据")
                }
                
                var missionArray = []
                for (var i = 0; i < missions.length; i++) {
                    var mission = missions[i]
                    mission.index = i + 1
                    // result_uuid 字段已经直接包含在任务记录中
                    
                    // 获取关联的航线名称
                    if (mission.route_uuid) {
                        var route = MissionDatabase.getRoute(mission.route_uuid)
                        mission.route_name = route.name || qsTr("未知航线")
                    } else {
                        mission.route_name = qsTr("未关联航线")
                    }
                    
                    missionArray.push(mission)
                }
                
                missionListModel = missionArray
                console.log("设置模型数据，共", missionArray.length, "条记录")
            } catch (error) {
                console.log("加载数据时发生错误:", error)
                missionListModel = []
            }
        } else {
            console.log("MissionDatabase 不可用")
            missionListModel = []
        }
    }
    
    function formatTimestamp(timestamp) {
        if (!timestamp || timestamp === 0) return qsTr("未记录")
        
        var date = new Date(timestamp * 1000)
        var month = (date.getMonth() + 1).toString().padStart(2, '0')
        var day = date.getDate().toString().padStart(2, '0')
        var hours = date.getHours().toString().padStart(2, '0')
        var minutes = date.getMinutes().toString().padStart(2, '0')
        var seconds = date.getSeconds().toString().padStart(2, '0')
        return month + "-" + day + " " + hours + ":" + minutes + ":" + seconds
    }
    
    function formatUuid(uuid) {
        return uuid ? uuid.substring(0, 8) + "..." : qsTr("无")
    }
    
    // 使用QGC标准的布局方式
    ColumnLayout {
        width:      Math.min(mainWindow.width * 0.95, ScreenTools.defaultFontPixelWidth * 130)
        spacing:    ScreenTools.defaultFontPixelHeight * 0.5
        
        // 头部信息
        RowLayout {
            Layout.fillWidth:   true
            
            QGCLabel {
                text:               qsTr("共 %1 条任务记录").arg(missionListModel.length)
                font.pointSize:     ScreenTools.mediumFontPointSize
            }
            
            Item { Layout.fillWidth: true }
            
            QGCButton {
                text:       qsTr("刷新")
                onClicked:  loadMissionList()
            }
        }
        
        // 表格头部
        Rectangle {
            Layout.fillWidth:   true
            height:             ScreenTools.defaultFontPixelHeight * 2
            color:              qgcPal.windowShade
            border.color:       qgcPal.text
            border.width:       1
            
            RowLayout {
                anchors.fill:       parent
                anchors.margins:    ScreenTools.defaultFontPixelWidth * 0.5
                spacing:            ScreenTools.defaultFontPixelWidth
                
                QGCLabel {
                    Layout.preferredWidth:  40
                    text:                   qsTr("序号")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  120
                    text:                   qsTr("开始时间")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  120
                    text:                   qsTr("完成时间")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  100
                    text:                   qsTr("成果UUID")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  130
                    text:                   qsTr("关联日志名称")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignLeft
                }
                
                QGCLabel {
                    Layout.preferredWidth:  120
                    text:                   qsTr("关联航线名称")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignLeft
                }
                
                QGCLabel {
                    Layout.preferredWidth:  80
                    text:                   qsTr("操作")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
            }
        }
        
        // 使用QGCListView显示数据
        QGCListView {
            id:                     missionListView
            Layout.fillWidth:       true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 20
            model:                  missionListModel
            
            delegate: Rectangle {
                width:          missionListView.width
                height:         ScreenTools.defaultFontPixelHeight * 2.5
                color:          index % 2 === 0 ? qgcPal.window : qgcPal.windowShade
                border.color:   qgcPal.text
                border.width:   0.5
                
                RowLayout {
                    anchors.fill:       parent
                    anchors.margins:    ScreenTools.defaultFontPixelWidth * 0.5
                    spacing:            ScreenTools.defaultFontPixelWidth
                    
                    // 序号
                    QGCLabel {
                        Layout.preferredWidth:  40
                        text:                   modelData.index || (index + 1)
                        horizontalAlignment:    Text.AlignCenter
                    }
                    
                    // 开始时间
                    QGCLabel {
                        Layout.preferredWidth:  120
                        text:                   formatTimestamp(modelData.start_time || 0)
                        horizontalAlignment:    Text.AlignCenter
                        elide:                  Text.ElideRight
                        
                        // 鼠标悬停显示完整时间
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && modelData.start_time
                                text: {
                                    var date = new Date((modelData.start_time || 0) * 1000)
                                    return date.toLocaleString(Qt.locale(), "yyyy-MM-dd hh:mm:ss")
                                }
                                delay: 500
                            }
                        }
                    }
                    
                    // 完成时间
                    QGCLabel {
                        Layout.preferredWidth:  120
                        text:                   formatTimestamp(modelData.end_time || 0)
                        horizontalAlignment:    Text.AlignCenter
                        elide:                  Text.ElideRight
                        
                        // 鼠标悬停显示完整时间
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && modelData.end_time
                                text: {
                                    var date = new Date((modelData.end_time || 0) * 1000)
                                    return date.toLocaleString(Qt.locale(), "yyyy-MM-dd hh:mm:ss")
                                }
                                delay: 500
                            }
                        }
                    }
                    
                    // 成果UUID
                    QGCLabel {
                        Layout.preferredWidth:  100
                        text:                   formatUuid(modelData.result_uuid)
                        horizontalAlignment:    Text.AlignCenter
                        color:                  modelData.result_uuid ? qgcPal.text : qgcPal.colorGrey
                        
                        // 鼠标悬停显示完整UUID
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && modelData.result_uuid
                                text: modelData.result_uuid || qsTr("无成果数据")
                                delay: 500
                            }
                        }
                    }
                    
                    // 关联日志名称
                    QGCLabel {
                        Layout.preferredWidth:  130
                        text:                   modelData.log_file_name || qsTr("未指定")
                        elide:                  Text.ElideRight
                        horizontalAlignment:    Text.AlignLeft
                        
                        // 鼠标悬停显示完整日志名称
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && (modelData.log_file_name || qsTr("未指定")).length > 0
                                text: modelData.log_file_name || qsTr("未指定日志文件")
                                delay: 500
                            }
                        }
                    }
                    
                    // 关联航线名称
                    QGCLabel {
                        Layout.preferredWidth:  120
                        text:                   modelData.route_name || qsTr("未知航线")
                        elide:                  Text.ElideRight
                        horizontalAlignment:    Text.AlignLeft
                        color:                  modelData.route_name ? qgcPal.text : qgcPal.colorGrey
                        
                        // 鼠标悬停显示完整航线名称
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && modelData.route_name
                                text: modelData.route_name || qsTr("未知航线名称")
                                delay: 500
                            }
                        }
                    }
                    
                    // 操作按钮
                    QGCButton {
                        Layout.preferredWidth:  80
                        text:                   qsTr("成果展示")
                        onClicked: {
                            console.log("成功展示任务:", modelData.uuid)
                            console.log("任务详情:")
                            console.log("- 任务UUID:", modelData.uuid)
                            console.log("- 关联航线:", modelData.route_name)
                            console.log("- 开始时间:", formatTimestamp(modelData.start_time))
                            console.log("- 完成时间:", formatTimestamp(modelData.end_time))
                            console.log("- 日志文件:", modelData.log_file_name)
                            console.log("- 成果UUID:", modelData.result_uuid)
                            mainWindow.showMessageDialog(qsTr("任务展示"), 
                                qsTr("任务已成功展示！\n\n") +
                                qsTr("关联航线: %1\n").arg(modelData.route_name || qsTr("未知航线")) +
                                qsTr("开始时间: %1\n").arg(formatTimestamp(modelData.start_time)) +
                                qsTr("完成时间: %1\n").arg(formatTimestamp(modelData.end_time)) +
                                qsTr("日志文件: %1\n").arg(modelData.log_file_name || qsTr("未指定")) +
                                qsTr("成果数据: %1").arg(modelData.result_uuid ? qsTr("有") : qsTr("无")))
                        }
                    }
                }
            }
        }
        
        // 底部状态
        QGCLabel {
            Layout.fillWidth:       true
            text:                   missionListModel.length === 0 ? 
                                      qsTr("暂无任务数据") : 
                                      qsTr("提示: 使用鼠标滚轮或拖拽滚动条查看更多数据")
            color:                  qgcPal.colorGrey
            horizontalAlignment:    Text.AlignCenter
            wrapMode:               Text.WordWrap
        }
    }
} 