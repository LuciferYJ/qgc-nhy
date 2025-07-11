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
    id:         routeListDialog
    title:      qsTr("航迹列表")
    buttons:    Dialog.Close
    
    property var routeListModel: []
    property var qgcPal: QGroundControl.globalPalette
    
    Component.onCompleted: {
        console.log("RouteListDialog 创建完成")
        loadRouteList()
    }
    
    function loadRouteList() {
        console.log("开始加载航线数据...")
        if (typeof MissionDatabase !== 'undefined' && MissionDatabase) {
            try {
                var routes = MissionDatabase.getAllRoutes()
                console.log("获取到", routes.length, "条航线数据")
                
                var routeArray = []
                for (var i = 0; i < routes.length; i++) {
                    var route = routes[i]
                    route.index = i + 1
                    routeArray.push(route)
                }
                
                routeListModel = routeArray
                console.log("设置模型数据，共", routeArray.length, "条记录")
            } catch (error) {
                console.log("加载数据时发生错误:", error)
                routeListModel = []
            }
        } else {
            console.log("MissionDatabase 不可用")
            routeListModel = []
        }
    }
    
    function formatTimestamp(timestamp) {
        var date = new Date(timestamp * 1000)
        var year = date.getFullYear().toString().slice(-2)  // 取年份后两位
        var month = (date.getMonth() + 1).toString().padStart(2, '0')
        var day = date.getDate().toString().padStart(2, '0')
        var hours = date.getHours().toString().padStart(2, '0')
        var minutes = date.getMinutes().toString().padStart(2, '0')
        var seconds = date.getSeconds().toString().padStart(2, '0')
        return year + "-" + month + "-" + day + " " + hours + ":" + minutes + ":" + seconds
    }
    
    function formatDuration(seconds) {
        var hours = Math.floor(seconds / 3600)
        var minutes = Math.floor((seconds % 3600) / 60)
        var secs = seconds % 60
        
        if (hours > 0) {
            return hours + "h " + minutes + "m " + secs + "s"
        } else if (minutes > 0) {
            return minutes + "m " + secs + "s"
        } else {
            return secs + "s"
        }
    }
    
    function formatDistance(meters) {
        if (meters >= 1000) {
            return (meters / 1000).toFixed(2) + " km"
        } else {
            return meters.toFixed(0) + " m"
        }
    }
    
    // 使用QGC标准的布局方式
    ColumnLayout {
        width:      Math.min(mainWindow.width * 0.9, ScreenTools.defaultFontPixelWidth * 100)
        spacing:    ScreenTools.defaultFontPixelHeight * 0.5
        
        // 头部信息
        RowLayout {
            Layout.fillWidth:   true
            
            QGCLabel {
                text:               qsTr("共 %1 条航线记录").arg(routeListModel.length)
                font.pointSize:     ScreenTools.mediumFontPointSize
            }
            
            Item { Layout.fillWidth: true }
            
            QGCButton {
                text:       qsTr("刷新")
                onClicked:  loadRouteList()
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
                    text:                   qsTr("航线名称")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignLeft
                }
                
                QGCLabel {
                    Layout.preferredWidth:  140
                    text:                   qsTr("创建时间")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  60
                    text:                   qsTr("航点数")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  80
                    text:                   qsTr("航线长度")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  80
                    text:                   qsTr("预计时长")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
                
                QGCLabel {
                    Layout.preferredWidth:  120
                    text:                   qsTr("操作")
                    font.bold:              true
                    horizontalAlignment:    Text.AlignCenter
                }
            }
        }
        
        // 使用QGCListView显示数据
        QGCListView {
            id:                     routeListView
            Layout.fillWidth:       true
            Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 20
            model:                  routeListModel
            
            delegate: Rectangle {
                width:          routeListView.width
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
                    
                    // 航线名称
                    QGCLabel {
                        Layout.preferredWidth:  120
                        text:                   modelData.name || qsTr("未命名航线")
                        elide:                  Text.ElideRight
                        horizontalAlignment:    Text.AlignLeft
                        
                        // 鼠标悬停显示完整内容
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse && (modelData.name || qsTr("未命名航线")).length > 0
                                text: modelData.name || qsTr("未命名航线")
                                delay: 500
                            }
                        }
                    }
                    
                    // 创建时间
                    QGCLabel {
                        Layout.preferredWidth:  140
                        text:                   formatTimestamp(modelData.modify_time || 0)
                        horizontalAlignment:    Text.AlignCenter
                        elide:                  Text.ElideRight
                        
                        // 鼠标悬停显示完整时间
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            ToolTip {
                                visible: parent.containsMouse
                                text: {
                                    var date = new Date((modelData.modify_time || 0) * 1000)
                                    return date.toLocaleString(Qt.locale(), "yyyy-MM-dd hh:mm:ss")
                                }
                                delay: 500
                            }
                        }
                    }
                    
                    // 航点数
                    QGCLabel {
                        Layout.preferredWidth:  60
                        text:                   modelData.waypoint_count || "0"
                        horizontalAlignment:    Text.AlignCenter
                    }
                    
                    // 航线长度
                    QGCLabel {
                        Layout.preferredWidth:  80
                        text:                   formatDistance(modelData.route_length || 0)
                        horizontalAlignment:    Text.AlignCenter
                    }
                    
                    // 预计时长
                    QGCLabel {
                        Layout.preferredWidth:  80
                        text:                   formatDuration(modelData.estimated_duration || 0)
                        horizontalAlignment:    Text.AlignCenter
                    }
                    
                    // 操作按钮
                    RowLayout {
                        Layout.preferredWidth:  120
                        spacing:                ScreenTools.defaultFontPixelWidth * 0.5
                        
                        QGCButton {
                            Layout.fillWidth:   true
                            text:               qsTr("编辑")
                            onClicked: {
                                console.log("编辑航线:", modelData.name)
                                console.log("UUID:", modelData.uuid)
                                // TODO: 实现编辑功能
                            }
                        }
                        
                        QGCButton {
                            Layout.fillWidth:   true
                            text:               qsTr("删除")
                            onClicked: {
                                console.log("删除航线:", modelData.name)
                                if (typeof MissionDatabase !== 'undefined' && MissionDatabase) {
                                    if (MissionDatabase.deleteRoute(modelData.uuid)) {
                                        console.log("删除成功")
                                        loadRouteList()
                                    } else {
                                        console.log("删除失败")
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // 底部状态
        QGCLabel {
            Layout.fillWidth:       true
            text:                   routeListModel.length === 0 ? 
                                      qsTr("暂无航线数据") : 
                                      qsTr("提示: 使用鼠标滚轮或拖拽滚动条查看更多数据")
            color:                  qgcPal.colorGrey
            horizontalAlignment:    Text.AlignCenter
            wrapMode:               Text.WordWrap
        }
    }
} 