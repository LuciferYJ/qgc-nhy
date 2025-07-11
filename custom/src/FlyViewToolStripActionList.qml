/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQml.Models

import QGroundControl
import QGroundControl.Controls

ToolStripActionList {
    id: _root

    signal displayPreFlightChecklist

    model: [
        PreFlightCheckListShowAction { onTriggered: displayPreFlightChecklist() },
        
        // 隐藏原生按钮，使用自定义的takeoff按钮
        // GuidedActionTakeoff { },
        // GuidedActionLand { },
        // GuidedActionRTL { },
        // GuidedActionPause { },
        // FlyViewAdditionalActionsButton { },
        
        // 自定义的Takeoff按钮
        GuidedToolStripAction {
            text:       "起飞"                    // 自定义起飞按钮文字
            iconSource: "/res/takeoff.svg"           // 使用起飞图标
            visible:    true                         // 始终可见
            enabled:    true                         // 始终可用
            actionID:   _guidedController._customController.actionCustomTakeoff
        },
        
        // 自定义的Home按钮 - 直接执行，无确认
        ToolStripAction {
            text:       "Home"                // 自定义设置Home点按钮文字
            iconSource: "/res/home.svg"             // 使用自定义小房子图标
            visible:    true                       // 始终可见
            enabled:    true                       // 始终可用
            onTriggered: {
                _guidedController._customController._handleMyCustomHome()
            }
}
    ]
}
