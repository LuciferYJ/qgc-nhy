/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

// Custom builds can override this file to add custom guided actions.

import QtQml

QtObject {
    readonly property int actionCustomTakeoff: _guidedController.customActionStart + 0
    readonly property int actionCustomHome: _guidedController.customActionStart + 1

    readonly property string customButtonTitle: qsTr("我的起飞")
    readonly property string customHomeTitle: qsTr("设置Home")

    readonly property string customButtonMessage: qsTr("执行自定义起飞逻辑，无需检查载具状态")
    readonly property string customHomeMessage: qsTr("执行自定义设置Home逻辑")

    function customConfirmAction(actionCode, actionData, mapIndicator, confirmDialog) {
        switch (actionCode) {
        case actionCustomTakeoff:
            confirmDialog.hideTrigger = true
            confirmDialog.title = customButtonTitle
            confirmDialog.message = customButtonMessage
            break
        case actionCustomHome:
            confirmDialog.hideTrigger = true
            confirmDialog.title = customHomeTitle
            confirmDialog.message = customHomeMessage
            break
        default:
            return false // false = action not handled here
        }

        return true // true = action handled here
    }

    function customExecuteAction(actionCode, actionData, sliderOutputValue, optionChecked) {
        switch (actionCode) {
        case actionCustomTakeoff:
            console.log("🚁 [自定义] 执行自定义takeoff逻辑")
            _handleMyCustomTakeoff()
            break
        case actionCustomHome:
            console.log("🏠 [自定义] 执行自定义设置Home逻辑")
            _handleMyCustomHome()
            break
        default:
            return false // false = action not handled here
        }

        return true // true = action handled here
    }

    function _handleMyCustomTakeoff() {
        console.log("🚁 [自定义] 开始自定义起飞流程")
        console.log("🚁 [自定义] 目标高度:", altitudeValue)
        console.log("🚁 [自定义] 选项状态:", optionChecked)
 

    }
    
    function _handleMyCustomHome() {
        console.log("🏠 [自定义] 执行自定义设置Home逻辑")
        
        
    }

    function _executeStandardTakeoffIfNeeded() {

    }
}