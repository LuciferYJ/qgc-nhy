/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlightDisplay

// Custom版本：完全隐藏右下角的所有状态显示器
// 包括TelemetryValuesBar和FlyViewInstrumentPanel
RowLayout {
    // 空布局 - 不显示任何状态信息
    // 这样可以完全清理右下角的显示区域
    visible: false
} 