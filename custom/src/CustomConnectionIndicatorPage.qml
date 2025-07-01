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

ToolIndicatorPage {
    showExpand: true

    property var statusReceiver: CustomStatusReceiver

    contentComponent: Component {
        SettingsGroupLayout {
            heading: qsTr("连接状态")

            LabelledLabel {
                label:      qsTr("状态")
                labelText:  statusReceiver.connected ? qsTr("已连接") : qsTr("未连接")
            }

            LabelledLabel {
                label:      qsTr("地址")
                labelText:  "127.0.0.1:7777"
            }
        }
    }

}
