/****************************************************************************
 *
 * (c) 2009-2019 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QObject>
#include <QString>
#include <QQmlEngine>
#include <QJSEngine>

/**
 * @brief 任务记录管理器
 * 
 * 简单的任务记录状态管理：
 * - 手动开始/停止任务记录
 * - 提供QML接口
 * - 记录状态通知
 */
class MissionRecordingManager : public QObject
{
    Q_OBJECT

    // 暴露给QML的属性
    Q_PROPERTY(bool isRecording READ isRecording NOTIFY isRecordingChanged)

public:
    explicit MissionRecordingManager(QObject* parent = nullptr);
    ~MissionRecordingManager();

    // QML Singleton创建函数
    static QObject* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);

    // 属性访问器
    bool isRecording() const { return _isRecording; }

    // 手动控制接口
    Q_INVOKABLE void startRecording();
    Q_INVOKABLE void stopRecording();

signals:
    void isRecordingChanged();
private:
    // 核心业务逻辑
    void _startMissionRecording();
    void _stopMissionRecording();

    // 属性设置函数
    void _setIsRecording(bool recording);

    // 成员变量
    bool _isRecording;
}; 