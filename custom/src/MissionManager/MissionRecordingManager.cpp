/****************************************************************************
 *
 * (c) 2009-2019 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "MissionRecordingManager.h"

#include <QDebug>

MissionRecordingManager::MissionRecordingManager(QObject* parent)
    : QObject(parent)
    , _isRecording(false)
{
    qDebug() << "MissionRecordingManager: 初始化";
}

MissionRecordingManager::~MissionRecordingManager()
{
    qDebug() << "MissionRecordingManager: 析构";
    if (_isRecording) {
        stopRecording();
    }
}

QObject* MissionRecordingManager::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine)
{
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    
    static MissionRecordingManager* instance = nullptr;
    if (!instance) {
        instance = new MissionRecordingManager();
        qDebug() << "MissionRecordingManager: 创建单例实例";
    }
    return instance;
}

void MissionRecordingManager::startRecording()
{
    qDebug() << "MissionRecordingManager: 手动开始记录";
    _startMissionRecording();
}

void MissionRecordingManager::stopRecording()
{
    qDebug() << "MissionRecordingManager: 手动停止记录";
    _stopMissionRecording();
}

void MissionRecordingManager::_startMissionRecording()
{
    qDebug() << "MissionRecordingManager: === 开始任务记录 ===";
    
    if (_isRecording) {
        qDebug() << "MissionRecordingManager: 任务记录已在进行中";
        return;
    }

    // 简单地设置记录状态为true
    _setIsRecording(true);
    qDebug() << "MissionRecordingManager: 任务记录已开始";
}

void MissionRecordingManager::_stopMissionRecording()
{
    qDebug() << "MissionRecordingManager: === 停止任务记录 ===";
    
    if (!_isRecording) {
        qDebug() << "MissionRecordingManager: 任务记录未激活";
        return;
    }

    // 简单地设置记录状态为false
    _setIsRecording(false);
    qDebug() << "MissionRecordingManager: 任务记录已停止";
}

// 属性设置函数
void MissionRecordingManager::_setIsRecording(bool recording)
{
    if (_isRecording != recording) {
        _isRecording = recording;
        emit isRecordingChanged();
    }
}
