/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "UdpCommandLink.h"
#include <QDebug>

// 静态成员初始化
UdpCommandLink* UdpCommandLink::_instance = nullptr;
QMutex UdpCommandLink::_mutex;

UdpCommandLink::UdpCommandLink(const QString& sendRemoteHost, quint16 sendRemotePort, quint16 receiveLocalPort, QObject *parent) : 
    QObject(parent),
    _sendSocket(nullptr),
    _receiveSocket(nullptr),
    _sendRemoteAddress(sendRemoteHost),
    _sendRemotePort(sendRemotePort),
    _receiveLocalPort(receiveLocalPort),
    _isInitialized(false)
{
    qDebug() << "UdpCommandLink: 构造函数 - 发送目标:" << sendRemoteHost << ":" << sendRemotePort 
             << ", 接收端口:" << receiveLocalPort;
}

UdpCommandLink::~UdpCommandLink()
{
    if (_sendSocket) {
        _sendSocket->close();
        delete _sendSocket;
        _sendSocket = nullptr;
    }
    if (_receiveSocket) {
        _receiveSocket->close();
        delete _receiveSocket;
        _receiveSocket = nullptr;
    }
    qDebug() << "UdpCommandLink: 析构函数";
}

UdpCommandLink* UdpCommandLink::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine)
{
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    return UdpCommandLink::instance();
}

UdpCommandLink* UdpCommandLink::instance(const QString& sendRemoteHost, quint16 sendRemotePort, quint16 receiveLocalPort)
{
    if (_instance == nullptr) {
        _mutex.lock();
        if (_instance == nullptr) {
            // 只有在提供了完整配置时才创建新实例
            if (!sendRemoteHost.isEmpty() && sendRemotePort != 0 && receiveLocalPort != 0) {
                _instance = new UdpCommandLink(sendRemoteHost, sendRemotePort, receiveLocalPort);
                _instance->initialize();
            } else {
                qWarning() << "UdpCommandLink: 无法创建实例 - 需要完整的网络配置参数";
                qWarning() << "sendRemoteHost:" << sendRemoteHost << "sendRemotePort:" << sendRemotePort << "receiveLocalPort:" << receiveLocalPort;
            }
        }
        _mutex.unlock();
    } else if (!sendRemoteHost.isEmpty() || sendRemotePort != 0 || receiveLocalPort != 0) {
        // 已存在实例但收到了新的配置，更新现有实例
        _mutex.lock();
        bool needReinit = false;
        if (!sendRemoteHost.isEmpty() && _instance->_sendRemoteAddress.toString() != sendRemoteHost) {
            _instance->setSendRemoteAddress(sendRemoteHost);
            needReinit = true;
        }
        if (sendRemotePort != 0 && _instance->_sendRemotePort != sendRemotePort) {
            _instance->setSendRemotePort(sendRemotePort);
            needReinit = true;
        }
        if (receiveLocalPort != 0 && _instance->_receiveLocalPort != receiveLocalPort) {
            _instance->setReceiveLocalPort(receiveLocalPort);
            needReinit = true;
        }
        if (needReinit) {
            // 强制重新初始化
            _instance->_isInitialized = false;
            _instance->initialize();
        }
        _mutex.unlock();
    }
    return _instance;
}

bool UdpCommandLink::initialize()
{
    qDebug() << "UdpCommandLink: 开始初始化...";
    qDebug() << "当前状态 - 已初始化:" << _isInitialized;
    qDebug() << "发送目标:" << _sendRemoteAddress.toString() << ":" << _sendRemotePort;
    qDebug() << "接收端口:" << _receiveLocalPort;
    
    if (_isInitialized) {
        qDebug() << "UdpCommandLink: 已经初始化，跳过";
        return true;
    }

    // 检查参数是否完整
    if (_sendRemoteAddress.isNull() || _sendRemotePort == 0 || _receiveLocalPort == 0) {
        qWarning() << "UdpCommandLink: 初始化失败 - 网络参数不完整";
        qWarning() << "发送目标:" << _sendRemoteAddress.toString() << ":" << _sendRemotePort;
        qWarning() << "接收端口:" << _receiveLocalPort;
        qWarning() << "地址是否为空:" << _sendRemoteAddress.isNull();
        return false;
    }

    // 清理旧的套接字
    if (_sendSocket) {
        delete _sendSocket;
        _sendSocket = nullptr;
    }
    if (_receiveSocket) {
        delete _receiveSocket;
        _receiveSocket = nullptr;
    }

    // 创建发送套接字
    _sendSocket = new QUdpSocket(this);
    qDebug() << "UdpCommandLink: 创建发送套接字成功";
    
    // 创建接收套接字并绑定到指定端口
    // 绑定到0.0.0.0，这样可以接收来自任何IP的数据，更新发送IP时不会影响接收
    _receiveSocket = new QUdpSocket(this);
    if (!_receiveSocket->bind(QHostAddress::Any, _receiveLocalPort)) {
        qWarning() << "UdpCommandLink: 无法绑定接收端口" << _receiveLocalPort << ":" << _receiveSocket->errorString();
        delete _sendSocket;
        delete _receiveSocket;
        _sendSocket = nullptr;
        _receiveSocket = nullptr;
        return false;
    }
    
    // 连接接收套接字的信号槽
    connect(_receiveSocket, &QUdpSocket::readyRead, this, &UdpCommandLink::_readPendingDatagrams);
    
    qDebug() << "UdpCommandLink: 初始化成功";
    qDebug() << "发送套接字 -> 目标:" << _sendRemoteAddress.toString() << ":" << _sendRemotePort;
    qDebug() << "接收套接字 -> 监听 0.0.0.0:" << _receiveLocalPort << "(接收来自任何IP的数据)";
    
    _isInitialized = true;
    return true;
}

qint64 UdpCommandLink::sendCommand(const QByteArray &data)
{
    if (!_sendSocket || !_isInitialized) {
        qWarning() << "UdpCommandLink: 尝试在未初始化的发送套接字上发送数据";
        return -1;
    }
    
    qint64 bytesSent = _sendSocket->writeDatagram(data, _sendRemoteAddress, _sendRemotePort);
    
    if (bytesSent == -1) {
        qWarning() << "UdpCommandLink: 发送数据失败:" << _sendSocket->errorString();
    } else {
        qDebug() << "UdpCommandLink: 发送" << bytesSent << "字节到" << _sendRemoteAddress.toString() << ":" << _sendRemotePort;
    }
    
    return bytesSent;
}

void UdpCommandLink::_readPendingDatagrams()
{
    while (_receiveSocket && _receiveSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(_receiveSocket->pendingDatagramSize());
        
        QHostAddress sender;
        quint16 senderPort;
        
        _receiveSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        
        //qDebug() << "UdpCommandLink: 接收到" << datagram.size() << "字节，来自" << sender.toString() << ":" << senderPort;
        
        // 发出信号，通知接收到数据
        emit dataReceived(datagram);
    }
}

void UdpCommandLink::setSendRemoteAddress(const QString &address)
{
    _sendRemoteAddress = QHostAddress(address);
    qDebug() << "UdpCommandLink: 设置发送目标地址为" << address;
}

void UdpCommandLink::setSendRemotePort(quint16 port)
{
    _sendRemotePort = port;
    qDebug() << "UdpCommandLink: 设置发送目标端口为" << port;
}

void UdpCommandLink::setReceiveLocalPort(quint16 port)
{
    // 如果已经初始化，需要重新初始化以使用新端口
    bool wasInitialized = _isInitialized;
    
    if (wasInitialized) {
        if (_receiveSocket) {
            _receiveSocket->close();
            delete _receiveSocket;
            _receiveSocket = nullptr;
        }
        _isInitialized = false;
    }
    
    _receiveLocalPort = port;
    qDebug() << "UdpCommandLink: 设置接收监听端口为" << port;
    
    if (wasInitialized) {
        initialize();
    }
}

// 静态方法实现
void UdpCommandLink::setGlobalSendRemoteAddress(const QString &address)
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        instance->setSendRemoteAddress(address);
        qDebug() << "UdpCommandLink: 全局设置发送目标地址为" << address;
    } else {
        qWarning() << "UdpCommandLink: 实例不存在，无法设置发送目标地址";
    }
}

void UdpCommandLink::setGlobalSendRemotePort(quint16 port)
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        instance->setSendRemotePort(port);
        qDebug() << "UdpCommandLink: 全局设置发送目标端口为" << port;
    } else {
        qWarning() << "UdpCommandLink: 实例不存在，无法设置发送目标端口";
    }
}

void UdpCommandLink::setGlobalReceiveLocalPort(quint16 port)
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        instance->setReceiveLocalPort(port);
        qDebug() << "UdpCommandLink: 全局设置接收监听端口为" << port;
    } else {
        qWarning() << "UdpCommandLink: 实例不存在，无法设置接收监听端口";
    }
}

QString UdpCommandLink::getGlobalSendRemoteAddress()
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        return instance->_sendRemoteAddress.toString();
    }
    return QString();
}

quint16 UdpCommandLink::getGlobalSendRemotePort()
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        return instance->_sendRemotePort;
    }
    return 0;
}

quint16 UdpCommandLink::getGlobalReceiveLocalPort()
{
    UdpCommandLink* instance = UdpCommandLink::instance();
    if (instance) {
        return instance->_receiveLocalPort;
    }
    return 0;
} 