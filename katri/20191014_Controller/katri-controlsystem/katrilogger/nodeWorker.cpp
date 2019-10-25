#include "nodeWorker.h"

NodeWorker::NodeWorker(QObject *parent) :
    QTcpServer(parent)
{
}

void NodeWorker::startServer()
{
    QHostAddress host(AppConfig::instance().getIP());
    if(!this->listen(host, static_cast<quint16>(AppConfig::instance().getPort().toInt())))
    {
        qDebug() << "Could not start server";
    }
    else
    {
        qDebug() << "Start server " << AppConfig::instance().getIP() << " Port " << AppConfig::instance().getPort();
    }
}


void NodeWorker::incomingConnection(qintptr socketDescriptor)
{
    // We have a new connection
    qDebug() << socketDescriptor << " Connecting...";

    SocketThread *thread = new SocketThread(socketDescriptor, this);

    // connect signal/slot
    // once a thread is not needed, it will be beleted later
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    connect(this, &NodeWorker::publishAllSocket, thread, &SocketThread::publisher);
    connect(thread, &SocketThread::removeSocket, this, &NodeWorker::deleteSocket);
    connect(thread, &SocketThread::signalReceiveRelayState, this, &NodeWorker::slotReceiveRelayState);
    connect(thread, &SocketThread::signalReceiveVideoFrame, this, &NodeWorker::signalReceiveVideoFrame);
    thread->start();

    QString deviceIp = thread->getSocket()->peerAddress().toString();
    //register for device.
    listDevice.insert(deviceIp,thread);
    // We have a new connection
    qDebug() << thread->getSocket()->peerAddress().toString() << " Connected...";
    emit signalSetloggerConnect(AppConfig::instance().getListDeviceName(deviceIp));
}


void NodeWorker::publish(Command &cmd) {

    if (listDevice.size() > 0)
    {
        QString ip = AppConfig::instance().getHostForDevice(static_cast<uint8_t>(cmd.getType()),QString::fromStdString(cmd.getId()));
        QMap <QString, SocketThread*>::iterator socket;
        socket = listDevice.find(ip);
        if(socket != listDevice.end())
        {
            socket.value()->publisher(cmd);
        }
    }

}

QMap <QString, SocketThread*> NodeWorker::getListDevice()
{
    return  listDevice;
}

void NodeWorker::publishAll(Command &cmd)
{
    emit publishAllSocket(cmd);
}

void NodeWorker::deleteSocket(QString host)
{
    listDevice.remove(host);
    emit signalRemoveloggerConnected(AppConfig::instance().getListDeviceName(host));
}

void NodeWorker::slotReceiveRelayState(QString loggerNodeName, bool status)
{
    emit signalRecRelayState(loggerNodeName, status);
}
