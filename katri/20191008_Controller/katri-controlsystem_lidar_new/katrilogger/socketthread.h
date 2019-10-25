#ifndef SOCKETHREAD_H
#define SOCKETHREAD_H

#include <QThread>
#include <QTcpSocket>
#include <QDebug>
#include <QTime>
#include "command.h"
class SocketThread : public QThread
{
    Q_OBJECT
public:
    explicit SocketThread(qintptr ID, QObject *parent = 0);

    void run();
    QTcpSocket *getSocket();

signals:
    void error(QTcpSocket::SocketError socketerror);
    void removeSocket(QString host);
    void signalReceiveRelayState(QString loggerNodeName, bool status);
public slots:
    void readyRead();
    void disconnected();
    void publisher(Command &cmd);

private:
    QTcpSocket *socket;
    qintptr socketDescriptor;
    QScopedPointer<QTcpSocket> m_socket;

    void receiveCommand(QByteArray &Data);
};

#endif // SOCKETHREAD_H
