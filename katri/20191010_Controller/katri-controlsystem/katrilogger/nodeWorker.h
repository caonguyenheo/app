#ifndef MYTCPSERVER_H
#define MYTCPSERVER_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QDebug>
#include <QVector>
#include "socketthread.h"
#include <QTimer>
#include "appConfig.h"
#include "command.h"
class NodeWorker : public QTcpServer
{
    Q_OBJECT
public:
    explicit NodeWorker(QObject *parent = 0);
    void startServer();
    QMap <QString, SocketThread*> getListDevice();
     void publishAll(Command &cmd);
     void publish(Command &cmd);
signals:
   void publishAllSocket(Command &cmd);
   void signalRecRelayState(QString loggerNodeName, bool status);

public slots:
    void deleteSocket(QString host);
    void slotReceiveRelayState(QString loggerNodeName, bool status);

protected:
    void incomingConnection(qintptr socketDescriptor);
private:
    QTcpServer *server;
    QMap <QString, SocketThread*> listDevice;
};

#endif // MYTCPSERVER_H
