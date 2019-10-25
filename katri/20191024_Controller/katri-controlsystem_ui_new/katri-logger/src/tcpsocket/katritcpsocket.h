#ifndef KATRITCPSOCKET_H
#define KATRITCPSOCKET_H


#include <QTcpSocket>
#include <QDebug>
#include "../command.h"
#include "QTimer"
#include "QtNetwork"
#include <QJsonDocument>
#include "../loggerManager.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;
class TcpSocket : public QTcpSocket
{
    Q_OBJECT
public:
    explicit TcpSocket(QObject *parent = 0);
    void relayState(bool isSuccessful);

signals:
    //void messageReceived(const Command command);

public slots:
    void handleMessageReceiving();
    void serverDisconnected();
     void doConnect();

protected:


signals:
    void messageReceived(const Command command);

private:
     std::shared_ptr<spdlog::logger> _logger = nullptr;

    QTimer   *timeReconnect;


};

#endif // KATRITCPSOCKET_H
