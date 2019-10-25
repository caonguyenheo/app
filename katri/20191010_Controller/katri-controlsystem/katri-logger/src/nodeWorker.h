#ifndef NODEWORKER_H
#define NODEWORKER_H


#include <QTcpSocket>
#include <QDebug>
#include "command.h"
#include "QTimer"
#include "QtNetwork"
#include <QJsonDocument>
#include "loggerManager.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include "appConfig.h"

using namespace std::chrono_literals;
class NodeWorker : public QTcpSocket
{
    Q_OBJECT
public:
    explicit NodeWorker(QObject *parent = 0);
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

#endif // NODEWORKER_H
