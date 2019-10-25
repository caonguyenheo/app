#include <QtCore/QJsonObject>
#include "socketthread.h"
#include <QJsonDocument>
#include <QHostAddress>
#include <QImage>

const int MAX_COMMAND_SIZE = 1000;

SocketThread::SocketThread(qintptr ID, QObject *parent) :
    QThread(parent)
{
    this->socketDescriptor = ID;
    qDebug() << " Thread started";
    socket = new QTcpSocket();

    // set the ID
    if(!socket->setSocketDescriptor(this->socketDescriptor))
    {
        // something's wrong, we just emit a signal
        emit error(socket->error());
        return;
    }

    // connect socket and signal
    // note - Qt::DirectConnection is used because it's multithreaded
    //        This makes the slot to be invoked immediately, when the signal is emitted.

    connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()), Qt::DirectConnection);
    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));

    // We'll have multiple clients, we want to know which is which
    qDebug() << socketDescriptor << " Client connected";

    // make this thread a loop,
    // thread will stay alive so that signal/slot to function properly
    // not dropped out in the middle when thread dies


}

void SocketThread::run()
{
    // thread starts here
    exec();

}

void SocketThread::readyRead()
{

   // will write on server side window
    qDebug()<< "Receive message from: " << socket->peerAddress();
    // get the data from server
    QByteArray data = socket->readAll();
    QByteArray message = QByteArray::fromBase64(data);

    // will write on server side window
    QJsonParseError parseError;
    QJsonDocument jsonResponse = QJsonDocument::fromJson(message, &parseError);
    //First, check for parsing errors, this way:
    if(parseError.error != QJsonParseError::NoError)
    {
        qDebug() << "Parse error: " << parseError.errorString();
    }
    QJsonObject json = jsonResponse.object();
    QJsonValue dataType = json.value("DataType");
    QJsonValue dataJson = json.value("Data");

    QByteArray arrayData =  pixmapFrom(dataJson);
    qDebug() << "data r: " << dataJson;

    QImage pixmap;
    pixmap.loadFromData(arrayData);
    pixmap.save("/home/kevin/file.jpg");
    //Convert json to Command here and emit them
    Command command(static_cast<Command::DataType >(QString(dataType.toString()).toInt()));

    switch(command.getDataType())
    {
        case Command::DataType::VIDEO_LIVE :
            receiveLiveData(arrayData);
            break;
        case Command::DataType::COMMAND :
            receiveCommand(arrayData);
            break;
        default:
            break;

    }

}

void SocketThread::disconnected()
{
    qDebug() << socket->peerAddress() << " Disconnected";
    emit removeSocket(socket->peerAddress().toString());
    socket->deleteLater();
    exit(0);
}

QTcpSocket *SocketThread::getSocket()
{
    return socket;
}


void SocketThread::publisher(Command &cmd)
{
    QJsonObject json;

    try
    {
        qDebug() << "Send message to " << socket->peerAddress();

        json.insert("Action", QString::number(static_cast<uint8_t>(cmd.getAction())));
        json.insert("Type", QString::number(static_cast<uint8_t>(cmd.getType())));
        json.insert("Id", QString::fromStdString(cmd.getId()));
        json.insert("Kitti_No", QString::number(static_cast<uint>(cmd.getKittiNo())));

        QByteArray messageArr = QJsonDocument(json).toJson().toBase64();
        socket->write(messageArr);
        socket->waitForBytesWritten();

    }
    catch (std::exception &e)
    {
        qDebug() << "Got exception when publishing a message: " << e.what();
    }
    catch (...)
    {
        qDebug() << "Got error when publishing a message";
    }
}

void SocketThread::receiveCommand(QByteArray data)
{
      if (data.size() <= MAX_COMMAND_SIZE) {
        qDebug()<< " Data Receive: " << data;

        QString message = QString(data);
        QStringList dataSplit = message.split(",");
        if(dataSplit.size() < 3)
        {
            return;
        }

        //Convert json to Command here and emit them
        Command command(static_cast<Command::Action>(dataSplit[0].toInt()),
                        static_cast<Command::Type>(dataSplit[1].toInt()),
                        dataSplit[2].toStdString(),
                        0);


        qDebug() << "Received command: "
                 << QString::fromStdString(Command::getActionName(command.getAction())) << ", "
                 << QString::fromStdString(Command::getTypeName(command.getType())) << ", "
                 << QString::fromStdString(command.getId()) << ", "
                 << QString::number(command.getKittiNo());

        if (command.isValid()) {
            // Get ACK node list
            /*if (command.getAction() == Command::Action::ACK) {
                emit signalLoggerNameAck(QString::fromStdString(command.getId()));
            }*/

            if (command.isAckFromLoggerNodeWithLoggerNodeName()) {
                // Logger stop sending ACK message -> redundant
                // Stop retry
                /*_retry = 0;
                this->_one_off_timer->cancel();

                qDebug() << "Katri Logger Node: " << QString::fromStdString(command.getId())
                         << " is connected to Katri Controller Node: "
                         << AppConfig::instance().getNodeName();*/

            } else if (command.isPingFromLoggerNode()) {
                // Send ACK message to Logger Node
                //ack(command.getId());
                //emit signalPingLoggerNodeId(QString::fromStdString(command.getId()));
            }
            else if (command.isOkRelayResponse()) {
                qDebug() << "Received RELAY OK";
                //relayState(command);
                emit signalReceiveRelayState(QString::fromStdString(command.getId()), true);
            }
            else if (command.isFailRelayResponse()) {
                qDebug() << "Received RELAY FAIL";
                //relayState(command);
                emit signalReceiveRelayState(QString::fromStdString(command.getId()), false);
            }
            else {
                qDebug() << "Ignore a command";
            }
        }
        else {
            qDebug() << "An invalid command, just ignore it.";
        }
    }

}

void SocketThread::receiveLiveData(QByteArray data)
{
    // Convert IP to logger ID
    QString ipAddr = socket->peerAddress().toString();
    int id = AppConfig::instance().getLoggerIdFromIp(ipAddr);
    emit signalReceiveVideoFrame(0x02, id, data);
}


QByteArray SocketThread::pixmapFrom(const QJsonValue &val) {
    auto const encoded = val.toString().toLatin1();
//    QPixmap p;
//    p.loadFromData(QByteArray::fromBase64(encoded), "JPG");

    return QByteArray::fromBase64(encoded);

}
