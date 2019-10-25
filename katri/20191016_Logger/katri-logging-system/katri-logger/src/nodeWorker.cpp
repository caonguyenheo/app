#include "nodeWorker.h"


NodeWorker::NodeWorker(QObject *parent) :
    QTcpSocket(parent)
{

    _logger = LoggerManager::getAppLogger();
    QHostAddress address(AppConfig::instance().getIpClient());
    this->setPeerAddress(address);
    timeReconnect = new QTimer;
    timeReconnect->setInterval(5000);
    connect(timeReconnect, SIGNAL(timeout()), this, SLOT(doConnect()));
    connect(this, SIGNAL(readyRead()), this, SLOT(handleMessageReceiving()),Qt::DirectConnection);
    connect(this, SIGNAL(disconnected()), this, SLOT(serverDisconnected()),Qt::DirectConnection);
    timeReconnect->start();
}

/*
* @implement
*/
void NodeWorker::doConnect()
{
    qDebug() << "Connect to server: " << AppConfig::instance().getIpServer() << "port: " << AppConfig::instance().getPortServer();
    this->connectToHost(AppConfig::instance().getIpServer(), static_cast<quint16>(AppConfig::instance().getPortServer().toInt()));

    if(this->waitForConnected(5000))
    {
        qDebug() << "Connected!";
        if(timeReconnect->isActive())
        {
            timeReconnect->stop();
        }
    }
    else
    {
        qDebug() << "Not connected!";
    }
}


void NodeWorker::handleMessageReceiving()
{

   // get the data from server
   QByteArray data = this->readAll();
   QString message = QString(QByteArray::fromBase64(data));

   // will write on server side window
   QJsonParseError parseError;
   QJsonDocument jsonResponse = QJsonDocument::fromJson(QByteArray::fromBase64(data), &parseError);
   //First, check for parsing errors, this way:
   if(parseError.error != QJsonParseError::NoError)
   {
       qDebug() << "Parse error: " << parseError.errorString();
   }
    QJsonObject json = jsonResponse.object();
    QJsonValue action = json.value("Action");
    QJsonValue type = json.value("Type");
    QJsonValue id = json.value("Id");
    QJsonValue kitti_No = json.value("Kitti_No");

    //Convert json to Command here and emit them
    Command command(static_cast<Command::Action>(QString(action.toString()).toInt()),
                               static_cast<Command::Type>(QString(type.toString()).toInt()),
                               id.toString().toStdString(),
                               kitti_No.toString().toUInt());

    qDebug() << "Received command: "
             << QString::fromStdString(Command::getActionName(command.getAction())) << ", "
             << QString::fromStdString(Command::getTypeName(command.getType())) << ", "
             << QString::fromStdString(command.getId()) << ", "
             << QString::number(command.getKittiNo());

   // Emit to run devive
    emit messageReceived(command);
    this->waitForBytesWritten();

}


void NodeWorker::serverDisconnected()
{
     timeReconnect->start();
}

void NodeWorker::relayState(bool isSuccessful) {

    QString mes = "";

    QString nodeName;
    if(isSuccessful) {

        mes.append(QString::number(static_cast<uint8_t>(Command::Action::RELAY_OK_STATE)));
        mes.append(",");
        mes.append(QString::number(static_cast<uint8_t>(Command::Type::LOGGER)));
        mes.append(",");
        mes.append(AppConfig::instance().getDeviceName());

        qDebug() << mes;
        this->write(mes.toLocal8Bit());
        this->waitForBytesWritten();

        nodeName = AppConfig::instance().getDeviceName();
        if (_logger)
            _logger->info("Sent RELAY_STATE command Done: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_OK_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          nodeName.toStdString());
        qDebug()<<"relayState successfuly";
    }
    else {
        mes.append(QString::number(static_cast<uint8_t>(Command::Action::RELAY_FAIL_STATE)));
        mes.append(",");
        mes.append(QString::number(static_cast<uint8_t>(Command::Type::LOGGER)));
        mes.append(",");
        mes.append(AppConfig::instance().getDeviceName());

        this->write(mes.toLocal8Bit());
        this->waitForBytesWritten();

        nodeName = AppConfig::instance().getDeviceName();
        if (_logger)
            _logger->info("Sent RELAY_STATE command Fail: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_FAIL_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          nodeName.toStdString());
        qDebug()<<"relayState Fail";
    }
}

void NodeWorker::sendImage(QByteArray &arr)
{
    qDebug()<<"arr.size()"<<arr.size();
    //qDebug()<<"data:"<<arr;

    try {
        this->write(arr);
    } catch (std::exception &e) {
        if (_logger)
            _logger->debug("Got exception when sending image: {}", e.what());
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when sending image");
    }
}
