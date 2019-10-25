#include "katritcpsocket.h"


TcpSocket::TcpSocket(QObject *parent) :
    QTcpSocket(parent)
{

    _logger = LoggerManager::getAppLogger();

    QString ip  = AppConfig::instance().getIp();
    QString post = AppConfig::instance().getPostServer();
    this->setPeerAddress(QHostAddress::AnyIPv4);

    //this->connectToHost(ip, quint16(post.toInt()));
     this->connectToHost("192.168.1.117", 8088);


    connect(this, SIGNAL(readyRead()), this, SLOT(readyRead()),Qt::DirectConnection);

    timeReconnect = new QTimer;
    timeReconnect->setInterval(5000);
    connect(timeReconnect, SIGNAL(timeout()), this, SLOT(doConnect()));
    connect(this, SIGNAL(readyRead()), this, SLOT(handleMessageReceiving()),Qt::DirectConnection);
    connect(this, SIGNAL(disconnected()), this, SLOT(serverDisconnected()),Qt::DirectConnection);

}

void TcpSocket::doConnect()
{
    this->connectToHost("192.168.1.187", 8088);

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


void TcpSocket::handleMessageReceiving()
{
//    QTime time;
//    time.start();

//   // get the information
//   QString Data = this->readAll();
//   QStringList data = Data.split(" ");

//   // will write on server side window
//   qDebug() << " Data in: " << Data << "time: " <<time;
//   //this->write(Data);
//   Command::Action action =static_cast<Command::Action>(data[0].toInt());
//   Command::Type type = static_cast<Command::Type>(data[1].toInt());
//   Command command(action, type, data[2].toStdString(), data[3].toUInt());

//   emit messageReceived(command);


   // get the data from server
   QByteArray data = this->readAll();
   QString message = QString(QByteArray::fromBase64(data));

   // will write on server side window
   qDebug() << " Data in: " << message << "time: " <<time;
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

    qDebug() << " Data in json: " << action  << "timen: " <<time;
   // Emit to run devive
    emit messageReceived(command);

    this->write("CLient has received with");
    this->waitForBytesWritten();

}


void TcpSocket::serverDisconnected()
{
     QTime time;
     qDebug() << " disconnected timen: " <<time;
     timeReconnect->start();
}

void TcpSocket::relayState(bool isSuccessful) {

    QJsonObject json;
    QString nodeName;
    if(isSuccessful) {
        json.insert("Action",static_cast<uint8_t>(Command::Action::RELAY_OK_STATE));
        json.insert("Type", static_cast<uint8_t>(Command::Type::LOGGER));
        json.insert("Id", AppConfig::instance().getNodeName());
         QByteArray messageArr = QJsonDocument(json).toJson().toBase64();
        this->write(messageArr);
        this->waitForBytesWritten();

        nodeName = AppConfig::instance().getNodeName();
        if (_logger)
            _logger->info("Sent RELAY_STATE command Done: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_OK_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          nodeName.toStdString());
        qDebug()<<"relayState successfuly";
    }
    else {
        json.insert("Action",static_cast<uint8_t>(Command::Action::RELAY_FAIL_STATE));
        json.insert("Type", static_cast<uint8_t>(Command::Type::LOGGER));
        json.insert("Id", AppConfig::instance().getNodeName());
         QByteArray messageArr = QJsonDocument(json).toJson().toBase64();
        this->write(messageArr);
        this->waitForBytesWritten();
        nodeName = AppConfig::instance().getNodeName();
        if (_logger)
            _logger->info("Sent RELAY_STATE command Fail: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_FAIL_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          nodeName.toStdString());
        qDebug()<<"relayState Fail";
    }
}

