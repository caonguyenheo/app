#include "mainwindow.h"

#include <QHBoxLayout>
#include <QFontDatabase>
#include "controlpanel.h"
#include "common/common.h"


static QMap<QString,RelayStatus> relayArray;

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent)
{
    this->setLayout(new QHBoxLayout());
    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
    setWindowFlags( Qt::Dialog
                  | Qt::CustomizeWindowHint
//                  | Qt::WindowTitleHint
//                  | Qt::WindowCloseButtonHint
//                  | Qt::WindowMinimizeButtonHint
//                  | Qt::WindowMaximizeButtonHint
//                  | Qt::FramelessWindowHint
                    );

    // Set background color
    this->setupUi();
    this->setStyleSheet( "background-color:#272727" );
    this->setMinimumSize(1010,580);
    this->setGeometry(450, 250, 1010, 580);
    //this->showFullScreen();
    this->setupMediator();
    this->setupLogFile();

    if (QFontDatabase::addApplicationFont(":/fonts/Roboto/Roboto-Bold.ttf") < 0){ }

    else {
        this->setStyleSheet("font-family:'Roboto';");
    }
}

MainWindow::~MainWindow()
{
    delete mediator;
}

/**
 * Start a thread for ROS Node in seperate thread.
 * It should be called before displaying the UI.
 * @brief MainWindow::startNode
 * @param argc
 * @param argv
 */
void MainWindow::startNode(int argc, char *argv[])
{
    // Start a thread for ROS node

    //Start server
    server.startServer();

//    connect(node, SIGNAL(signalReceiveVideoFrame(int, int, QByteArray)),
//            this, SLOT(slotReceiveVideoFrame(int, int, QByteArray)));
//    connect(node, SIGNAL(signalLostNetWorkConnection(int, int, QByteArray)),
//            this, SLOT(slotReciveLostConnection(int, int, QByteArray)));

    connect(&server, SIGNAL(signalReceiveVideoFrame(int,int,QByteArray)),
            this, SLOT(slotReceiveVideoFrame(int,int,QByteArray)));
    connect(&server, SIGNAL(signalRecRelayState(QString, bool)),
            this, SLOT(slotReceiveRelayState(QString, bool)));

//    connect(node, SIGNAL(signalLoggerNameAck(QString)),
//            this, SLOT(slotLoggerNameAck(QString)));
//    connect(node, SIGNAL(signalPingLoggerNodeId(QString)),
//            this, SLOT(slotPingLoggerNodeId(QString)));
    connect(&server, SIGNAL(signalSetloggerConnect(QList<QString>)),
            this, SLOT(slotSetLoggerConnected(QList<QString>)));
    connect(&server, SIGNAL(signalRemoveloggerConnected(QList<QString>)),
            this, SLOT(slotRemoveLoggerConnected(QList<QString>)));
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QWidget::mousePressEvent(event);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == this->layout()->menuBar())
    {
        if (event->type() == QEvent::MouseButtonDblClick)
        {
            this->slotClickedMaximumSizeButton();
        }
    }
    return false;
}

void MainWindow::setupMediator()
{
    mediator = new ConcreteMediator();
    observer = new ConcreteObserver(mediator);

    QMap<Command::Type,ControlL*> mapControl = m_viewer->getMapControl();
    for(int i = 0; i < mapControl.keys().size(); i++)
    {
        Command::Type key = mapControl.keys().at(i);
        QList<RecordButton*> listButton = mapControl.value(key)->getListRecordButton();
        for(int j = 0; j < listButton.size(); j++){
            Device *device = new ConcreteDevice( mediator, int(listButton.at(j)->getID()), key);
            connect(listButton.at(j)->getRecordButton(), SIGNAL(clicked(bool)), device, SLOT(update(bool)));
//            connect(listButton.at(j)->getRecordButton(), SIGNAL(clicked(bool)), m_viewCamera, SLOT(slotSetActiveChecked(bool)));
        }
    }

    connect(observer, SIGNAL(signalSendCommand(Command::Type, int, bool, bool)), this, SLOT(slotSendCommand(Command::Type, int, bool, bool)));
    connect(observer, SIGNAL(signalSendCommand(Command::Type, int, bool, bool)), m_viewer, SLOT(slotGetSendCommandDevice(Command::Type, int, bool, bool)));
    connect(mediator,&ConcreteMediator::signalUpdateStatusCameraView, this, &MainWindow::handlerCameraView);
//    connect(mediator,&ConcreteMediator::signalUpdateStatusCameraControl, this, &MainWindow::handlerCameraControl);
//    connect(m_viewCamera, SIGNAL(signalClickedInside(bool, int)), this, SLOT(slotUpdateDeviceOutSide(bool, int)));

    //TEST
    //connect(mediator,&ConcreteMediator::signalAllButtonControlDisable,this,&MainWindow::slotButtonControlDisable);
    //connect(mediator,&ConcreteMediator::signalAllButtonControlEnable,this,&MainWindow::slotButtonControlEnable);
    //connect(message,&Message::signalPopUpMessage, this, &MainWindow::Test);
}

void MainWindow::setupLogFile()
{
    QString UpdateDate = QDate::currentDate().toString("dd_MM_yyyy");
    //qDebug()<<"UpdateDate = "<< UpdateDate ;

    //Check if Current date change
    //if date change then create a new logfile.

    if(UpdateDate!=Curdate){
        std::string new_date = UpdateDate.toStdString()+"_";
        Message::setlogMessage(new_date +"LogRecord.txt", "RECORD");
        Message::setlogMessage(new_date +"LogRelay.txt", "RELAY");
    }    
}


void MainWindow::handlerCameraView(uint id, bool status)
{
   qDebug() <<Q_FUNC_INFO << status;
   m_viewer->getControlViewCamera()->setStatus(id,status);
}

//void MainWindow::handlerCameraControl(uint id, bool status)
//{
//    qDebug() <<Q_FUNC_INFO;
////    m_viewCamera->setViewStatus(id,status);
//}

void MainWindow::setupUi()
{
    m_viewer = new ViewerPanel(this);
    m_record = new ControlPanel(this);
    m_fullscreen = new FullScreen(this);
    m_viewChart = new PerformanceManager(this);
    m_viewCamera = new DashboardPanel(this);
    m_menubar = new MenuBar(this);
    frame = new FrameLess(this);


    QHBoxLayout *layout = qobject_cast<QHBoxLayout*>(this->layout());
    layout->setSpacing(0);
    layout->setMargin(0);

    layout->addWidget(m_viewer);
    layout->addWidget(m_record);
    layout->addWidget(m_viewChart);
    layout->addWidget(m_viewCamera);
    layout->setStretch(0,4);
    layout->setStretch(1,1);

    m_viewer->viewChartConnection(this, SLOT(slotViewChartClicked()));
    m_viewer->viewCameraConnection(this, SLOT(slotCameraDashboardClicked()));
    m_record->recordAllConnection(this, SLOT(slotRecordAllBtnClicked(bool)));
    m_record->relayAllConnection(this, SLOT(slotRelayAllBtnClicked(bool)));
    m_record->stopAllConnection(this, SLOT(slotStopAllBtnClicked(bool)));
    m_viewChart->backButtonConnection(this, SLOT(slotbackButtonClicked()));
    m_viewCamera->backButtonConnection(this, SLOT(slotbackButtonClicked()));
//    m_viewCamera->recordAllConnection(this, SLOT(slotRecordAllBtnClickedviewcamera(bool)));
    //m_viewCamera->sendCommandConnection(this, SLOT(slotSendCommand(bool, int)));
    m_menubar->exitConnection(this,SLOT(slotClickedExitButton()));
    m_menubar->minimumSizeConnection(this,SLOT(slotClickedMinimumSizeButton()));
    m_menubar->maximumSizeConnection(this,SLOT(slotClickedMaximumSizeButton()));
    connect(m_viewer->getControlViewCamera(),SIGNAL(signalChangeStatus(bool)),this,SLOT(slotRecordAllBtnClickedviewcamera(bool)));
    connect(m_viewer,SIGNAL(signalResize(bool)),this,SLOT(slotResizeWindow(bool)));

    m_viewer->setVisible(true);
    m_record->setVisible(true);
    m_viewChart->setVisible(false);
    m_viewCamera->setVisible(false);
    m_fullscreen->setVisible(false);

    this->layout()->setMenuBar(m_menubar);
    this->layout()->menuBar()->installEventFilter(this);
    //m_record->setActiveStopEnabled(false);
    m_record->setRelayEnabled(1);
}

void MainWindow::slotButtonControlDisable()
{
    m_record->Stoprecording();
}

void MainWindow::slotUpdateDeviceOutSide(bool status, int deviceId)
{
    qDebug()<<"slotUpdateDeviceOutSide : "<<deviceId;
    QMap<Command::Type,ControlL*> mapControl = m_viewer->getMapControl();
    for(int i = 0; i < mapControl.keys().size(); i++)
    {
        Command::Type key = mapControl.keys().at(i);
        if(key == Command::Type::CAMERA) {
            QList<RecordButton*> listButton = mapControl.value(key)->getListRecordButton();
            for(int j = 0; j < listButton.size(); j++){
                if(listButton.at(j)->getID() == uint(deviceId))
                {
                    listButton.at(j)->setActive(status);
                    break;
                }
            }
        }
    }
}

void MainWindow::slotRecordAllBtnClicked(bool active)
{
    m_viewer->recordAllActivation(active);
//    m_viewCamera->recordAllActivation(active);
    m_viewer->setStatus(active);
    m_record->updateStatusRecordButton(active);
    if (active==true) {
        m_record->StartRecord(active);
    }
    else {
        m_record->setRelayEnabled(1);
    }
}

void MainWindow::slotRelayAllBtnClicked(bool active)
{
    if (m_viewer->isRecording() == false && active == true) {
        this->m_viewer->setEnableAllView(false);
        this->m_record->startRelayingStatus();
        sendRelayCommand();
}
    else {
        this->m_record->stopRelayingStatus();
        //this->m_record->stoploading();
    }
}

void MainWindow::slotStopAllBtnClicked(bool active)
{
    m_record->updateStatusStopButton(active);
    if (active==true) {
    m_record->StopRecord(active);
    }
    else {
        m_record->setRelayEnabled(!active);
    }
}

void MainWindow::slotRecordAllBtnClickedviewcamera(bool active)
{
    qDebug()<<"slotRecordAllBtnClickedviewcamera: "<<active;
    m_viewer->getControlViewCamera()->setStatus(active);
//    if(active != m_viewCamera->getStatus()) {
//        m_viewCamera->slotSetActiveChecked(active);
//    }
}
void MainWindow::slotResizeWindow(bool active)
{
    if (active)
        this->setMinimumSize(1010, 580);
        this->resize(1010, 580);
}


void MainWindow::slotViewChartClicked()
{
    m_viewer->setVisible(false);
    m_record->setVisible(false);
    this->setMinimumSize(1010, 580);
    this->resize(1010, 580);
    this->geometry().x();
    m_viewChart->setVisible(true);
}

void MainWindow::slotbackButtonClicked()
{
    m_viewChart->setVisible(false);

    m_viewCamera->setVisible(false);
    this->setMinimumSize(1010, 580);
    this->resize(1010, 580);
    m_viewer->setVisible(true);
    m_record->setVisible(true);
}

void MainWindow::slotCameraDashboardClicked()
{
    m_viewer->setVisible(false);
    m_record->setVisible(false);
    m_viewChart->setVisible(false);
    this->setMinimumSize(1010, 580);
    this->resize(1010, 580);
    m_viewCamera->setVisible(true);
    //m_viewCamera->setStatus(true);
}

void MainWindow::slotClickedExitButton()
{
    this->close();
}

void MainWindow::slotClickedMinimumSizeButton()
{
    this->showMinimized();
}

void MainWindow::slotClickedMaximumSizeButton()
{
    if(this->isMaximized() == false)
    {
        m_menubar->pBtnMax->setIcon(QIcon(":/img/icon/Retore-ic.png"));
        this->showMaximized();
    }
    else {
        m_menubar->pBtnMax->setIcon(QIcon(":/img/icon/Maximize-ic.png"));
        this->showNormal();
    }
}

void MainWindow::slotSendCommand(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS)
{
    // Implement send command to Logger
    if (recordStatus == true && isNewDS == true) {
        startNewKittiCommand( deviceType,
                              QString::number(deviceId)
                                  .rightJustified(3, '0', true)
                                  .toStdString());
    }
    else if (recordStatus == true && isNewDS == false) {
        startCurrentKittiCommand( deviceType,
                                  QString::number(deviceId)
                                      .rightJustified(3, '0', true)
                                      .toStdString());
    }
    else {
        stopRecordCommand( deviceType,
                           QString::number(deviceId)
                               .rightJustified(3, '0', true)
                               .toStdString());
    }
}

void MainWindow::startNewKittiCommand(Command::Type loggerType, string loggerId)
{
    Runtime runtime;
    uint currSeq = 0;
    QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
    if (runtime.getCurrentDate() == currentDate) {
        currSeq = runtime.getCurrentSequence().toUInt();
    }

    if (runtime.getCurrentDate() == currentDate) {
        currSeq = runtime.getNextSequence().toUInt();
    }

    Command cmd(Command::Action::START_NEW_KITTI,
                loggerType,
                loggerId,
                currSeq);

    server.publish(cmd);
//    node->publish(cmd);
    // Update current sequence and current date
    runtime.setCurrentSequence(currSeq);
    runtime.setCurrentDate();

    // ROS command message
    qDebug() << "Send Command: " << " startRecord = true" << ", newDataSet = true"
             << ", loggerId =" << QString::fromStdString(loggerId);
}

void MainWindow::startCurrentKittiCommand(Command::Type loggerType, string loggerId)
{
    Runtime runtime;
    uint currSeq = 0;
    QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
    if (runtime.getCurrentDate() == currentDate) {
        currSeq = runtime.getCurrentSequence().toUInt();
    }

    Command cmd(Command::Action::START,
                loggerType,
                loggerId,
                currSeq);
//    node->publish(cmd);
    server.publish(cmd);
    // ROS command message
    qDebug() << "Send Command: " << " startRecord = true" << ", newDataSet = false"
             << ", loggerId =" << QString::fromStdString(loggerId);
}

void MainWindow::stopRecordCommand(Command::Type loggerType, string loggerId)
{
    Command stopCmd(Command::Action::STOP, loggerType, loggerId, 0);

    //node->publish(stopCmd);
    server.publish(stopCmd);

    // ROS command message
    qDebug() << "Send Command: " << " startRecord = stop" << ", loggerId ="
             << QString::fromStdString(loggerId);
}

void MainWindow::slotReceiveVideoFrame(int deviceType, int deviceId, QByteArray data)
{
    qDebug() << "Receive video frame"
             << ": deviceType=" << uint8_t(deviceType)
             << ", deviceId=" << deviceId
             << ", data=" << &data;

    if (deviceId >= 0 && deviceId <= 3) {
        qDebug() << "Send frame data to HD camera " << deviceId;
        m_viewCamera->setDataViewHD(deviceId, data);
    }
    else if (deviceId >= 4 && deviceId <= 7) {
        qDebug() << "Send frame data to FHD camera " << deviceId;
        m_viewCamera->setDataViewFullHD(deviceId%4, data);
    }
    else {
        qDebug() << "No camera recieve data";
    }
}
void MainWindow::slotReciveLostConnection(int deviceType, int deviceId, QByteArray data)
{
    qDebug() << "slotReciveLostConnection"
             << ": deviceType=" << uint8_t(deviceType)
             << ", deviceId=" << deviceId
             << ", data=" << &data;
    QString message =  QString::fromLatin1(data.data());
    qDebug()<<"message = "<<message;
    Message::logMessage(MessageType::LOG, MessageStatus::FAILED, message.toStdString(), "GPS");
}

void MainWindow::sendRelayCommand()
{
    // Reset logger node status status
    m_relayTimeout = false;
    relayArray.clear();
    Q_FOREACH(QString key, this->m_viewer->getLoggerNodeNameList()) {
        relayArray.insert(key, RelayStatus::NONE);
    }
    qDebug() << "Relay logger node list:" << relayArray.keys();

    Command relayCmd(Command::Action::COPY, Command::Type::ALL, "000", 0);

//    node->publish(relayCmd);

    server.publishAll(relayCmd);

    QTimer::singleShot(AppConfig::instance().getRelayTimeout(), this, SLOT(slotRelayTimeout()));
}

//void MainWindow::slotLoggerNameAck(QString loggerNodeName)
//{
//    int index = relayArray.keys().indexOf(loggerNodeName);
//    qDebug() << index;
//    if (index < 0) {
//        relayArray.insert(loggerNodeName, RelayStatus::NONE);
//    }
//}

void MainWindow::slotReceiveRelayState(QString loggerNodeName, bool status)
{
    qDebug() << "slotReceiveRelayState";
    if (m_relayTimeout == false) {
        RelayStatus relay = status ? RelayStatus::RELAYOK : RelayStatus::RELAYFAIL;
        qDebug() << "Relay:" << loggerNodeName << uint8_t(relay);

        int index = relayArray.keys().indexOf(loggerNodeName);
        qDebug() << index;
        if (index < 0) {
            relayArray.insert(loggerNodeName, relay);
        }
        else {
            relayArray[loggerNodeName] = relay;
        }

        // Verify relay all done
        verifyRelayAllDone();
    }
}

void MainWindow::verifyRelayAllDone()
{
    QStringList failLoggerName;
    bool existedNone = false;
    // Check all logger status is responded
    Q_FOREACH(QString key, relayArray.keys()) {
        if (relayArray.value(key) == RelayStatus::NONE) {
            existedNone = true;
        }
        else if (relayArray.value(key) == RelayStatus::RELAYFAIL) {
            failLoggerName << key;
        }
    }

    if (!existedNone) {
        this->m_viewer->setEnableAllView(true);
        this->m_record->stopRelayingStatus();
    }

    // Show sucessfully message
    if (failLoggerName.count() <= 0) {
        this->m_viewer->setMessageRelaySuccess(5000);
    }
    else {
        QString err;
        Q_FOREACH(QString str, failLoggerName) {
            err = err + str + " ";
        }

        qDebug() << "Fail relaying data source:" << err;
    }
}

void MainWindow::slotRelayTimeout()
{
    m_relayTimeout = true;
    this->m_record->stopRelayingStatus();
    this->m_viewer->setEnableAllView(true);

    QString msg = "Relay timeout: " + QString::number(AppConfig::instance().getRelayTimeout()) + " miliseconds";
    this->m_viewer->relayMessage(msg);

    QStringList failLoggerName;
    bool status = false;
    // Check all logger status is responded
    Q_FOREACH(QString key, relayArray.keys()) {
        if (relayArray.value(key) == RelayStatus::NONE) {
            qDebug() << "NONE";
        }
        else if (relayArray.value(key) == RelayStatus::RELAYOK) {
            failLoggerName << key;
            qDebug() << "RELAYOK";
            Message::logMessage(MessageType::LOG, MessageStatus::SUCCESS, MSG005, "RELAY");
            status= true;
        }
        else if (relayArray.value(key) == RelayStatus::RELAYFAIL) {
            failLoggerName << key;
            qDebug() << "RELAYFAIL";
            Message::logMessage(MessageType::LOG, MessageStatus::FAILED, MSG006, "RELAY");
            status= false;
        }
    }

    // Show errror message
    if (failLoggerName.count() > 0) {
        QString err;
        Q_FOREACH(QString str, failLoggerName) {
            err = err + str + " ";
        }
        qDebug() << "Fail relaying data source:" << err;

        //Active message box
        this->setMinimumSize(1010,724);
        this->resize(1010, 724);
        this->m_viewer->EnableViewMessageBox(status, 5000);
    }    
}

// Logger Node PING received
void MainWindow::slotPingLoggerNodeId(QString nodeNameId)
{
    this->m_viewer->setMessageLoggerNodeId(nodeNameId, QDateTime::currentDateTime());
}

void MainWindow::slotSetLoggerConnected(QList<QString> deviceId)
{
    for (int i = 0; i <deviceId.size(); i++)
    {
        this->m_viewer->setMessageLoggerNodeId(deviceId[i], QDateTime::currentDateTime());
    }
}

void MainWindow::slotRemoveLoggerConnected(QList<QString> deviceId)
{
    for (int i = 0; i <deviceId.size(); i++)
    {
        this->m_viewer->removeLogger(deviceId[i]);
    }
}
