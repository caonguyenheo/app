#include "viewerpanel.h"
#include "ui_viewerpanel.h"
#include <QDebug>
#include <QScrollBar>
#include "message/singlemessage.h"
#include "common/common.h"


ViewerPanel::ViewerPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ViewerPanel)
{
    ui->setupUi(this);
    this->InitControlUI();
    m_isAllChecked = false;
  //  connect(this,SIGNAL(signalChangeStatus(bool)),parent,SLOT(slotRecordAllBtnChange(bool)));
}

void ViewerPanel::InitControlUI()
{
    // Left Layout
    controlViewLidar = new ControlL(this);
    controlViewLidar->setName("Lidar");
    controlViewLidar->setType(Command::Type::LIDAR);
    controlViewLidar->AddRecordButton(NAME_LIDAR);
    controlViewLidar->AddNameLidarView(NAME_LIDAR);

    controlViewGPS = new ControlL(this);
    controlViewGPS->setName("GPS");
    controlViewGPS->setType(Command::Type::GNSS);
    controlViewGPS->setViewColor();
    controlViewGPS->AddRecordButton(NAME_GPS);
    controlViewGPS->AddNameLidarView(NAME_GPS);

    controlViewCamera = new ControlL(this);
    controlViewCamera->setName("Camera");
    controlViewCamera->setType(Command::Type::CAMERA);
    controlViewCamera->setViewColor();
    controlViewCamera->AddRecordButton(NAME_CAMERA);
    controlViewCamera->AddNameLidarView(NAME_CAMERA);

    controlViewCanBus = new ControlL(this);
    controlViewCanBus->setName("CAN BUS");
    controlViewCanBus->setType(Command::Type::CAN);
    controlViewCanBus->setViewColor();
    controlViewCanBus->AddRecordButton(NAME_CANBUS);
    controlViewCanBus->AddNameLidarView(NAME_CANBUS);

    // Set layout
    ui->m_gridLayout->addWidget(controlViewLidar, 0 , 0 , 1 , 1);
    ui->m_gridLayout->addWidget(controlViewGPS, 0 , 1 , 1 , 1);
    ui->m_gridLayout->addWidget(controlViewCamera, 1 , 0 , 1 , 1);
    ui->m_gridLayout->addWidget(controlViewCanBus, 1 , 1 , 1 , 1);
    ui->m_gridLayout->setVerticalSpacing(30);
    ui->m_gridLayout->setHorizontalSpacing(30);
    ui->m_gridLayout->setContentsMargins(50,50,50,50);

    mapControl.insert(Command::Type::LIDAR,  controlViewLidar);
    mapControl.insert(Command::Type::GNSS,   controlViewGPS);
    mapControl.insert(Command::Type::CAMERA, controlViewCamera);
    mapControl.insert(Command::Type::CAN,    controlViewCanBus);

    //Message relay sucessful
    lblicon = new QLabel(this);
    lblmsg = new QLabel(this);

    lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/Success-icon.png) 0 0 0 0 stretch stretch;");
    lblmsg->setText("Data relay successfully");
    lblmsg->setStyleSheet("background-color:transparent;border:none;color : green; font:12px 'Roboto';");

    QPoint point = ui->verticalLayout_2->geometry().topLeft();
    lblicon->setGeometry(point.x()+50, point.y()+25, 14 , 14);
    lblicon->setMaximumSize(QSize(14,14));
    lblmsg->setGeometry(point.x()+70, point.y()+22, 200, 20);
    lblmsg->setMaximumSize(QSize(200,20));

    lblicon->hide();
    lblmsg->hide();

    // Ping message
    lblPingMsg = new QLabel(this);
    lblPingMsg->setText("");
    lblPingMsg->setStyleSheet("background-color:transparent; border:none; color:green; font:12px 'Roboto';");
    lblPingMsg->setGeometry(point.x()+50, point.y()+25, this->rect().width(), this->rect().width());
    lblPingMsg->setWordWrap(true);
    showHidePingMessage(true);

    m_loggerRespond = new QTimer();
    connect(m_loggerRespond, SIGNAL(timeout()), this, SLOT(slotTimerRefreshLoggerList()));
    m_loggerRespond->start(10000);

    ui->scrollArea->setStyleSheet("background-color:#2a2a2a; border:0px;");

    ui->scrollArea->verticalScrollBar()->setStyleSheet("QScrollBar:vertical {"
                                                         "    background: #484848;"
                                                         "    width:10px;"
                                                         "    margin: 40px 0px 30px 0px;"
                                                         "    border-radius: 5px;"
                                                         "}"
                                                         "QScrollBar::handle:vertical {"
                                                           "    background: #060606;"
                                                           "    border-radius: 5px;"
                                                           "    min-height: 0px;"
                                                         "}"
                                                         "QScrollBar::add-line:vertical {"
                                                         "    background: #060606;"
                                                         "    height: 0px;"
                                                         "    subcontrol-position: bottom;"
                                                         "    subcontrol-origin: margin;"
                                                         "}"
                                                         "QScrollBar::sub-line:vertical {"
                                                         "    background: #060606;"
                                                         "    height: 0px;"
                                                         "    subcontrol-position: top;"
                                                         "    subcontrol-origin: margin;"
                                                         "}"
                                                        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {"
                                                           "background: none;"
                                                        "}"
                                                       );
   ui->Message_box->setHidden(true);
}

ViewerPanel::~ViewerPanel()
{
    if (lblPingMsg) {
	    delete lblPingMsg;
	    lblPingMsg = nullptr;
	}
    delete ui;
}

void ViewerPanel::EnableViewMessageBox(bool status, uint16_t time)
{
    ui->Message_box->setHidden(false);
    SingleMessage *msg = new SingleMessage(ui->scrollAreaWidgetContents);
    if (status==true){
        msg->setTextMessage(SUCCESS);
        msg->setStateIcon(SUCCESS);
        ui->message->addWidget(msg);
    }
    else if (status==false) {
        msg->setTextMessage(ERROR);
        msg->setStateIcon(ERROR);
        ui->message->addWidget(msg);
    }

    //After Test delete this line
    QTimer::singleShot(time, this, SLOT(HideViewMessageBox()));
}

void ViewerPanel::HideViewMessageBox()
{
    ui->Message_box->setHidden(true);
    emit signalResize(true);
}

void ViewerPanel::recordAllActivation(bool active)
{
    controlViewLidar->activeAllButton(active);
    controlViewGPS->activeAllButton(active);
    controlViewCamera->activeAllButton(active);
    controlViewCanBus->activeAllButton(active);
}

void ViewerPanel::viewChartConnection(const QObject *receiver, const char *member)
{
    controlViewLidar->setViewChartAction("View chart", receiver, member);
    controlViewGPS->setViewChartAction("View chart", receiver, member);
    controlViewCanBus->setViewChartAction("View chart", receiver, member);
}

void ViewerPanel::viewCameraConnection(const QObject *receiver, const char *member)
{
    controlViewCamera->setViewCameraAction("Camera dashboard", receiver, member);
}

void ViewerPanel::slotUpdateStatus()
{
    bool isChange = controlViewLidar->getStatus();
    isChange |= controlViewCamera->getStatus();
    isChange |= controlViewCanBus->getStatus();
    isChange |= controlViewGPS->getStatus();
    if((!isChange && controlViewLidar->getStatus() != m_isAllChecked) || (isChange && !m_isAllChecked)) {
        emit signalChangeStatus(!m_isAllChecked);
    }
}
//deviceType, deviceId, recordStatus, isNewDS
void ViewerPanel::slotGetSendCommandDevice(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS)
{
    // ToDo set message on View Panel
    qDebug()<<"deviceType : "<<uint8_t(deviceType) <<"deviceId : "<<deviceId<<"deviceId : "<<recordStatus<<"isNewDS : "<<isNewDS;
    //SingleMessage *msg2 = new SingleMessage(ui->scrollAreaWidgetContents);
    //msg2->setTextMessage(QString("%1, %2 Send Command Success").arg(uint8_t(deviceType)).arg(deviceId));
    //ui->message->addWidget(msg2);
}

void ViewerPanel::setStatus(bool status)
{
    m_isAllChecked = status;
//    controlViewLidar->setStatus(status);
//    controlViewGPS->setStatus(status);
//    controlViewCamera->setStatus(status);
//    controlViewCanBus->setStatus(status);
}

void ViewerPanel::setEnableAllView(bool status)
{
    controlViewLidar->setEnabled(status);
    controlViewGPS->setEnabled(status);
    controlViewCamera->setEnabled(status);
    controlViewCanBus->setEnabled(status);
}

void ViewerPanel::setMessageRelaySuccess(uint16_t time)
{
    showHidePingMessage(false);
    lblicon->show();
    lblmsg->show();

    if(time > 0) {
        QTimer::singleShot(time, this, SLOT(hideMessageRelayStatus()));
    }
}

void ViewerPanel::hideMessageRelayStatus()
{
    showHidePingMessage(false);
    lblicon->hide();
    lblmsg->hide();
    showHidePingMessage(true);
}

void ViewerPanel::slotTimerRefreshLoggerList()
{
    m_mutexLogger.lock();
    //for (int i = pingLoggerId.count()-1; i >= 0; i--)
    Q_FOREACH(QString key, m_loggerNodeName.keys())
    {
        QDateTime dt = QDateTime::currentDateTime();
        qint64 delayInterval = m_loggerNodeName[key].msecsTo(dt);
        qDebug() << "delayInterval:" << key << delayInterval;

        //Remove logger node name in case of no longer respond
        if (delayInterval > AppConfig::instance().getLoggerRespondTimeout()) {
            m_loggerNodeName.remove(key);
        }
    }
    lblPingMsg->setText(QString("Logger available: ") + m_loggerNodeName.keys().join(", "));
    showHidePingMessage(true);
    m_mutexLogger.unlock();
}

void ViewerPanel::setMessageLoggerNodeId(QString nodeNameId, QDateTime dt)
{
    m_mutexLogger.lock();
    if (m_loggerNodeName.contains(nodeNameId) == false) {
        // If not exist => add into QMap
        m_loggerNodeName.insert(nodeNameId, dt);
        lblPingMsg->setText(QString("Logger available: ") + m_loggerNodeName.keys().join(", "));
    }
    else {
        // If existed in QMap, update timestamp
        m_loggerNodeName[nodeNameId] = dt;
    }
	showHidePingMessage(true);
    m_mutexLogger.unlock();
}

void ViewerPanel::LidarLiveStreamConnection(const QObject *receiver, const char *member)
{
    controlViewLidar->setLidarLiveStreamAction("Lidar live stream", receiver, member);
}

void ViewerPanel::showHidePingMessage(bool status)
{
    if(status){
        lblicon->hide();
        lblmsg->hide();
    }
    lblPingMsg->setVisible(status);
}

void ViewerPanel::relayMessage(QString msg)
{
    lblPingMsg->setText(msg);
    showHidePingMessage(true);
}

QStringList ViewerPanel::getLoggerNodeNameList()
{
    return m_loggerNodeName.keys();
}

bool ViewerPanel::isRecording()
{
    bool ret = controlViewLidar->isRecording()  ||
               controlViewGPS->isRecording()    ||
               controlViewCamera->isRecording() ||
               controlViewCanBus->isRecording();
    return ret;
}

void ViewerPanel::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if (lblPingMsg != nullptr) {
        QPoint point = ui->verticalLayout_2->geometry().topLeft();
        lblPingMsg->setGeometry(point.x()+50, point.y()+10, this->rect().width()-100, 30);
    }
}
