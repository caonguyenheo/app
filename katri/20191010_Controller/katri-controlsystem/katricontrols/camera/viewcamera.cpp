#include "viewcamera.h"
#include "ui_viewcamera.h"
#include <QDebug>
#include <QScrollBar>

viewcamera::viewcamera(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::viewcamera)
{
    ui->setupUi(this);
    this->InitControlUI();


    ui->pb_record->setCheckable(true);
//    ui->pb_record->setStyleSheet("QPushButton{background-color:transparent;background-image: url(:/img/icon/RecNonselect-btn.png); border:none;font-size:1px;}"
//                                 "QPushButton:checked{background-image: url(:/img/icon/RecSelected-btn.png);border:none;font-size:1px;}"
//                                );
    QMap<int, QString> mapDataInitview;
    mapDataInitview.insert(0, "");
    mapDataInitview.insert(1, "");
    mapDataInitview.insert(2, "");
    mapDataInitview.insert(3, "");
    ui->viewhd->initDataUI(true, mapDataInitview);
    mapDataInitview.clear();
    mapDataInitview.insert(4, "");
    mapDataInitview.insert(5, "");
    mapDataInitview.insert(6, "");
    mapDataInitview.insert(7, "");
    ui->viewfhd->initDataUI(true, mapDataInitview);
    setConnect();
}

viewcamera::~viewcamera()
{
    delete ui;
}

void viewcamera::InitControlUI()
{  
    this->setStyleSheet("font-family:Roboto");
    ui->label_record->setFont(QFont(":/fonts/Roboto/Roboto-Bold.ttf"));
    ui->label_record->setStyleSheet("font:14px 'Roboto'; color:white; font-weight:bold;margin-bottom:7px;");
    //this->QScrollBar
    ui->scrollArea->setStyleSheet("background-color:#272727;border:0px;");

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

}

void viewcamera::updateLabel(QString Text)
{
    ui->label_record->setText(Text);
}

void viewcamera::backButtonConnection(const QObject *receiver, const char *member)
{
    connect(ui->Backpanel->backButton(), SIGNAL(released()), receiver, member);
}

void viewcamera::recordAllConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_record, SIGNAL(clicked(bool)), this, SLOT(slotRecordAllClicked(bool)));
    connect(ui->pb_record, SIGNAL(clicked(bool)), receiver, member);
}

void viewcamera::recordAllActivation(bool status)
{
    ui->viewhd->setactive(status);
    ui->viewfhd->setactive(status);
}

void viewcamera::setStatus(bool status)
{
    ui->pb_record->setChecked(status);
    ui->pb_record->clicked(status);
}
void viewcamera::slotSetActiveChecked(bool status)
{
    if(status == false){
        ui->pb_record->setChecked(status);
    }
    else{
        bool isState = true;
        for(int i = 0 ; i < ui->viewhd->getVideoViewerList().size(); i++)
        {
            viewerperformance *vPerform = ui->viewhd->getVideoViewerList().at(i);
            RecordButton * recordBtn = vPerform->getRecordButton();
            bool isStatus = recordBtn->getStatusButton();
            if(isStatus == false){
                isState = false;
                break;
            }
        }
        if(isState == true){
            for(int i = 0 ; i < ui->viewfhd->getVideoViewerList().size(); i++)
            {
                viewerperformance *vPerform = ui->viewfhd->getVideoViewerList().at(i);
                RecordButton * recordBtn = vPerform->getRecordButton();
                bool isStatus = recordBtn->getStatusButton();
                if(isStatus == false){
                    isState = false;
                    break;
                }
            }
        }
        ui->pb_record->setChecked(isState);

    }
}

void viewcamera::slotReturnSignalOut(bool status)
{
    QPushButton *btSender = dynamic_cast<QPushButton*>(sender());
    RecordButton* pRecordButton = dynamic_cast<RecordButton*>(btSender->parent());
    viewerperformance *vPerform = dynamic_cast<viewerperformance*>(pRecordButton->parent());
    int deviceID = vPerform->getDeviceId();
    emit signalClickedInside(status, deviceID);
}

void viewcamera::setViewStatus(uint id,bool status)
{
    for(int i = 0 ; i < ui->viewhd->getVideoViewerList().size(); i++)
    {
        viewerperformance *vPerform = ui->viewhd->getVideoViewerList().at(i);
        if(vPerform->getDeviceId() == id)
        {
            vPerform->setChecked(status);
            return;
        }
    }
    for(int i = 0 ; i < ui->viewfhd->getVideoViewerList().size(); i++)
    {
        viewerperformance *vPerform = ui->viewfhd->getVideoViewerList().at(i);
        if(vPerform->getDeviceId() == id)
        {
            vPerform->setChecked(status);
            return;
        }
    }
}

void viewcamera::setConnect()
{
    for(int i = 0 ; i < ui->viewhd->getVideoViewerList().size(); i++)
    {
        viewerperformance *vPerform = ui->viewhd->getVideoViewerList().at(i);
        connect(vPerform->getRecordButton()->getRecordButton(), SIGNAL(clicked(bool)), this, SLOT(slotReturnSignalOut(bool)));
    }
    for(int i = 0 ; i < ui->viewfhd->getVideoViewerList().size(); i++)
    {
        viewerperformance *vPerform = ui->viewfhd->getVideoViewerList().at(i);
        connect(vPerform->getRecordButton()->getRecordButton(), SIGNAL(clicked(bool)), this, SLOT(slotReturnSignalOut(bool)));

    }
}

void viewcamera::sendCommandConnection(const QObject *receiver, const char *member)
{
    ui->viewhd->sendCommandConnection(receiver, member);
    ui->viewfhd->sendCommandConnection(receiver, member);
}
/**
 * @brief viewcamera::setDataViewHD
 * @param deviceType
 * @param deviceId
 * @param data
 * content set data image camera
 */
void viewcamera::setDataViewHD(Command::Type deviceType, int deviceId, QByteArray &data)
{
    Q_UNUSED(deviceType);
    ui->viewhd->setDataCamara(deviceId, data);
}
/**
 * @brief viewcamera::setDataViewFullHD
 * @param deviceType
 * @param deviceId
 * @param data
 * content set data image camera
 */
void viewcamera::setDataViewFullHD(Command::Type deviceType, int deviceId, QByteArray &data)
{
    Q_UNUSED(deviceType);
    ui->viewfhd->setDataCamara(deviceId, data);
}
void viewcamera::slotRecordAllClicked(bool status)
{
    recordAllActivation(status);

    if (status == true) {
        updateLabel("STOP RECORD ALL");
    }
    else {
        updateLabel("RECORD ALL");
    }
}

bool viewcamera::getStatus()
{
    return ui->pb_record->isChecked();
}

QList<RecordButton*> viewcamera::getListRecordButton()
{
    QList<RecordButton*> listRecordButton;
    for(int i = 0 ; i < ui->viewhd->getVideoViewerList().size(); i++)
    {
        listRecordButton.append(ui->viewhd->getVideoViewerList().at(i)->getRecordButton());
    }
    for(int i = 0 ; i < ui->viewfhd->getVideoViewerList().size(); i++)
    {
        listRecordButton.append(ui->viewfhd->getVideoViewerList().at(i)->getRecordButton());
    }
    return  listRecordButton;
}
