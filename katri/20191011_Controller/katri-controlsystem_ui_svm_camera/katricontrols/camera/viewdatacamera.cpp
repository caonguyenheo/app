#include <QDebug>
#include "viewdatacamera.h"
#include "ui_viewdatacamera.h"

viewdatacamera::viewdatacamera(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::viewdatacamera)
{
    ui->setupUi(this);
}

viewdatacamera::~viewdatacamera()
{
    delete ui;
}
void viewdatacamera::initDataUI(QMap<int, QString> mapDataSource)
{
    this->mapDataIDCameraIPAddress = mapDataSource;
    int indexCol = 0;
    int indexRow = 0;
    for(int i = 0; i< this->mapDataIDCameraIPAddress.size(); i++){
        int camId = mapDataIDCameraIPAddress.keys().at(i);
        QString ipAddress = mapDataIDCameraIPAddress.values().at(i);
        m_view = new viewerperformance(this, camId);
        m_view->setname("I-VSB #" + QString::number(camId+1));
        video = new Video(this);
        video->setObjectName(QString::number(camId));
        setValueIP(m_view, true, ipAddress);
        m_view->addwidgetleft(video);
        if(i % 2 == 0){
            ui->m_gridLayout->addWidget(m_view, indexRow , 0,  1 , 1);
            indexRow++;
        }
        else{
            ui->m_gridLayout->addWidget(m_view, indexCol , 1, 1 , 1);
            indexCol++;
        }
        listViewerPerformance.push_back(m_view);
        video->setbackground();
    }
}

void viewdatacamera::setValueIP(viewerperformance *m_view, bool isDefault, QString value)
{
    Q_UNUSED(isDefault);
    m_view->setIP(value);
}

void viewdatacamera::setactive(bool status)
{
//    m_view->setActivate(status);
//    m_view1->setActivate(status);
//    m_view2->setActivate(status);
//    m_view3->setActivate(status);
    for(int i = 0; i < listViewerPerformance.size(); i++)
    {
        listViewerPerformance.at(i)->setActivate(status);
    }
}

QList<viewerperformance*> viewdatacamera::getVideoViewerList()
{
    return listViewerPerformance;
}

void viewdatacamera::sendCommandConnection(const QObject *receiver, const char *member)
{
    Q_FOREACH(viewerperformance* view, listViewerPerformance) {
        view->sendCommandConnection(receiver, member);
    }
}
/**
 * @brief viewdatacamera::setDataCamara
 * @param deviceType
 * @param deviceId
 * @param data
 * note: set data for camera device
 */
void viewdatacamera::setDataCamara(int deviceId, QByteArray &data)
{
    for(int i = 0; i<this->listViewerPerformance.size(); i++){
        viewerperformance* view = this->listViewerPerformance.at(i);
        if(view->getDeviceId() == deviceId) {
            qDebug()<<"[viewdatacamera---Data Camera---]:device id: "<< deviceId;
            Video *videoObj = dynamic_cast<Video*>(view->getVideoObject());
            if (videoObj) {
                videoObj->setDataCameraView(data);
            }
            break;
        }
    }
}
