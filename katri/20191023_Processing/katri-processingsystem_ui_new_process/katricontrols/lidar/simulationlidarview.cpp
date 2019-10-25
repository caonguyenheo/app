#include "simulationlidarview.h"
#include "ui_simulationlidarview.h"
#include "common.h"
#include<bits/stdc++.h>
#include <QDebug>
//#include <ThreadRenderLidar/threadrenderlidarleft.h>
//#include <ThreadRenderLidar/threadrenderlidarright.h>
//#include <ThreadRenderLidar/threadrenderlidarbottom.h>

#define VIEWER_NUM 3

LidarGroupView::LidarGroupView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SimulationLidarView)
{
    // Initialization
    ui->setupUi(this);

    // Create 3 lidar views
    for (int i = 0; i < VIEWER_NUM; i++) {
        LidarView* lidarWidget = new LidarView(this);
        m_viewList.insert(i, lidarWidget);
    }
    setLidarVerticalLayout();
}

LidarGroupView::~LidarGroupView()
{
    delete ui;
}

void LidarGroupView::setViewName(QString &name)
{
    if (name.compare("") != 0) {
        ui->simulationLabel->setText(name);
        this->setStyleMainScreenLayout();
    }
}

void LidarGroupView::setStyleMainScreenLayout()
{
    ui->simulationLabel->setStyleSheet( "color: white;"
                                    "font: 14px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:7px;"
                                    );
    ui->simulationLabel->setFixedHeight(29);
}

void LidarGroupView::setViewColor(QColor &color)
{
    ui->simulationPanel->setStyleSheet( "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: rgb("
                                        +QString::number(color.red()) + ","
                                        +QString::number(color.green()) + ","
                                        +QString::number(color.blue()) + ");"
                                      );
}

void LidarGroupView::setStyleFullScreenLayout()
{
    ui->simulationLabel->setStyleSheet( "color: white;"
                                    "font: 32px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:5px;"
                                    );
    ui->simulationLabel->setFixedHeight(55);
}

void LidarGroupView::appendStyleSheet(QString stylesheet)
{
    ui->simulationPanel->setStyleSheet(ui->simulationPanel->styleSheet().append(stylesheet));
}

void LidarGroupView::setImageSimulation(bool display)
{
    if(lbicon == nullptr) {
        lbicon = new QLabel(this);
        movie = new QMovie(":/img/icon/Loading_iCon.gif");
        lbicon->setAttribute( Qt::WA_TranslucentBackground, true );
        lbicon->setMovie(movie);
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28 , 28);
        lbicon->setMaximumSize(QSize(28,28));
    }
    if(lbicon != nullptr) {
        if(display) {
            lbicon->show();
            movie->start();
        }
        else {
            lbicon->hide();
            movie->stop();
        }
    }
}

void LidarGroupView::setLidarHorizontalLayout()
{
    ui->verticalLayout->removeWidget(m_viewList[0]);
    ui->verticalLayout->removeWidget(m_viewList[1]);
    ui->verticalLayout->removeWidget(m_viewList[2]);
    hLayout1 = new QHBoxLayout();
    hLayout1->addWidget(m_viewList[0]);
    hLayout1->addWidget(m_viewList[1]);
    hLayout1->addWidget(m_viewList[2]);
    ui->verticalLayout->addItem(hLayout1);
    hLayout1->setSpacing(20);
    ui->verticalLayout->setContentsMargins(20,92,20,92);
}

void LidarGroupView::setLidarVerticalLayout()
{
    ui->verticalLayout->removeItem(hLayout1);
    ui->verticalLayout->addWidget(m_viewList[0]);
    ui->verticalLayout->addWidget(m_viewList[1]);
    ui->verticalLayout->addWidget(m_viewList[2]);
    ui->verticalLayout->setSpacing(20);
    ui->verticalLayout->setContentsMargins(46,20,46,22);
}

void LidarGroupView::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if(lbicon != nullptr) {
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28 ,28);
    }
}

void LidarGroupView::createConnectLidarLeft()
{
//    ThreadRenderLidarLeft *LeftLidar  = new ThreadRenderLidarLeft;
//    LeftLidar->moveToThread(&ThreadLoadDataLeft);
//    connect(&ThreadLoadDataLeft, &QThread::finished, LeftLidar, &QObject::deleteLater);
//    connect(this, &LidarGroupView::getLidarDataLeft, LeftLidar, &ThreadRenderLidarLeft::loadLidarData);
//    connect(this, &LidarGroupView::getTimestampLeft, LeftLidar, &ThreadRenderLidarLeft::getTimestamp);
//    connect(this, &LidarGroupView::setDataForRenderLeftView, LeftLidar, &ThreadRenderLidarLeft::setDataForRender);
//    connect(this, &LidarGroupView::pauseEvent, LeftLidar, &ThreadRenderLidarLeft::pause);
//    connect(this, &LidarGroupView::playEvent, LeftLidar, &ThreadRenderLidarLeft::play);
//    connect(this, &LidarGroupView::executeJumpPosition, LeftLidar, &ThreadRenderLidarLeft::jumpPosition);
//    connect(this, &LidarGroupView::BrowseEvent, LeftLidar, &ThreadRenderLidarLeft::BrowseNewData);
//    ThreadLoadDataLeft.start();
}
