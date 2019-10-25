#include "navigationmanagersmall.h"
#include "ui_navigationmanagersmall.h"

NavigationManagerSmall::NavigationManagerSmall(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NavigationManagerSmall)
{
    ui->setupUi(this);
    this->setMaximumSize(250, 260);

    //this->setStyleSheet("background-color:rgb(47,47,47);font-size:14px;font-family:'Roboto';");
    // Camera view
    Init_cameraview();
    initDefaultValue();
}

NavigationManagerSmall::~NavigationManagerSmall()
{
    delete ui;
}

void NavigationManagerSmall::Init_cameraview()
{
     //this->setStyleSheet("font-size:10px;font-family:'Roboto';");
     this->setContentsMargins(0,0,0,0);
     ui->lbInfo->setStyleSheet("font:13px 'Roboto';font-weight:bold; color:#272727;");

}
void NavigationManagerSmall::initDefaultValue()
{
    ui->cpuSmallGraph->setGraphName(GraphType::CPU, "");

    ui->cpuSmallGraph->graph()->setBackgroundProperties(1, QColor("#277daf"), QColor("#ffffff"));
    ui->cpuSmallGraph->graph()->setLineProperties(1, QColor("#277daf"), QColor("#f1f6fa"));

    //ui->memSmallGraph->setGraphName(GraphType::Memory, "%1/%2 GB (%3%)");
    ui->memSmallGraph->setGraphName(GraphType::Memory, "");
    ui->memSmallGraph->graph()->setBackgroundProperties(1, QColor("#8b12ae"), QColor("#ffffff"));
    ui->memSmallGraph->graph()->setLineProperties(1, QColor("#8b12ae"), QColor("#f4f2f4"));

    //ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "Ethernet %1\nS:%2 R:%3 Kbps");
    ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "");
    ui->ethSmallGraph->graph()->setBackgroundProperties(1, QColor("#a36c3a"), QColor("#ffffff"));
    ui->ethSmallGraph->graph()->setLineProperties(1, QColor("#a36c3a"), QColor("#fcf3eb"));

    //ui->dskSmallGraph->setGraphName(GraphType::Disk, "%1%");
    ui->dskSmallGraph->setGraphName(GraphType::Disk, "");
    ui->dskSmallGraph->graph()->setBackgroundProperties(1, QColor("#73a74d"), QColor("#ffffff"));
    ui->dskSmallGraph->graph()->setLineProperties(1, QColor("#73a74d"), QColor("#eff7e9"));

    //ui->gpuSmallGraph->setGraphName(GraphType::GPU, "Intel(R) UHD Grap...\n%1%");
    ui->gpuSmallGraph->setGraphName(GraphType::GPU, "");
    ui->gpuSmallGraph->graph()->setBackgroundProperties(1, QColor("#277daf"), QColor("#ffffff"));
    ui->gpuSmallGraph->graph()->setLineProperties(1, QColor("#277daf"), QColor("#f1f6fa"));

}
void NavigationManagerSmall::loadValue()
{
    //ui->cpuSmallGraph->setGraphName(GraphType::CPU, "%1% %2 Ghz");
    ui->cpuSmallGraph->setGraphName(GraphType::CPU, "8% 2.27 Ghz");

    //ui->memSmallGraph->setGraphName(GraphType::Memory, "%1/%2 GB (%3%)");
    ui->memSmallGraph->setGraphName(GraphType::Memory, "6.4/7.8 GB (82%)");
    //ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "Ethernet %1\nS:%2 R:%3 Kbps");
    ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "Ethernet 2\nS:1024 R:0 Kbps");

    //ui->dskSmallGraph->setGraphName(GraphType::Disk, "%1%");
    ui->dskSmallGraph->setGraphName(GraphType::Disk, "25%");

    //ui->gpuSmallGraph->setGraphName(GraphType::GPU, "Intel(R) UHD Grap...\n%1%");
    ui->gpuSmallGraph->setGraphName(GraphType::GPU, "Intel(R) UHD Grap...\n 2%");

}

void NavigationManagerSmall::realtimeDataSlot()
{
    ui->cpuSmallGraph->realtimeDataSlot();
    QThread::msleep(10);
    ui->memSmallGraph->realtimeDataSlot();
    QThread::msleep(10);
    ui->ethSmallGraph->realtimeDataSlot();
    QThread::msleep(10);
    ui->dskSmallGraph->realtimeDataSlot();
    QThread::msleep(10);
    ui->gpuSmallGraph->realtimeDataSlot();
}
