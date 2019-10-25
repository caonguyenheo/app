#include "navigationmanager.h"
#include "ui_navigationmanager.h"

NavigationManager::NavigationManager(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NavigationManager)
{
    ui->setupUi(this);

    this->setStyleSheet("background-color: rgb(47, 47, 47);font-size:14px;font-family:'Roboto';");

    ui->cpuSmallGraph->initLayout(false);
    //ui->cpuSmallGraph->setGraphName(GraphType::CPU, "%1% %2 Ghz");
    ui->cpuSmallGraph->setGraphName(GraphType::CPU, "8% 2.27 Ghz");
    ui->cpuSmallGraph->graph()->setBackgroundProperties(1, QColor("#277daf"), QColor("#ffffff"));
    ui->cpuSmallGraph->graph()->setLineProperties(1, QColor("#277daf"), QColor("#f1f6fa"));
//    ui->cpuSmallGraph->setStyleSheet("font-size:30px;");

    ui->memSmallGraph->initLayout(false);
    //ui->memSmallGraph->setGraphName(GraphType::Memory, "%1/%2 GB (%3%)");
    ui->memSmallGraph->setGraphName(GraphType::Memory, "6.4/7.8 GB (82%)");
    ui->memSmallGraph->graph()->setBackgroundProperties(1, QColor("#8b12ae"), QColor("#ffffff"));
    ui->memSmallGraph->graph()->setLineProperties(1, QColor("#8b12ae"), QColor("#f4f2f4"));

    ui->ethSmallGraph->initLayout(false);
    //ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "Ethernet %1\nS:%2 R:%3 Kbps");
    ui->ethSmallGraph->setGraphName(GraphType::Ethernet, "Ethernet 2\nS:0 R:0 Kbps");
    ui->ethSmallGraph->graph()->setBackgroundProperties(1, QColor("#a36c3a"), QColor("#ffffff"));
    ui->ethSmallGraph->graph()->setLineProperties(1, QColor("#a36c3a"), QColor("#fcf3eb"));

    ui->dskSmallGraph->initLayout(false);
    //ui->dskSmallGraph->setGraphName(GraphType::Disk, "%1%");
    ui->dskSmallGraph->setGraphName(GraphType::Disk, "25%");
    ui->dskSmallGraph->graph()->setBackgroundProperties(1, QColor("#73a74d"), QColor("#ffffff"));
    ui->dskSmallGraph->graph()->setLineProperties(1, QColor("#73a74d"), QColor("#eff7e9"));
    ui->dskSmallGraph->selectChanged(true);

    ui->gpuSmallGraph->initLayout(false);
    //ui->gpuSmallGraph->setGraphName(GraphType::GPU, "Intel(R) UHD\nGrap...%1%");
    ui->gpuSmallGraph->setGraphName(GraphType::GPU, "Intel(R) UHD\nGrap...2%");
    ui->gpuSmallGraph->graph()->setBackgroundProperties(1, QColor("#277daf"), QColor("#ffffff"));
    ui->gpuSmallGraph->graph()->setLineProperties(1, QColor("#277daf"), QColor("#f1f6fa"));
    //set Style
    setLayoutUI(ui->cpuSmallGraph);
    setLayoutUI(ui->memSmallGraph);
    setLayoutUI(ui->ethSmallGraph);
    setLayoutUI(ui->dskSmallGraph);
    setLayoutUI(ui->gpuSmallGraph);
}

NavigationManager::~NavigationManager()
{
    delete ui;
}

void NavigationManager::realtimeDataSlot()
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

void NavigationManager::setLayoutUI(QWidget *graph)
{
    if (QFontDatabase::addApplicationFont(":/fonts/Roboto/Roboto-Regular.ttf") < 0)
            qDebug()<<"Can't load this font";
    else{
//        qDebug()<<"Loaded font";
        graph->setStyleSheet("font:14px 'Roboto';");
    }

}
