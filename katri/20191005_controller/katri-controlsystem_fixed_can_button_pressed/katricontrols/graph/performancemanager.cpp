#include "performancemanager.h"
#include "ui_performancemanager.h"

PerformanceManager::PerformanceManager(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PerformanceManager)
{
    ui->setupUi(this);
    ui->backPanel->setDoublePanel();
    this->setStyleSheet( "background-color:rgb(39,39,39);" );

    ui->mainGraph->setBackgroundProperties(1, QColor("#73a74d"), QColor("#000000"));
    ui->mainGraph->setLineProperties(1, QColor("#6cca6b"), QColor("#10150d"));
    ui->mainGraph->setFakeValue(25);
    //ui->mainGraph->startTimer(1000);

    // Setup timer
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(1000);
}

PerformanceManager::~PerformanceManager()
{
    delete ui;
}

void PerformanceManager::realtimeDataSlot()
{
    ui->leftPanel->realtimeDataSlot();
    QThread::msleep(10);
    ui->mainGraph->realtimeDataSlot();
}

void PerformanceManager::backButtonConnection(const QObject *receiver, const char *member)
{
    connect(ui->backPanel->backButton(), SIGNAL(released()), receiver, member);
}
