#include "customgraphsmall.h"
#include "ui_customgraphsmall.h"

CustomGraphSmall::CustomGraphSmall(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CustomGraphSmall)
{
    ui->setupUi(this);

    ui->graph->setBackgroundProperties(1, QColor(Qt::blue), QColor(Qt::white));
    ui->graph->setLineProperties(1, QColor(Qt::blue), QColor(Qt::cyan));

    ui->graphFormat->setWordWrap(true);
    //ui->graphFormat->setAlignment(Qt::AlignCenter);

    // Setup timer
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
}

CustomGraphSmall::~CustomGraphSmall()
{
    delete ui;
}

void CustomGraphSmall::setGraphName(GraphType type, QString format)
{
    double val;
    switch (type) {
    case GraphType::CPU:
        m_graphName = "CPU";
        val = 8;
        break;
    case GraphType::Memory:
        m_graphName = "Memory";
        val = 82;
        break;
    case GraphType::Ethernet:
        m_graphName = "Ethernet";
        val = 12.5;
        break;
    case GraphType::Disk:
        m_graphName = "Disk 0 (C:D)";
        val = 25;
        break;
    case GraphType::GPU:
        m_graphName = "GPU";
        val = 2;
        break;
    }
    m_graphFormat = format;

    ui->graphFormat->setText(m_graphName + "\n" + m_graphFormat);
    ui->graph->setFakeValue(val);
}

void CustomGraphSmall::startTimer(int ms)
{
    if (ms >= 0) {
        // Interval 0 means to refresh as fast as possible
        dataTimer.setInterval(ms);
        dataTimer.start();
    }
}

void CustomGraphSmall::stopTimer()
{
    dataTimer.stop();
}

CustomGraph *CustomGraphSmall::graph()
{
    return ui->graph;
}

void CustomGraphSmall::realtimeDataSlot()
{
    double value = ui->graph->realtimeDataSlot();
    Q_UNUSED(value);
    //ui->graphFormat->setText(m_graphName + "\n" + m_graphFormat.arg(value,0,'g',2).arg(2.27));
}

void CustomGraphSmall::initLayout(bool isSmall)
{
    if (!isSmall) {
        ui->graphFormat->setStyleSheet("color:white;");

        this->setFixedSize(260,80);

        ui->horizontalLayout_3->setContentsMargins(30,0,0,0);
        ui->horizontalLayout_3->setSpacing(20);

        ui->widgetLayout->setFixedSize(100,60);

        ui->graph->setFixedSize(100,61);
    }
}

void CustomGraphSmall::selectChanged(bool checked)
{
    if (isSelected != checked) {
        isSelected = checked;
    }
    if (isSelected) {
        ui->widgetBack->setStyleSheet("background-color:#000000;");
    }
    else {
        ui->widgetBack->setStyleSheet("");
    }
}
