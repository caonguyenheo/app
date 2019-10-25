#include "customgraph.h"
#include "ui_customgraph.h"

CustomGraph::CustomGraph(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CustomGraph)
{
    ui->setupUi(this);

    // Add graph
    ui->customPlot->addGraph();
    ui->customPlot->axisRect()->setupFullAxesBox();
    ui->customPlot->yAxis->setRange(0, 100);
    //ui->customPlot->plotLayout()->setColumnSpacing(10);

    // Remove grid
    ui->customPlot->xAxis->grid()->setVisible(false);
    ui->customPlot->yAxis->grid()->setVisible(false);

    // Remove tick label
    ui->customPlot->xAxis->setTickLabels(false);
    ui->customPlot->xAxis2->setTickLabels(false);
    ui->customPlot->yAxis->setTickLabels(false);
    ui->customPlot->yAxis2->setTickLabels(false);

    // Remove Ticks
    ui->customPlot->xAxis->setTicks(false);
    ui->customPlot->xAxis2->setTicks(false);
    ui->customPlot->yAxis->setTicks(false);
    ui->customPlot->yAxis2->setTicks(false);

    ui->customPlot->xAxis->setTickLabelPadding(10);
    ui->customPlot->xAxis->setPadding(0);
    ui->customPlot->xAxis2->setPadding(0);
    ui->customPlot->yAxis->setPadding(0);
    ui->customPlot->yAxis2->setPadding(0);

    //ui->customPlot->layoutElementAt(QPointF(0,0))->setMinimumMargins(QMargins(0,0,0,0));

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));

    // Setup timer
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
}

CustomGraph::~CustomGraph()
{
}

void CustomGraph::startTimer(int ms)
{
    if (ms >= 0) {
        // Interval 0 means to refresh as fast as possible
        dataTimer.setInterval(ms);
        dataTimer.start();
    }
}

void CustomGraph::stopTimer()
{
    dataTimer.stop();
}

double CustomGraph::realtimeDataSlot()
{
    double ret = 0.0;
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.002) // at most add point every 2 ms
    {
        // add data to lines:
        ret = getFakeValue();
        ui->customPlot->graph(0)->addData(key, ret);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    ui->customPlot->xAxis->setRange(key, 60, Qt::AlignRight);
    ui->customPlot->replot();


    // calculate frames per second:
    static int frameCount;
    ++frameCount;
    return ret;
}

void CustomGraph::setBackgroundProperties(int width, QColor lineColor, QColor bgColor)
{
    // Sert border width and color
    QPen pen(lineColor);
    pen.setWidth(width);
    ui->customPlot->xAxis->setBasePen(pen);
    ui->customPlot->yAxis->setBasePen(pen);
    ui->customPlot->xAxis2->setBasePen(pen);
    ui->customPlot->yAxis2->setBasePen(pen);

    // Set background color
    ui->customPlot->setBackground(QBrush(bgColor));
}

void CustomGraph::setLineProperties(int width, QColor lineColor, QColor fillColor)
{
    // Set line width and color
    QPen pen(lineColor);
    pen.setWidth(width);
    ui->customPlot->graph(0)->setPen(pen);

    // Set fill color
    ui->customPlot->graph(0)->setBrush(QBrush(fillColor));
}

void CustomGraph::setFakeValue(double val)
{
    fakeVal = val;
}

double CustomGraph::getFakeValue()
{
    return fakeVal;
}
// Need implement margins
