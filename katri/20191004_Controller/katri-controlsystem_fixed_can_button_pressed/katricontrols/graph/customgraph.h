#ifndef CUSTOMGRAPH_H
#define CUSTOMGRAPH_H

#include <QWidget>
#include <QTimer>
#include "qcustomplot.h"

#include "katricontrols_export.h"

namespace Ui {
class CustomGraph;
}

class KATRICONTROLS_EXPORT CustomGraph : public QWidget
{
    Q_OBJECT

public:
    explicit CustomGraph(QWidget *parent = 0);
    ~CustomGraph();

    void startTimer(int ms = 0);
    void stopTimer();
    void setBackgroundProperties(int width, QColor lineColor, QColor bgColor);
    void setLineProperties(int width, QColor lineColor, QColor fillColor);

    void setFakeValue(double val);
    double getFakeValue();

public Q_SLOTS:
    double realtimeDataSlot();

private:
    Ui::CustomGraph *ui;
    QTimer dataTimer;
    double fakeVal;
};

#endif // CUSTOMGRAPH_H
