#ifndef CUSTOMGRAPHSMALL_H
#define CUSTOMGRAPHSMALL_H

#include <QWidget>
#include <QTimer>
#include "customgraph.h"

#include "katricontrols_export.h"

enum GraphType {
    CPU,
    Memory,
    Ethernet,
    Disk,
    GPU
};

namespace Ui {
class CustomGraphSmall;
}

class KATRICONTROLS_EXPORT CustomGraphSmall : public QWidget
{
    Q_OBJECT

public:
    explicit CustomGraphSmall(QWidget *parent = nullptr);
    ~CustomGraphSmall();

    void setGraphName(GraphType type, QString format);

    void startTimer(int ms = 0);
    void stopTimer();

    CustomGraph *graph();
    void initLayout(bool isSmall);
    void selectChanged(bool checked);

public Q_SLOTS:
    void realtimeDataSlot();

private:
    Ui::CustomGraphSmall *ui;
    QString m_graphName;
    QString m_graphFormat;
    QTimer dataTimer;
    bool isSelected = false;
};

#endif // CUSTOMGRAPHSMALL_H
