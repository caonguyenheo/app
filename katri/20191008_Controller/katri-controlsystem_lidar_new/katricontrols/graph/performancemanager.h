#ifndef PERFORMANCEMANAGER_H
#define PERFORMANCEMANAGER_H

#include <QWidget>
#include <QTimer>

#include "katricontrols_export.h"

namespace Ui {
class PerformanceManager;
}

class KATRICONTROLS_EXPORT PerformanceManager : public QWidget
{
    Q_OBJECT

public:
    explicit PerformanceManager(QWidget *parent = nullptr);
    ~PerformanceManager();

    void backButtonConnection(const QObject *receiver, const char *member);

public Q_SLOTS:
    void realtimeDataSlot();

private:
    Ui::PerformanceManager *ui;
    QTimer dataTimer;
};

#endif // PERFORMANCEMANAGER_H
