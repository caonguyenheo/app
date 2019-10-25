#ifndef VIEWDATAEXPORT_H
#define VIEWDATAEXPORT_H

#include <QWidget>
#include <QLabel>
#include <QDateTime>
#include "exportdata_left.h"

#include "katricontrols_export.h"

namespace Ui {
class viewdataexport;
}

class KATRICONTROLS_EXPORT ViewDataExport : public QWidget
{
    Q_OBJECT

public:
    explicit ViewDataExport(QWidget *parent = nullptr);
    ~ViewDataExport();

    void selectConnection(const QObject *receiver, const char *member);
    exportdata_left *camera = nullptr;
    exportdata_left *svm = nullptr;
    exportdata_left *gps = nullptr;
    exportdata_left *radar = nullptr;
    exportdata_left *lidar = nullptr;
    exportdata_left *canbus = nullptr;
    QString getStartTimestamp();
    QString getEndTimestamp();
    void setMinTimestamp(QDateTime min);
    void setMaxTimestamp(QDateTime max);

Q_SIGNALS:
    void signalMessageSuccess(int time);

public Q_SLOTS:
    void slotMessageSuccess(int time);
    void hideMessageExportStatus();
    void slotStartSliderValueChanged(double value);
    void slotEndSliderValueChanged(double value);

private:
    Ui::viewdataexport *ui;
    QLabel *lblicon = nullptr;
    QLabel *lblmsg = nullptr;
    QObject *_rangeSlider;
    QLabel *startLabel;
    QLabel *endLabel;

    QDateTime minTS;
    QDateTime maxTS;
};

#endif // VIEWDATAEXPORT_H
