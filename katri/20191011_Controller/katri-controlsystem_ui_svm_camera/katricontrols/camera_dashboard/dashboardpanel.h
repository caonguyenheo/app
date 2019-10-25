#ifndef DASHBOARDPANEL_H
#define DASHBOARDPANEL_H

#include <QWidget>
#include <QMap>
#include <QVBoxLayout>
#include <QHBoxLayout>
//#include <QVariantMap>
#include "katricontrols_export.h"
#include "../common/common.h"
#include <iostream>
#include <fstream>
#include "imageview.h"
#include "common/backpanel.h"

namespace Ui {
class DashboardPanel;
}
using namespace std;

class KATRICONTROLS_EXPORT DashboardPanel : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardPanel(QWidget *parent = nullptr);
    ~DashboardPanel();

    void setMainScreenLayout();
    void setFullScreenLayout();
    void backButtonConnection(const QObject *receiver, const char *member);
    void setDataViewHD(int deviceId, QByteArray &data);
    void setDataViewFullHD(int deviceId, QByteArray &data);

Q_SIGNALS:
    void signalMaxlenght(int max);

public Q_SLOTS:
    void setSpinnerDisplay(bool display);

private:
    Ui::DashboardPanel *ui;

//    QVBoxLayout *vLayoutRight;
    QHBoxLayout *hLayoutTop;
//    QHBoxLayout *hLayoutBottom;

public:
    ImageView *cameraView = nullptr;
    ImageView *svmView = nullptr;
//    MapViewer *gpsView = nullptr;
//    LidarGroupView *lidarGrpView = nullptr;
//    CANBusView *canBusView = nullptr;
//    SimulationView *radarView = nullptr;
//    PlaybackControl *control = nullptr;
};

#endif // DASHBOARDPANEL_H
