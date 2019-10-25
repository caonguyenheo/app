#ifndef VIEWERPANEL_H
#define VIEWERPANEL_H

#include <QWidget>
#include <QMap>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QVariantMap>
#include "katricontrols_export.h"
#include "common.h"
#include "simulationview.h"
#include "lidar/simulationlidarview.h"
//#include "playbackview.h"
#include "playbackcontrol.h"
#include <iostream>
#include <fstream>
#include <imageview.h>
#include <canbusview.h>
#include <mapviewer.h>

namespace Ui {
class ViewerPanel;
}
using namespace std;

class KATRICONTROLS_EXPORT ViewerPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ViewerPanel(QWidget *parent = nullptr);
    ~ViewerPanel();

    void setMainScreenLayout();
    void setFullScreenLayout();
    // Fake UI display
    bool fakeDisplay();
    void setInterfacePage(QString name, bool data);
    void showScreen(QVariantMap map);
    void getScreen(ScreenType type);
    void setSpacing4View(bool isRadar, bool isCamera, bool isSVM, bool isGPS, bool isLiDar, bool isCanbus);
    void setOutsideSpacing4View(int view);

Q_SIGNALS:
    void signalOfPlayControl(int);
    void signalStateFullOrScale(int);
    void signalMaxlenght(int max);


public Q_SLOTS:
    void setSpinnerDisplay(bool display);

private:
    Ui::ViewerPanel *ui;

    QVBoxLayout *vLayoutLeft;
    QVBoxLayout *vLayoutRight;
    QHBoxLayout *hLayoutTop;
    QHBoxLayout *hLayoutBottom;

public:
    ImageView *cameraView = nullptr;
    ImageView *svmView = nullptr;
    MapViewer *gpsView = nullptr;
    LidarGroupView *lidarGrpView = nullptr;
    CANBusView *canBusView = nullptr;
    SimulationView *radarView = nullptr;
    PlaybackControl *control = nullptr;
};

#endif // VIEWERPANEL_H
