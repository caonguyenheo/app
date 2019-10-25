#ifndef SIMULATIONLIDARVIEW_H
#define SIMULATIONLIDARVIEW_H

#include <QWidget>
#include <QLabel>
#include <QMovie>
#include <QMap>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <common.h>
#include "lidar/lidarview.h"

namespace Ui {
class SimulationLidarView;
}

class LidarGroupView : public QWidget
{
    Q_OBJECT

public:
    explicit LidarGroupView(QWidget *parent = nullptr);
    ~LidarGroupView() override;
    void setViewName(QString &name);
    void setStyleMainScreenLayout();
    void setViewColor(QColor &color);
    void setStyleFullScreenLayout();
    void appendStyleSheet(QString stylesheet);
    void setImageSimulation(bool display);

    void setLidarHorizontalLayout();
    void setLidarVerticalLayout();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    void createConnectLidarLeft();

Q_SIGNALS:
    void signalFinishedRender();
//    void getLidarDataLeft(QString path, QVector<QString> *vlidarData, bool *isLoadSuccess);
//    void getTimestampLeft(QString path,  QVector<qint32>* timestamps, bool *isLoadSuccess);
//    void getLidarDataRight(QString path, QVector<QString> *vlidarData, bool *isLoadSuccess);
//    void getTimestampRight(QString path,  QVector<qint32>* timestamps, bool *isLoadSuccess);
//    void getLidarDataBottom(QString path, QVector<QString> *vlidarData, bool *isLoadSuccess);
//    void getTimestampBottom(QString path,  QVector<qint32>* timestamps, bool *isLoadSuccess);
//    void createLidarLeftNextView(QString path, QVector<QVector3D> *curView);
//    void createLidarRightNextView(QString path, QVector<QVector3D> *curView);
//    void createLidarBottomNextView(QString path, QVector<QVector3D> *curView);
//    void setDataForRenderLeftView(ScatterDataModifier *lidarView,  LidarData  *lidarDataRender, int gap, bool isJump = false);
//    void setDataForRenderRightView(ScatterDataModifier *lidarView,  LidarData  *lidarDataRender, int gap, bool isJump = false);
//    void setDataForRenderBottomView(ScatterDataModifier *lidarView,  LidarData  *lidarDataRender, int gap, bool isJump = false);

public:
    QMap<int, LidarView*> m_viewList;
    QHBoxLayout *hLayout1;
    QHBoxLayout *hLayout2;

private:
    Ui::SimulationLidarView *ui;
    QLabel *lbicon = nullptr;
    QMovie *movie = nullptr;
};

#endif // SIMULATIONLIDARVIEW_H
