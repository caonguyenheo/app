#ifndef VIEWDATACAMERA_H
#define VIEWDATACAMERA_H

#include <QWidget>
#include <QMap>
#include "viewerperformance.h"
#include "videoplayer/video.h"
#include "../katrilogger/command.h"
#include "katricontrols_export.h"

namespace Ui {
class viewdatacamera;
}

class KATRICONTROLS_EXPORT viewdatacamera : public QWidget
{
    Q_OBJECT

public:
    explicit viewdatacamera(QWidget *parent = nullptr);
    ~viewdatacamera();
    void initDataUI(QMap<int, QString> mapDataSource);
    void setValueIP(viewerperformance *m_view, bool isDefault = false, QString value = "");
    void setactive(bool status);
    QList<viewerperformance*> getVideoViewerList();
    void sendCommandConnection(const QObject *receiver, const char *member);
    void setDataCamara(int deviceId, QByteArray &data);
private:
    Ui::viewdatacamera *ui;
    viewerperformance *m_view = nullptr;
    viewerperformance *m_view1 = nullptr;
    viewerperformance *m_view2 = nullptr;
    viewerperformance *m_view3 = nullptr;

    //Simulation widget
    Video *video = nullptr;

    QList<viewerperformance*> listViewerPerformance;
    QMap<int, QString> mapDataIDCameraIPAddress;
};

#endif // VIEWDATACAMERA_H
