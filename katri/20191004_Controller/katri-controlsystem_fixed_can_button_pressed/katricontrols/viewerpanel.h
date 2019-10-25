#ifndef VIEWERPANEL_H
#define VIEWERPANEL_H

#include <QWidget>
#include <QMap>
#include <QLabel>
#include <QDateTime>
#include <QMutex>
#include "controll.h"
#include "graph/performancemanager.h"
#include "../katrilogger/command.h"

#include "katricontrols_export.h"

namespace Ui {
class ViewerPanel;
}

class KATRICONTROLS_EXPORT ViewerPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ViewerPanel(QWidget *parent = nullptr);
    ~ViewerPanel();

    void InitControlUI();
    void recordAllActivation(bool active);
    void viewChartConnection(const QObject *receiver, const char *member);
    void viewCameraConnection(const QObject *receiver, const char *member);
    void setStatus(bool status);
    void setEnableAllView(bool status);
    QMap<Command::Type,ControlL*> getMapControl(){return  mapControl;}
    ControlL* getControlViewCamera(){return controlViewCamera;}
    void setMessageRelaySuccess(uint16_t time);
    bool isRecording();
    void EnableViewMessageBox(bool status, uint16_t time);

    // Update relay message
    void relayMessage(QString msg);

    // Logger Node PING received
    void setMessageLoggerNodeId(QString nodeNameId, QDateTime date);
    void showHidePingMessage(bool status);
    QStringList getLoggerNodeNameList();
    void AddTimeDevice(QStringList names);

Q_SIGNALS:
    void signalChangeStatus(bool);
    void signalResize(bool);

public Q_SLOTS:
    void slotUpdateStatus();
    void slotGetSendCommandDevice(Command::Type, int , bool, bool);
    void hideMessageRelayStatus();
    void slotTimerRefreshLoggerList();
    void HideViewMessageBox();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::ViewerPanel *ui;
    QGridLayout *mainGrid = nullptr;
    ControlL *controlViewLidar = nullptr;
    ControlL *controlViewGPS = nullptr;
    ControlL *controlViewCamera = nullptr;
    ControlL *controlViewCanBus = nullptr;
    PerformanceManager *performanceManager;
    QMap<Command::Type,ControlL*> mapControl;
    bool m_isAllChecked;
    QLabel *lblicon = nullptr;
    QLabel *lblmsg = nullptr;

    // Logger Node PING received
    QLabel *lblPingMsg = nullptr;
    QMap<QString,QDateTime> m_loggerNodeName;
    QMutex m_mutexLogger;
    QTimer *m_loggerRespond;

    //Time Deviced
    QLabel *m_seekCurrent_lidar = nullptr;
    QLabel *m_seekCurrent_gps = nullptr;
    QLabel *m_seekCurrent_canbus= nullptr;
    QLabel *m_seekCurrent_camera = nullptr;
};

#endif // VIEWERPANEL_H
