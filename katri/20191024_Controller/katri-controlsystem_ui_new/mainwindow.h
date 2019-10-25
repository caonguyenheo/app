#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QBuffer>
#include "viewerpanel.h"
#include "controlpanel.h"
#include "videoplayer/fullscreen.h"
#include "graph/performancemanager.h"
#include "camera/viewcamera.h"
#include "menubar.h"
#include "common/FrameLess.h"
#include "pattern/Mediator.h"
#include "katrilogger/command.h"
#include "katrilogger/nodeWorker.h"
#include "katrilogger/runtime.h"
#include "pattern/Message.h"
#include "define.h"
#include "katrilogger/appConfig.h"
#include "ui_all/record.h"
#include "common/common.h"
#include "checkedbox/checkedbox.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;
    MainWindow& operator=(const MainWindow&);

    enum Status {INIT, RECORD_ALL, STOP_ALL, DETECT_RECORD, DETECT_STOP, RELAYING, RELAY_STOP,};
    void setupUi();
    void showFullScreenFunc();
    void setupMediator();
    void setupLogFile();
    void startNode(int argc, char *argv[]);
    void sendRelayCommand();
    void verifyRelayAllDone();

public Q_SLOTS:
    void slotClickedExitButton();
    void slotClickedMinimumSizeButton();
    void slotClickedMaximumSizeButton();
    void slotcheckedlickchecbox();

#if (ENABLE_UI_OLD == 1)
    void slotRecordAllBtnClicked(bool active);
    void slotRelayAllBtnClicked(bool active);
    void slotStopAllBtnClicked(bool active);
    //void slotRecordAllBtnChange(bool active);
    void slotRecordAllBtnClickedviewcamera(bool active);
    void slotViewChartClicked();
    void slotViewCameraClicked();
    void slotbackButtonClicked();

    void slotSendCommand(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS);
    void startNewKittiCommand(Command::Type loggerType, string loggerId);
    void startCurrentKittiCommand(Command::Type loggerType, string loggerId);
    void stopRecordCommand(Command::Type loggerType, string loggerId);
    void slotReceiveVideoFrame(int deviceType, int deviceId, QByteArray data);

    void handlerCameraView(uint id, bool status);
    void handlerCameraControl(uint id, bool status);
    void slotReciveLostConnection(int deviceType, int deviceId, QByteArray data);
    void slotUpdateStatusButton(int status);
    void slotUpdateDeviceOutSide(bool status, int deviceId);

    //void slotLoggerNameAck(QString loggerNodeName);
    void slotReceiveRelayState(QString loggerNodeName, bool status);
    void slotRelayTimeout();

    // Logger Node PING received
    void slotPingLoggerNodeId(QString nodeNameId);

    void slotResizeWindow(bool active);
 #endif

Q_SIGNALS:

protected:
    void mousePressEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event);

private:
#if (ENABLE_UI_OLD == 1)
    ViewerPanel *m_viewer = nullptr;
    ControlPanel *m_record = nullptr;
    FullScreen *m_fullscreen = nullptr;
    PerformanceManager *m_viewChart;
    viewcamera *m_viewCamera = nullptr;
    FrameLess *frame = nullptr;
    ConcreteObserver *observer = nullptr;
    //NodeWorker *node = nullptr;
#endif
    Message *message = nullptr;
    MenuBar *m_menubar = nullptr;
    Mediator *mediator = nullptr;
    NodeWorker server;
    QMap <QString, SocketThread*>::iterator deviceid;
    record *m_viewerall = nullptr;
    //Relay setting
    bool m_relayTimeout;
    checkedbox *m_cbbox = nullptr;
    QString Curdate;
};

#endif // MAINWINDOW_H
