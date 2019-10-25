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
    void slotRecordAllBtnClicked(bool active);
    void slotRelayAllBtnClicked(bool active);
    void slotStopAllBtnClicked(bool active);
    //void slotRecordAllBtnChange(bool active);
    void slotRecordAllBtnClickedviewcamera(bool active);
    void slotViewChartClicked();
    void slotViewCameraClicked();
    void slotbackButtonClicked();
    void slotClickedExitButton();
    void slotClickedMinimumSizeButton();
    void slotClickedMaximumSizeButton();

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
Q_SIGNALS:

protected:
    void mousePressEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event);

private:
    ViewerPanel *m_viewer = nullptr;
    ControlPanel *m_record = nullptr;
    FullScreen *m_fullscreen = nullptr;
    PerformanceManager *m_viewChart;
    viewcamera *m_viewCamera = nullptr;
    MenuBar *m_menubar = nullptr;
    FrameLess *frame = nullptr;
    Mediator *mediator = nullptr;
    ConcreteObserver *observer = nullptr;
    Message *message = nullptr;
    //NodeWorker *node = nullptr;
    QString Curdate;
    NodeWorker server;
    QMap <QString, SocketThread*>::iterator deviceid;

    //Relay setting
    bool m_relayTimeout;
};

#endif // MAINWINDOW_H
