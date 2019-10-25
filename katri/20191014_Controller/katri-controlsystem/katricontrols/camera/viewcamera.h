#ifndef VIEWCAMERA_H
#define VIEWCAMERA_H

#include <QWidget>
#include <QMap>
#include "viewerperformance.h"
#include "viewdatacamera.h"
#include "common/backpanel.h"
#include "../katrilogger/command.h"

namespace Ui {
class viewcamera;
}

class KATRICONTROLS_EXPORT ViewCamera : public QWidget
{
    Q_OBJECT

public:
    explicit ViewCamera(QWidget *parent = nullptr);
    ~ViewCamera();
    void InitControlUI();
    void updateLabel(QString Text);
    void viewCameraConnection(const QObject *receiver, const char *member);
    void backButtonConnection(const QObject *receiver, const char *member);
    void recordAllActivation(bool status);
    void setStatus(bool status);
    void recordAllConnection(const QObject *receiver, const char *member);
    void sendCommandConnection(const QObject *receiver, const char *member);
    QList<RecordButton *> getListRecordButton();
    bool getStatus();
    void setDataViewHD(Command::Type deviceType, int deviceId, QByteArray &data);
    void setDataViewFullHD(Command::Type deviceType, int deviceId, QByteArray &data);
    void setViewStatus(uint id, bool status);
    void setConnect();

Q_SIGNALS:
    void signalSendCommand(bool newDataSet, bool startRecord, int deviceId);
    void signalClickedInside(bool status, int deviceId);
public Q_SLOTS:
    void slotRecordAllClicked(bool status);
    void slotSetActiveChecked(bool status);
    void slotReturnSignalOut(bool status);

private:
    Ui::viewcamera *ui;

};

#endif // viewcamera_H
