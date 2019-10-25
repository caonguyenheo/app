#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QWidget>
#include <QMovie>
#include <QTimer>
#include <QDebug>
#include "katricontrols_export.h"
#include <QLabel>

namespace Ui {
class ControlPanel;
}

class KATRICONTROLS_EXPORT ControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = nullptr);
    ~ControlPanel();

    void recordAllConnection(const QObject *receiver, const char *member);
    void relayAllConnection(const QObject *receiver, const char *member);
    void stopAllConnection(const QObject *receiver, const char *member);
    void updateStatusButton(QString Text, bool status);
    void setActiveButtonRecord(bool active);
    void setRelayEnabled(bool status);
    void setStopEnabled(bool status);
    void startRelayingStatus();
    void stopRelayingStatus();

    void updateStatusRecordButton(bool status);
    void updateStatusRelayButton(QString Text);

    void updateStatusStopButton(bool status);

    void StartRecord(bool status);
    void StopRecord(bool status);
    void RelayRecord(bool status);
    void Stoprecording();
    void updateLayoutStyles(bool active, QLabel *label);

    //void stoploading();

public Q_SLOTS:
    void setButtonIcon();

private:
    Ui::ControlPanel *ui;
    QMovie *myMovie = nullptr;
};

#endif // CONTROLPANEL_H
