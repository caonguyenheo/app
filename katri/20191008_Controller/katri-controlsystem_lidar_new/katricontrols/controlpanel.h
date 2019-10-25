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

    enum Status {INIT, RECORD_ALL, STOP_ALL, DETECT_RECORD, DETECT_STOP, RELAYING, RELAY_STOP,};
    void recordAllConnection(const QObject *receiver, const char *member);
    void relayAllConnection(const QObject *receiver, const char *member);
    void stopAllConnection(const QObject *receiver, const char *member);
    void setRelayEnabled(bool status);

    void UpdateStatusButton(int status);

    void updateStatusRecordButton(bool status);
    void updateStatusStopButton(bool status);
    void updateStyleRelay(bool status);
    void updateLayoutStyles(bool active, QLabel *label);

public Q_SLOTS:
    void setButtonIcon();

private:
    Ui::ControlPanel *ui;
    QMovie *myMovie = nullptr;
};

#endif // CONTROLPANEL_H
