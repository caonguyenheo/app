#ifndef PLAYBACKCONTROL_H
#define PLAYBACKCONTROL_H

#include <QWidget>
#include <QPushButton>
#include "katricontrols_export.h"

namespace Ui {
class PlaybackControl;
}

class KATRICONTROLS_EXPORT PlaybackControl : public QWidget
{
    Q_OBJECT

public:
    explicit PlaybackControl(QWidget *parent = nullptr);
    ~PlaybackControl();
    void fullScreenConnection(const QObject *receiver, const char *member);
    QPushButton *fullscreen = nullptr;

private:
    Ui::PlaybackControl *ui;
    bool isPlayed;
public:
    void setPlaySeekbar(bool isEnable);
public slots:
    void clickPlayBtn();
};

#endif // PLAYBACKCONTROL_H
