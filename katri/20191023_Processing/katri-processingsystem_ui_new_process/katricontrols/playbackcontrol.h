#ifndef PLAYBACKCONTROL_H
#define PLAYBACKCONTROL_H

#include <QWidget>
#include <QLabel>
#include "katricontrols_export.h"
#include <QVector>

namespace Ui {
class PlaybackControl;
}

class KATRICONTROLS_EXPORT PlaybackControl : public QWidget
{
    Q_OBJECT

public:
    explicit PlaybackControl(QWidget *parent = nullptr);
    ~PlaybackControl();
    void resizeEvent(QResizeEvent *event);
//    bool eventFilter(QObject *obj, QEvent *event);
    void controlConnection(const QObject *receiver, const char *member);
    void seekingBarConnection(const QObject *receiver, const char *member);


Q_SIGNALS:
    void signalFullOrScaleScreen(int);
    void signalMouseReleaseSlider(int);

public Q_SLOTS:
    void slotReceiveEOP(bool);
    void slotSyncSeekbarRange(int,int);
    void slotReceivePosition(int,int);
    void slotgetIconFullOrScaleScreen(int state);
    void slotClickFullOrScale();
    void slotSliderRelease();

private:
    Ui::PlaybackControl *ui;
    QLabel *m_seekCurrent;
    QLabel *m_seekTotal;
    int stateClickFullOrScale;
    int m_length = 0;
};

#endif // PLAYBACKCONTROL_H
