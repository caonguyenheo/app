#ifndef FULLSCREEN_H
#define FULLSCREEN_H

#include <QWidget>
#include "katricontrols_export.h"
#include <QLabel>
#include <QGridLayout>
#include "playbackcontrol.h"

class KATRICONTROLS_EXPORT FullScreen : public QWidget
{
        Q_OBJECT

public:
    explicit FullScreen(QWidget *parent = nullptr);
    ~FullScreen();
    void setupUI();
    void setbackground(QString path);
public slots:
    void slotClickedMaximumSizeButton();
protected:
    void resizeEvent(QResizeEvent *event) override;
private:
    PlaybackControl *playback = nullptr;
};

#endif // FULLSCREEN_H
