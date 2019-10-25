#ifndef VIDEO_H
#define VIDEO_H

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QDebug>
#include <QImage>
#include "fullscreen.h"
#include "zoombutton.h"

#include "katricontrols_export.h"

class KATRICONTROLS_EXPORT Video : public QWidget
{
    Q_OBJECT

public:
    explicit Video(QWidget *parent = nullptr);
    ~Video() override;

    void setupUI();
    void setbackground();
    void setDataCameraView(QByteArray &data);

public Q_SLOTS:
    void slotClickedMaximumSizeButton();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    zoombutton *zoom = nullptr;
    FullScreen *fullscreen = nullptr;
    QLabel *graphicsImage;
    int frameId;
    bool isRendering;
};

#endif // VIDEO_H
