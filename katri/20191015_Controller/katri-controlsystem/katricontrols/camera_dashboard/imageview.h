#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QWidget>
#include <QLabel>
#include <QMovie>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QThread>
#include "../common/common.h"
#include <QDir>
#include <QTimer>
#include <QTime>
#include <QTextStream>
#include <QDebug>
//#include <windows.h>
#include "glwidgetimage.h"

enum ImageType {
    CameraView = 0,
    SVMView = 1
};

namespace Ui {
class ImageView;
}

class ImageView : public QWidget
{
    Q_OBJECT

public:
    explicit ImageView(QWidget *parent = nullptr);
    ~ImageView() override;

    void setViewName(QString &name);
    void setViewColor(QColor &color);
    void appendStyleSheet(QString stylesheet);
    void setImageSimulation(bool display);
    void setStyleMainScreenLayout();
    void setStyleFullScreenLayout();
    void setStyle4LayoutView(int view);
    void setCameraLayout();
    void setSVMLayout();
    void setDataCamera(int deviceId, QByteArray &data);

    QGridLayout *gridLayout;
    QHBoxLayout *hLayout;
    QVBoxLayout *vLayout1;
    QVBoxLayout *vLayout3;

Q_SIGNALS:
    void pauseEvent();
    void playEvent();
    void executeJumpPosition(int timestamp, bool isPlay);
    void BrowseEvent();

protected:
    void resizeEvent(QResizeEvent *event) override;

public:
    QMap<int, GLWidgetImage*> m_viewList;

private:
    Ui::ImageView *ui;
    QLabel *lbicon = nullptr;
    QMovie *movie  = nullptr;
};
#endif // IMAGEVIEW_H
