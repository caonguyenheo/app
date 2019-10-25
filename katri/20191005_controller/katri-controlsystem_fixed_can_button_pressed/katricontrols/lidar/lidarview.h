#ifndef LIDARVIEW_H
#define LIDARVIEW_H

#include <QWidget>
#include "common/backpanel.h"
#include "common/common.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMap>

namespace Ui {
class LidarView;
}

class LidarView : public QWidget
{
    Q_OBJECT

public:
    explicit LidarView(QWidget *parent = nullptr);
    ~LidarView();
    void InitControlUI();
    void backButtonConnection(const QObject *receiver, const char *member);
    void setViewName(QString name);

    void setStyleMainScreenLayout();
    void setViewColor(QColor &color);
private:
    Ui::LidarView *ui;
};

#endif // LIDARVIEW_H
