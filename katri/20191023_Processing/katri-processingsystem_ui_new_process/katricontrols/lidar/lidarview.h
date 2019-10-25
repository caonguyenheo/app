#ifndef LIDARVIEW_H
#define LIDARVIEW_H

#include <QWidget>
#include <QPainter>
#include <QQuickWidget>
#include <QtDataVisualization>
#include "scatterdatamodifier.h"

using namespace QtDataVisualization;

class LidarView : public QWidget
{
    Q_OBJECT
public:
    LidarView(QWidget* parent = nullptr);
    ~LidarView() override;
    void paintEvent(QPaintEvent*) override;
    void readDataFile(QString path);
    void startRender();

Q_SIGNALS:
    void signalFinishedRender();

public Q_SLOTS:
    void slotReceiveTimestamp(int,QString);

private:
    QVector<QVector3D>  currentData;
    Q3DScatter          *m_3dScatter;
    ScatterDataModifier *m_scatterData;
};

#endif // LIDARVIEW_H
