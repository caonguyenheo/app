#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QWidget>
#include <QMovie>
#include <QTextEdit>
#include <QThread>
#include <windows.h>
//#include "katristyle.h"

// QMapControl includes.
#include <MapViewer/osm/QMapControl.h>
#include <MapViewer/osm/Geometry.h>
using namespace qmapcontrol;

namespace Ui {
    class GPSView;
}

class MapViewer : public QWidget
{
Q_OBJECT

public:
    explicit MapViewer(QWidget *parent = nullptr);

    ~MapViewer();

    void setViewName(QString &name);

    void setStyleMainScreenLayout();

    void setStyleFullScreenLayout();

    void setViewColor(QColor &color);

    void setImageSimulation(bool display);

    void appendStyleSheet(QString stylesheet);

    void moveObj(const PointWorldCoord& newCoord);

    bool insideRadius(const PointWorldCoord& newCoord);

private:

    Ui::GPSView *ui;
    QLabel *lbicon = nullptr;
    QMovie *movie = nullptr;

private:
    PointWorldCoord m_centralPoint;
    double m_lon;
    double m_lat;
    QString style;
    void setStyleMapviewer_Panel(QWidget *GPSpanel);
    void setStyleMapviewer_Label(QLabel *GPSLabel);

Q_SIGNALS:
    void signalFinishedRender();

public Q_SLOTS:
    void slotReceiveTimestamp(int key, QString value);

private:
    QMapControl* m_map_control;
    std::shared_ptr<Geometry> m_iconObj;

};

#endif
