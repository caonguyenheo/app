#include "mapviewer.h"
#include <ui_mapviewer.h>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QAction>
#include <QMenuBar>
#include <QStatusBar>
#include <QLabel>
#include <QTimer>
#include <QTime>
#include <common.h>
#include <cmath>

#include <MapViewer/osm/GeometryPointCircle.h>
#include <MapViewer/osm/GeometryPolygon.h>
#include <MapViewer/osm/LayerGeometry.h>
#include <MapViewer/osm/LayerMapAdapter.h>
#include <MapViewer/osm/MapAdapterOSM.h>

MapViewer::MapViewer(QWidget *parent)
        : QWidget(parent),
          ui(new Ui::GPSView)
{
    ui->setupUi(this);

    m_map_control = new QMapControl(this); // Create a new QMapControl.
    m_map_control->enableCrosshairs(false);
    m_map_control->enableScalebar(false);
    m_map_control->enableZoomControls(false);
    m_map_control->addLayer(std::make_shared<LayerMapAdapter>("Map", std::make_shared<MapAdapterOSM>())); //Add OSM map layer
    std::shared_ptr<LayerGeometry> gpsLayer(std::make_shared<LayerGeometry>("GPS")); // Add GPS layer.
    m_map_control->addLayer(gpsLayer);

    // Create object in GPS layer
    m_centralPoint = PointWorldCoord(0, 0);  //long, lat
    const char* iconResource = ":/img/icon/hyundaisupercar.png";
    m_iconObj = std::make_shared<GeometryPointImage>(m_centralPoint, iconResource);
    gpsLayer->addGeometry(m_iconObj);

    // Set central point
    m_map_control->setMapFocusPoint(m_centralPoint);
    m_map_control->setZoom(10);
    ui->verticalLayout->addWidget(m_map_control);
}

MapViewer::~MapViewer()
{
    delete ui;
}

void MapViewer::setViewName(QString &name)
{
    if (name.compare("") != 0) {
        ui->GPSLabel->setText(name);
        this->setStyleMainScreenLayout();
    }
}

void MapViewer::setStyleMainScreenLayout()
{
    this->setStyleMapviewer_Label(ui->GPSLabel);
    ui->GPSLabel->setFixedHeight(29);
}

void MapViewer::setStyleFullScreenLayout()
{
    ui->GPSLabel->setStyleSheet("color: white;"
                                "font: 32px 'Roboto';"
                                "font-weight:500;"
                                "padding-bottom:5px;"
    );
    ui->GPSLabel->setFixedHeight(55);
}

void MapViewer::setViewColor(QColor &)
{
    this->setStyleMapviewer_Panel(ui->GPSPanel);
}

void MapViewer::setImageSimulation(bool display)
{
    if (lbicon == nullptr) {
        lbicon = new QLabel(this);
        movie = new QMovie(":/img/icon/Loading_iCon.gif");
        lbicon->setAttribute(Qt::WA_TranslucentBackground, true);
        lbicon->setMovie(movie);
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x() - 14, point.y() + 2, 28, 28);
        lbicon->setMaximumSize(QSize(28, 28));
    }
    if (lbicon != nullptr) {
        if (display) {
            lbicon->show();
            movie->start();
        } else {
            lbicon->hide();
            movie->stop();
        }
    }
}

void MapViewer::appendStyleSheet(QString stylesheet)
{
    ui->GPSPanel->setStyleSheet(ui->GPSPanel->styleSheet().append(stylesheet));
}

void MapViewer::slotReceiveTimestamp(int key, QString value)
{
    Q_UNUSED(key);
//    QTime now = QTime::currentTime();
    QStringList split = value.split(" ");
    if (split.count() >=2) {
        m_lat = split[0].toDouble();
        m_lon = split[1].toDouble();
//        qDebug() << "Lat:" << m_lat << "Long:"<< m_lon;

        PointWorldCoord objOrdinate(m_lon, m_lat);
        moveObj(objOrdinate);

        emit signalFinishedRender();
    }
//    qDebug() << "GPS render:" << now.msecsTo(QTime::currentTime());
}

void MapViewer::setStyleMapviewer_Panel(QWidget *GPSpanel)
{
//    style = KatriStyle::getExportFilecss();
//    GPSpanel->setStyleSheet(style);
}

void MapViewer::setStyleMapviewer_Label(QLabel *GPSLabel)
{
//    style = KatriStyle::getExportFilecss();
//    GPSLabel->setStyleSheet(style);
}

void MapViewer::moveObj(const PointWorldCoord& newCoord)
{
    std::shared_ptr<LayerGeometry> gpsLayer = std::static_pointer_cast<LayerGeometry>(m_map_control->getLayer("GPS"));
    if(!insideRadius(newCoord))
    {
        //object is far from the visible area, set focus again
        m_centralPoint = newCoord;
        m_map_control->setMapFocusPoint(m_centralPoint);
    }
    gpsLayer->removeGeometry(m_iconObj, false); //remove object but not re-draw
    const char* iconResource = ":/img/icon/hyundaisupercar.png";
    m_iconObj = std::make_shared<GeometryPointImage>(newCoord, iconResource);
    gpsLayer->addGeometry(m_iconObj);
}

bool MapViewer::insideRadius(const PointWorldCoord& newCoord)
{
    // Check object inside of bound
    double distanceX = std::abs(newCoord.rawPoint().x() - m_centralPoint.rawPoint().x());
    double distanceY = std::abs(newCoord.rawPoint().y() - m_centralPoint.rawPoint().y());
    double distance = std::sqrt(distanceX*distanceX + distanceY*distanceY);
    double boundary = std::min(this->size().width()/2, this->size().height()/2);

    return (distance < boundary);
}
