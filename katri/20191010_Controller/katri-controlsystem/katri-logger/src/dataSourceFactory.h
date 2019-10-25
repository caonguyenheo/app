#ifndef DATASOURCEFACTORY_H
#define DATASOURCEFACTORY_H

#include "QHash"
#include "QString"
#include "camera/camera.h"
#include "camera/cameraParameter.h"
#include "can/can.h"
#include "can/canParameter.h"
#include "config.h"
#if ENABLE_ROS_VERSION == 1
#include "lidar/lidar.h"
#include "lidar/lidarParameter.h"
#include "gps/gps.h"
#include "gps/gpsParameter.h"
#endif
#include "source.h"

namespace datasourcefactory {
const QString ID = "id";
const QString SRC = "src";
} // namespace datasourcefactory

using namespace datasourcefactory;

template<class T, class P>
class DataSourceFactory
{
public:
    DataSourceFactory() {}
    T *createSourceInstance(const QHash<QString, QString> &) const;
};

template<>
class DataSourceFactory<Can, CanParameter>
{
public:
    DataSourceFactory() {}
    Can *createSourceInstance(const QHash<QString, QString> &param) const
    {
        CanParameter canParameter;
        canParameter.setId(param.value(ID));
        canParameter.setSrc(param.value(SRC));

        Can *can = new Can(canParameter);

        return can;
    }
};

template<>
class DataSourceFactory<Camera, CameraParameter>
{
public:
    DataSourceFactory() {}
    Camera *createSourceInstance(const QHash<QString, QString> &param) const
    {
        CameraParameter camParameter;
        camParameter.setId(param.value(ID));
        camParameter.setSrc(param.value(SRC));
        camParameter.setSize(AppConfig::instance().getVideoSize());
        camParameter.setFormat(AppConfig::instance().getVideoFormat());
        camParameter.setFileName(AppConfig::instance().getVideoName());

        Camera *cam = new Camera(camParameter);

        return cam;
    }
};
#if ENABLE_ROS_VERSION == 1
template<>
class DataSourceFactory<Lidar, LidarParameter>
{
public:
    DataSourceFactory() {}
    Lidar *createSourceInstance(const QHash<QString, QString> &param) const
    {
        LidarParameter lidarParameter;
        lidarParameter.setId(param.value(ID));

        Lidar *lidar = new Lidar(lidarParameter);

        return lidar;
    }
};

template<>
class DataSourceFactory<Gps, GpsParameter>
{
public:
    DataSourceFactory() {}
    Gps *createSourceInstance(const QHash<QString, QString> &param) const
    {
        GpsParameter gpsParameter;
        gpsParameter.setId(param.value(ID));

        Gps *gps = new Gps(gpsParameter);

        return gps;
    }
};
#endif
#endif // DATASOURCEFACTORY_H
