#ifndef DATASOURCEMANAGER_H
#define DATASOURCEMANAGER_H

#include "QDebug"
#include "QHash"
#include "QSettings"
#include "QThread"
#include "appConfig.h"
#include "camera/camera.h"
#include "can/can.h"
#include "command.h"
#include "config.h"
#include "dataSourceFactory.h"
#include "lidar/lidar.h"
#include "gps/gps.h"
#include "node1Worker.h"
#include "nodeWorker.h"
#include "runtime.h"
#include "source.h"

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include <QDirIterator>

#include "src/camera/imageextraction.h"
#include "tcpsocket/katritcpsocket.h"
const QString cameraSourceConfig = "camera.properties";
const QString canSourceConfig    = "can.properties";
const QString lidarSourceConfig  = "lidar.properties";
const QString gpsSourceConfig    = "gps.properties";
enum Key { CAMERA, CAN, LIDAR, GPS };

class DataSourceManager
{
public:
    static DataSourceManager& instance();
    DataSourceManager &lookup();
    DataSourceManager &createSources();
    void start(const Command);
    void stop(const Command);
    void copy(const Command _cmd);
    void setNode(NodeWorker *);
    void setSocket(NodeWorker *);
    NodeWorker *getNode();
    QString pathkititar;    
    //void CheckNumberframe(QString Path_folder, QString Path_timestamp, QString Filename);

private:
    DataSourceManager(const DataSourceManager &) = delete;
    DataSourceManager& operator =(const DataSourceManager &) = delete;
    DataSourceManager();
    void _startMe(Source *, const Command);
    void _stopMe(Source *, const Command);

    QString _getKittiPath(const Command);

/*
Q_SIGNALS:
    void signalimageExtractor();
*/

private:
    QHash<int, QHash<QString, QString>> _sources;
    std::shared_ptr<spdlog::logger> _logger;
    QHash<QString, Camera *> _cameraInstances;
    QHash<QString, Can *> _canInstances;
#if ENABLE_ROS_VERSION == 1
    QHash<QString, Lidar *> _lidarInstances;
    QHash<QString, Gps *> _gpsInstances;
#endif
    NodeWorker *_node;
    NodeWorker *_tcpclient;
    //ImageExtraction* m_imgExt = nullptr;
    //QString Path_file_RAW;
};

#endif // DATASOURCEMANAGER_H
