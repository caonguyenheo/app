#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "QMutex"
#include "QQueue"
#include "command.h"
#include "config.h"
#include "dataSourceManager.h"
#include "loggerManager.h"
#include "nodeWorker.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include <QObject>

#if ENABLE_ROS_VERSION == 1
#include "node1Worker.h"
#include "lidar/ousterLaunch.h"
#include "lidar/velodyneLaunch.h"
#include "gps/gpsLaunch.h"
#endif

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(int argc, char **argv, QObject *parent = nullptr);
    ~Controller();
    void startWork();
private slots:
    void doIt(const Command command);
private:
    QQueue<Command> _commands;
    NodeWorker *_node = nullptr;
    int _argc;
    char **_argv = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;
    //mutable QMutex _mutex;

#if ENABLE_ROS_VERSION == 1
    Node1Worker *_node1 = nullptr;
    VelodyneLaunch *_velodyneLaunch = nullptr;
    OusterLaunch *_ousterLaunch = nullptr;
    GpsLaunch *_gpsLaunch = nullptr;
#endif
};

#endif // CONTROLLER_H
