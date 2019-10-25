#ifndef GPSLAUNCH_H
#define GPSLAUNCH_H
#include "../config.h"

#if ENABLE_ROS_VERSION == 1
#include "QDir"
#include "QProcess"
#include "QThread"
#include "../loggerManager.h"

class GpsLaunch : public QThread
{
    Q_OBJECT
public:
    GpsLaunch(QObject *parent = nullptr);
    ~GpsLaunch() override;
    void run() override;

private:
    QProcess *_process = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

    //private Q_SLOTS:
    //    void captureDataOutput();
};
#endif
#endif // GPSLAUNCH_H
