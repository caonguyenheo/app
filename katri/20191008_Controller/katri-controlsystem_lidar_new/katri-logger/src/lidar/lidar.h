#ifndef LIDAR_H
#define LIDAR_H

#include "../config.h"
#if ENABLE_ROS_VERSION == 1
#include "QDir"
#include "lidarParameter.h"
#include "../loggerManager.h"
#include "../node1Worker.h"
#include "../source.h"

class Lidar : public Source
{
public:
    Lidar();
    Lidar(const LidarParameter &, QObject *parent = nullptr);
    ~Lidar() override;
    void start() override;
    void stop() override;

    bool isRunning() override;
    QString getId() override;
    QString getName() override;
    QString getStatus() override;
    void setPath(const QString &) override;

private:
    void _createOutputDirectory();
    void _updateStatus(bool status);
    void _initRuntimeFile();

private:
    LidarParameter _parameters;
    std::shared_ptr<spdlog::logger> _logger = nullptr;
};
#endif
#endif // LIDAR_H
