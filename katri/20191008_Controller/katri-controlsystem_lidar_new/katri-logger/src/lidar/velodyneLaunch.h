#ifndef VELODYNELAUNCH_H
#define VELODYNELAUNCH_H
#include "../config.h"
#if ENABLE_ROS_VERSION == 1
#include "QDir"
#include "QProcess"
#include "QThread"
#include "../loggerManager.h"

class VelodyneLaunch : public QThread
{
    Q_OBJECT
public:
    VelodyneLaunch(QObject *parent = nullptr);
    ~VelodyneLaunch() override;
    void run() override;

private:
    QProcess *_process = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

    //private Q_SLOTS:
    //    void captureDataOutput();
};
#endif
#endif // VELODYNELAUNCH_H
