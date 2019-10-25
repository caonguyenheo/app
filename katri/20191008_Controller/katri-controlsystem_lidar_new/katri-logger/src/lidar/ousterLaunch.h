#ifndef OUSTERLAUNCH_H
#define OUSTERLAUNCH_H
#include "../config.h"

#if ENABLE_ROS_VERSION == 1
#include "QDir"
#include "QProcess"
#include "QThread"
#include "../loggerManager.h"

class OusterLaunch : public QThread
{
    Q_OBJECT
public:
    OusterLaunch(QObject *parent = nullptr);
    ~OusterLaunch() override;
    void run() override;

private:
    QProcess *_process = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

    //private Q_SLOTS:
    //    void captureDataOutput();
};
#endif

#endif // OUSTERLAUNCH_H
