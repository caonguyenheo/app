#ifndef LOGGERMANAGER_H
#define LOGGERMANAGER_H
#include "QString"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "appConfig.h"
#include "QFile"
#include "QDateTime"

enum class LogPrefix{APP, CAMERA, CAN, LIDAR, RADAR, GPS};

class LoggerManager
{
public:
    static std::shared_ptr<spdlog::logger> getLogger(const LogPrefix prefix = LogPrefix::APP, const QString id ="");
    static std::shared_ptr<spdlog::logger> getAppLogger();
    static std::shared_ptr<spdlog::logger> getCanLogger(const QString id);
    static std::shared_ptr<spdlog::logger> getCameraLogger(const QString id);
    static std::shared_ptr<spdlog::logger> getLidarLogger(const QString id);
    static std::shared_ptr<spdlog::logger> getGpsLogger(const QString id);
    static QString getLogPrefix(const LogPrefix prefix);
    static void init();
private:
    static void _logToApp(const char *);
};

#endif // LOGGERMANAGER_H
