#include "loggerManager.h"

std::shared_ptr<spdlog::logger> LoggerManager::getAppLogger(){
    return getLogger(LogPrefix::APP);
}

std::shared_ptr<spdlog::logger> LoggerManager::getCanLogger(const QString id){
    return getLogger(LogPrefix::CAN, id);
}

std::shared_ptr<spdlog::logger> LoggerManager::getLidarLogger(const QString id)
{
    return getLogger(LogPrefix::LIDAR, id);
}

std::shared_ptr<spdlog::logger> LoggerManager::getCameraLogger(const QString id){
    return getLogger(LogPrefix::CAMERA, id);
}

std::shared_ptr<spdlog::logger> LoggerManager::getGpsLogger(const QString id){
    return getLogger(LogPrefix::GPS, id);
}

std::shared_ptr<spdlog::logger> LoggerManager::getLogger(const LogPrefix prefix, const QString id){
    try{
        QString loggerName = getLogPrefix(prefix) + id;
        QString filename = loggerName + ".log";
        auto logger = spdlog::get(loggerName.toStdString());
        if (!logger){
            logger = spdlog::rotating_logger_mt(loggerName.toStdString(), filename.toStdString() ,
                                                1048576 * AppConfig::instance().getLogSize(),
                                                AppConfig::instance().getNumberOfLogs());
        }
        return logger;
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        _logToApp(ex.what());
    }
    return nullptr;
}

QString LoggerManager::getLogPrefix(const LogPrefix prefix){
    if (prefix == LogPrefix::APP) return "app";
    if (prefix == LogPrefix::CAMERA) return "camera";
    if (prefix == LogPrefix::CAN) return "can";
    if (prefix == LogPrefix::LIDAR) return "lidar";
    if (prefix == LogPrefix::RADAR) return "radar";
    if (prefix == LogPrefix::GPS) return "gps";
    return "app";
}

void LoggerManager::init(){
    try{
        spdlog::set_level(spdlog::level::from_str(AppConfig::instance().getLogLevel().toStdString()));
        spdlog::flush_every(std::chrono::seconds(5));
        // init app logger
        auto _logger = getLogger(LogPrefix::APP);
        if(_logger)
            _logger->info("Welcome to KATRI Logging System!");
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        _logToApp(ex.what());
    }
}

void LoggerManager::_logToApp(const char *msg){
    QFile file("app.log");
    if (file.open(QIODevice::Append | QIODevice::Text))
    {
        QTextStream out(&file);
        out << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") << " Internal logger error occurred: " << msg << "\n";
        file.close();
    }
}
