#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "QSettings"

namespace appconfig {
const QString config = "config.properties";
} // namespace appconfig

using namespace appconfig;

class AppConfig
{
public:
    static AppConfig & instance();
    ~AppConfig();
private:
    AppConfig();
    AppConfig(const AppConfig &) = delete;
    AppConfig &operator=(const AppConfig &) = delete;

    // ROS properties
    QString _topicName;
    QString _nodeName;

    QStringList _loggerNodes;
    QStringList _cameras;

public:
    void reload();

    // ROS configuration
    QString getTopicName() const;
    QString getNodeName() const;

    // Camera datasources
    QStringList getCameras() const;

    // Logger Nodes
    QStringList getLoggerNodes();
};



#endif // APPCONFIG_H
