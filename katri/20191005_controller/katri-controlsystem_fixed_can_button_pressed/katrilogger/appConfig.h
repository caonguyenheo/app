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
    bool _ros2Enable;

    QStringList _loggerNodes;

    int _relayTimeout;
    int _loggerRespondTimeout;

public:
    void reload();

    // ROS configuration
    QString getTopicName() const;
    QString getNodeName() const;

    // Camera datasources
    QStringList getCameras() const;

    // Logger Nodes
    QStringList getLoggerNodes();

    // Enable ROS2
    bool getRos2Enable();

    // Get relay timeout
    int getRelayTimeout();
    int getLoggerRespondTimeout();
};

#endif // APPCONFIG_H
