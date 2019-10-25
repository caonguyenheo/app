#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "QSettings"

namespace appconfig {
const QString config = "config.properties";
} // namespace appconfig
const QString CCAN_PREFIX = "ccan";
const QString CAMERA_PREFIX = "cam";
const QString LIDAR_PREFIX  = "lidar";
const QString GPS_PREFIX    = "GNSS";
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

     QString _ipserver;
     QString _port;
     // Camera port mapping table
     QHash<QString, QString> _portTable;

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

    QString getIP() const;
    QString getPort() const;

    // Camera mapping table
    QString getHostForDevice(uint8_t,const QString &);
    QString getDeviceName(QString &);
    int getLoggerIdFromIp(const QString &ipAddr);
    QList<QString> getListDeviceName(QString &);

};

#endif // APPCONFIG_H
