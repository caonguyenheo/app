#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "QDate"
#include "QDebug"
#include "QSettings"
namespace appconfig {
const QString LIDAR_PREFIX = "lidar";
const QString CAMERA_PREFIX = "camera";
const QString CAN_PREFIX = "can";
const QString GPS_PREFIX = "gps";
const QString config = "config.properties";
const QString NAS = "NAS.properties";
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

    // Camera mapping table
    QHash<QString, QString> _cameraTable;

    // Can mapping table
    QHash<QString, QString> _canTable;

    // Lidar mapping table
    QHash<QString, QString> _lidarTable;

    // Gps mapping table
    QHash<QString, QString> _gpsTable;

    // ROS properties
    QString _topicName;
    QString _nodeName;

    // Global properties
    QString _rootPath;

    // Nas properties
    QString _rootPathNAS;
    QString _username;
    QString _password;

    // Camera properties
    QString _videoSize;
    QString _videoFormat;
    QString _videoName;

    // Log properties
    uint _numberOfLogs;
    uint _logSize;
    QString _logLevel;

    // Control Node
    QString _controllerNodeName;

    //create share directory
    QString _shareDirPath;

    //create share ip server
    QString _ipserver;

    //create share postserver
    QString _portserver;

    //create share ip client
    QString _ipclient;

    //create share directory
    bool _keepKittiData;

public:
    void reload();

    // Lidar mapping table
    QString getLidarMappingValue(const QString &);

    // CAN mapping table
    QString getCanMappingValue(const QString &);

    // Camera mapping table
    QString getCameraMappingValue(const QString &);

    // Gps mapping table
    QString getGpsMappingValue(const QString &);

    // ROS configuration
    QString getTopicName() const;
    QString getNodeName() const;

    // Global configuration
    QString getRootPath() const;

    // Camera configuration
    QString getVideoSize() const;
    QString getVideoFormat() const;
    QString getVideoName() const;

    // Log configuration
    uint getNumberOfLogs() const;
    uint getLogSize() const;
    QString getLogLevel() const;

    QString getControllerNodeName() const;
    QString getRootPathNAS() const;
    QString getUserName() const;
    QString getPassword() const;
    QString getShareDirPath() const;
    bool getKeepKittiData() const;

    QString getIpServer() const;
    QString getPortServer() const;
    QString getIpClient() const;
};



#endif // APPCONFIG_H
