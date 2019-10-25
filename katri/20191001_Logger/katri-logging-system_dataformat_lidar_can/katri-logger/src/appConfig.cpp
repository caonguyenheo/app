#include "appConfig.h"

AppConfig::AppConfig()
{
    reload();
}

AppConfig::~AppConfig()
{
}

void AppConfig::reload()
{
    QSettings _config(config, QSettings::IniFormat);
    QStringList keys = _config.allKeys();

    for (int i = 0; i < keys.size(); ++i) {
        if (keys.at(i).startsWith("camera")) {
            _cameraTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        if (keys.at(i).startsWith("can")) {
            _canTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        if (keys.at(i).startsWith("lidar")) {
            _lidarTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        if (keys.at(i).startsWith("gps")) {
            _gpsTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        qDebug() << keys.at(i) << " = " << _config.value(keys.at(i)) << "\n";
    }

    _topicName = _config.value("topic_name", "").toString();
    _nodeName = _config.value("node_name", "").toString();
    _rootPath = _config.value("data_path", "").toString();
    _shareDirPath = _config.value("share_directory", "").toString();
    _rootPathNAS = _config.value("data_path_NAS", "").toString();
    _username = _config.value("user", "").toString();
    _password = _config.value("password", "").toString();
    _videoName = _config.value("video_name", "").toString();
    _videoSize = _config.value("video_size", "").toString();
    _videoFormat = _config.value("video_format", "").toString();
    _numberOfLogs = _config.value("number_of_logs", "3").toUInt();
    _logSize = _config.value("log_size", "5").toUInt();
    _logLevel = _config.value("log_level", "off").toString();
    _controllerNodeName = _config.value("controller_node_name", "").toString();

    QString keepData = _config.value("keep_kitti_data", "false").toString();
    _keepKittiData = keepData == "true" ? true : false;
}

AppConfig &AppConfig::instance()
{
    static AppConfig instance;
    return instance;
}

QString AppConfig::getLidarMappingValue(const QString &key)
{
    return _lidarTable.value(LIDAR_PREFIX + "_" + key, "");
}

QString AppConfig::getCameraMappingValue(const QString &key)
{
    return _cameraTable.value(CAMERA_PREFIX + "_" + key, "");
}

QString AppConfig::getGpsMappingValue(const QString &key)
{
    return _gpsTable.value(GPS_PREFIX + "_" + key, "");
}

QString AppConfig::getCanMappingValue(const QString &key)
{
    return _canTable.value(CAN_PREFIX + "_" + key, "");
}

QString AppConfig::getTopicName() const
{
    return _topicName;
}

QString AppConfig::getNodeName() const
{
    return _nodeName;
}

QString AppConfig::getVideoName() const
{
    return _videoName;
}

QString AppConfig::getRootPath() const
{
    return _rootPath;
}

QString AppConfig::getRootPathNAS() const
{
    return _rootPathNAS;
}

QString AppConfig::getUserName() const
{
    return _username;
}

QString AppConfig::getPassword() const
{
    return _password;
}

QString AppConfig::getVideoSize() const
{
    return _videoSize;
}

QString AppConfig::getVideoFormat() const
{
    return _videoFormat;
}

uint AppConfig::getNumberOfLogs() const
{
    return _numberOfLogs;
}

uint AppConfig::getLogSize() const
{
    return _logSize;
}

QString AppConfig::getLogLevel() const
{
    return _logLevel;
}

QString AppConfig::getControllerNodeName() const
{
    return _controllerNodeName;
}

QString AppConfig::getShareDirPath() const
{
    return _shareDirPath;
}

bool AppConfig::getKeepKittiData() const
{
    return _keepKittiData;
}
