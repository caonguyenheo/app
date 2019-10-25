#include "appConfig.h"
#include <QDebug>
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
            _loggerNodes << _config.value(keys.at(i)).toString();
        }
    }

    for (int i = 0; i < keys.size(); ++i) {
        if (keys.at(i).startsWith("cam_")) {
            _portTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        if (keys.at(i).startsWith("lidar_")) {
            _portTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }

        if (keys.at(i).startsWith("GNSS_")) {
            _portTable.insert(keys.at(i), _config.value(keys.at(i)).toString());
        }
    }

    _ros2Enable = _config.value("ros2_enable", "").toBool();
    _nodeName   = _config.value("node_name", "").toString();
    _topicName  = _config.value("topic_name", "").toString();

    _relayTimeout = _config.value("relay_timeout", "600000").toInt();
    _loggerRespondTimeout = _config.value("logger_respond_timeout", "60000").toInt();
    _ipserver = _config.value("ip", "").toString();
    _port = _config.value("port", "").toString();

}

AppConfig &AppConfig::instance()
{
    static AppConfig instance;
    return instance;
}

QString AppConfig::getTopicName() const
{
    return _topicName;
}

QString AppConfig::getNodeName() const
{
    return _nodeName;
}

QStringList AppConfig::getLoggerNodes()
{
    return _loggerNodes;
}

bool AppConfig::getRos2Enable()
{
    return _ros2Enable;
}

int AppConfig::getRelayTimeout()
{
    return _relayTimeout;
}

int AppConfig::getLoggerRespondTimeout()
{
    return _loggerRespondTimeout;
}

QString AppConfig::getIP() const
{
    return _ipserver;
}

QString AppConfig::getPort() const
{
    return _port;
}

QString AppConfig::getHostForDevice(uint8_t devType,const QString &devId)
{
    QString key = "";
    switch(devType) {
        case 0x01: //CAN
            break;
        case 0x02: //CAMERA
            key = CAMERA_PREFIX + "_" + devId;
            break;
        case 0x03: //LIDAR
            key = LIDAR_PREFIX + "_" + devId;
            break;
        case 0x04: //RADAR
            break;
        case 0x05: //MOBILEYE
            break;
        case 0x06: //GNSS
            key = GPS_PREFIX + "_" + devId;
            break;
        default:
            break;
    }

    return _portTable.value(key, "");
}
