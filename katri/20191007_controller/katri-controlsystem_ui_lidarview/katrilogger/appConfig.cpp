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
            _loggerNodes << _config.value(keys.at(i)).toString();
        }
    }

    _ros2Enable = _config.value("ros2_enable", "").toBool();
    _nodeName   = _config.value("node_name", "").toString();
    _topicName  = _config.value("topic_name", "").toString();

    _relayTimeout = _config.value("relay_timeout", "600000").toInt();
    _loggerRespondTimeout = _config.value("logger_respond_timeout", "60000").toInt();
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
