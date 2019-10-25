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
            _cameras << _config.value(keys.at(i)).toString();
        }

        if (keys.at(i).startsWith("logger")) {
            _loggerNodes << _config.value(keys.at(i)).toString();
        }
    }

    _topicName = _config.value("topic_name", "").toString();
    _nodeName = _config.value("node_name", "").toString();

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

QStringList AppConfig::getCameras() const
{
    return _cameras;
}
