#include "appConfig.h"
#include <QDate>

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

    // NAS properties
    _nasDataPath = _config.value("nas_data_path", "").toString();

    // LOAD TO configuration
    _loadToDate = _config.value("load_to_date", LOAD_TO_CURRENT).toString().toLower();
    if ( _loadToDate.compare(LOAD_TO_ALL) == 0
      || _loadToDate.compare(LOAD_TO_CURRENT) == 0 ) {
        return;
    }
    else if (!QDate::fromString(_loadToDate, LOAD_TO_DATE_FORMAT).isValid()) {
        _loadToDate = LOAD_TO_CURRENT;
    }
}

AppConfig &AppConfig::instance()
{
    static AppConfig instance;
    return instance;
}

// NAS properties
QString AppConfig::getNasDataPath() const
{
    return _nasDataPath;
}

// LOAD TO configuration
QString AppConfig::getLoadToDate() const
{
    return _loadToDate;
}
