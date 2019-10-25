#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "QSettings"

namespace appconfig {
const QString config = "config.properties";

const QString LOAD_TO_ALL         = "all";
const QString LOAD_TO_CURRENT     = "current";
const QString LOAD_TO_DATE_FORMAT = "yyyy_MM_dd";

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

    // NAS properties
    QString _nasDataPath;

    // LOAD TO configuration
    QString _loadToDate;

public:
    void reload();

    // NAS configuration
    QString getNasDataPath() const;

    // LOAD TO configuration
    QString getLoadToDate() const;
};

#endif // APPCONFIG_H
