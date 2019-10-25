#include "gpsParameter.h"
GpsParameter::GpsParameter() {}

GpsParameter::GpsParameter(const QString &id, const QString &path)
{
    _id = id;
    _path = path;
}

GpsParameter &GpsParameter::operator=(const GpsParameter &rhs)
{
    _id = rhs._id;
    _path = rhs._path;
    return *this;
}

QString GpsParameter::getPath() const
{
    return _path;
}
void GpsParameter::setPath(const QString &path)
{
    _path = path;
}

void GpsParameter::setId(const QString &id)
{
    _id = id;
}
QString GpsParameter::getId() const
{
    return _id;
}

QString GpsParameter::getDataPath() const
{
    QString dirPath;
    return dirPath.append(_path)
        .append("/")
        .append(AppConfig::instance().getGpsMappingValue(getId()))
        .append("/data");
}

QString GpsParameter::getDataFilePath() const
{
    QString dirPath;
    return dirPath.append(_path)
        .append("/")
        .append(AppConfig::instance().getGpsMappingValue(getId()))
        .append("/data")
        .append("/gnss.txt");
}

QString GpsParameter::getTimestampPath() const
{
    QString dirPath;
    return dirPath.append(_path)
        .append("/")
        .append(AppConfig::instance().getGpsMappingValue(getId()))
        .append("/timestamps.txt");
}

QString GpsParameter::getDataformatPath() const
{
    QString dirPath;
    return dirPath.append(_path)
            .append("/")
            .append(AppConfig::instance().getGpsMappingValue(getId()))
            .append("/dataformat.txt");
}
