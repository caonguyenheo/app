#include "lidarParameter.h"
LidarParameter::LidarParameter() {}

LidarParameter::LidarParameter(const QString &id, const QString &path)
{
    _id = id;
    _path = path;
}

LidarParameter &LidarParameter::operator=(const LidarParameter &rhs)
{
    _id = rhs._id;
    _path = rhs._path;
    return *this;
}

QString LidarParameter::getPath() const
{
    return _path;
}
void LidarParameter::setPath(const QString &path)
{
    _path = path;
}

void LidarParameter::setId(const QString &id)
{
    _id = id;
}
QString LidarParameter::getId() const
{
    return _id;
}

QString LidarParameter::getDataPath() const
{
    QString dirPath;
    return dirPath.append(_path)
        .append("/")
        .append(AppConfig::instance().getLidarMappingValue(getId()))
        .append("/data");
}
QString LidarParameter::getTimestampPath() const
{
    QString dirPath;
    return dirPath.append(_path)
            .append("/")
            .append(AppConfig::instance().getLidarMappingValue(getId()))
            .append("/timestamps.txt");
}
QString LidarParameter::getDataFormatPath() const
{
    QString dirPath;
    return dirPath.append(_path)
        .append("/")
        .append(AppConfig::instance().getLidarMappingValue(getId()))
        .append("/dataformat.txt");
}
