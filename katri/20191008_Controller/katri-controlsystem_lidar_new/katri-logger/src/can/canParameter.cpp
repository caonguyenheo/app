#include "canParameter.h"

CanParameter::CanParameter()
{

}

CanParameter::CanParameter(const QString &id,
                           const QString &src,
                           const QString &path)
{
    _src = src;
    _path = path;
    _id = id;
}

CanParameter::~CanParameter(){

}

CanParameter& CanParameter::operator=(const CanParameter & rhs) {
    _src = rhs._src;
    _path = rhs._path;
    _id = rhs._id;
    return *this;
}

void CanParameter::setPath(const QString &path)
{
    _path = path;
}

QString CanParameter::getPath() const{
    return _path;
}

void CanParameter::setSrc(const QString &src)
{
    _src = src;
}

QString CanParameter::getSrc() const{
    return _src;
}

QString CanParameter::getProgram() const{
    return _program;
}

QString CanParameter::getId() const{
    return _id;
}

void CanParameter::setId(const QString &id)
{
    _id = id;
}

void CanParameter::setFileName(const QString &name)
{
    _fileName = name;
}

QString CanParameter::getTimestampPath() const
{
    QString dirPath;
    return dirPath.append(_path).append("/")
            .append(AppConfig::instance().getCanMappingValue(getId()))
            .append("/timestamps.txt");
}

QString CanParameter::getDataPath() const
{
    QString dirPath;
    return dirPath.append(_path).append("/")
            .append(AppConfig::instance().getCanMappingValue(getId())).
            append("/data");
}

QString CanParameter::getFilePath() const
{
    QString dataPath;
    dataPath.append(getDataPath()).append("/").append(_fileName);
    return dataPath;
}

QString CanParameter::getCanDirectoryPath() const
{
    QString dirPath;
    dirPath.append(_path).append("/");
    dirPath.append(AppConfig::instance().getCanMappingValue(getId()));

    return dirPath;
}
