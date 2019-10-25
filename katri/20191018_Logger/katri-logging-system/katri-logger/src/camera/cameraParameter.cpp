#include "cameraParameter.h"

CameraParameter::CameraParameter(){}

CameraParameter::CameraParameter(const QString &id,
                                 const QString &src,
                                 const QString &size,
                                 const QString &format,
                                 const QString &fileName,
                                 const QString &path)
{
    _src = src;
    _size = size;
    _format = format;
    _path = path;
    _id = id;
    _fileName = fileName;
    _datasetIndex = 0;
}

CameraParameter::~CameraParameter(){

}

CameraParameter& CameraParameter::operator=(const CameraParameter & rhs) {
    _src = rhs._src;
    _size = rhs._size;
    _format = rhs._format;
    _path = rhs._path;
    _id = rhs._id;
    _fileName = rhs._fileName;
    _datasetIndex = rhs._datasetIndex;
    return *this;
}

void CameraParameter::setPath(const QString path){
    if (_path.compare(path) != 0) {
        _path = path;
        _datasetIndex = 0;
    }
    else {
        _datasetIndex++;
    }
}

QString CameraParameter::getPath() const{
    return _path;
}

void CameraParameter::setSrc(const QString src){
    _src = src;
}

QString CameraParameter::getSrc() const{
    return _src;
}

void CameraParameter::setSize(const QString size){
    _size = size;
}

QString CameraParameter::getSize() const{
    return _size;
}

void CameraParameter::setFormat(const QString format){
    _format = format;
}

QString CameraParameter::getFormat() const{
    return _format;
}

QString CameraParameter::getProgram() const{
    QFileInfo checkFile(_program);
    if (checkFile.exists() && checkFile.isFile()) {
        return checkFile.absoluteFilePath();
    }
    return _program;
}

QString CameraParameter::getId() const{
    return _id;
}

void CameraParameter::setId(const QString id){
    _id = id;
}

QString CameraParameter::getFileName() const{
    return _fileName;
}

void CameraParameter::setFileName(const QString fileName){
    _fileName = fileName;
}

QString CameraParameter::getDataSetPath() const
{
    QString dirPath;
    dirPath.append(_path).append("/");
    // The value uses to calculate storage path. So it must be set to the value getting
    // from application configuration with a pattern "camera_{id}"
    dirPath.append(AppConfig::instance().getCameraMappingValue(getId()));

    return dirPath;
}

QString CameraParameter::getTimestampPath() const
{
    QString fileName = QString("/timestamps_%1.txt").arg(_datasetIndex, 10, 10, QLatin1Char('0'));

    QString timestampPath;
    timestampPath.append(getDataSetPath()).append(fileName);

    return timestampPath;
}

QString CameraParameter::getFilePath() const
{
    QString fileName = QString("/data/%1.raw").arg(_datasetIndex, 10, 10, QLatin1Char('0'));

    QString dataPath;
    dataPath.append(getDataSetPath()).append(fileName);

    return dataPath;
}
