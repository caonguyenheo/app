#include "lidar.h"
#if ENABLE_ROS_VERSION == 1
Lidar::Lidar() {}

Lidar::Lidar(const LidarParameter &param, QObject *parent)
    : Source(parent)
{
    _parameters = param;
    _logger = LoggerManager::getLidarLogger(_parameters.getId());
}

Lidar::~Lidar() {}

void Lidar::_initRuntimeFile()
{
    QString runtimeFile;
    runtimeFile.append("lidar").append(_parameters.getId()).append(".runtime");

    QString key = _parameters.getDataPath();
    key.remove("/");
    // Create lidarxxx.runtime if it is not available
    if (!QFile::exists(runtimeFile)) {
        QSettings runtime(runtimeFile, QSettings::IniFormat);
        // The next index for data file will be 0. Therefore, set -1 to dataPath as a key.
        runtime.setValue(key, "-1");
    }
}

void Lidar::_updateStatus(bool status)
{
    QString runtimeFile;
    runtimeFile.append("lidar").append(_parameters.getId()).append(".runtime");

    if (getId() == "000") {
        Node1Worker::velodyneLidar1Runtime = runtimeFile;
        Node1Worker::velodyneLidar1DataPath = _parameters.getDataPath();
        Node1Worker::velodyneLidar1TimestampPath = _parameters.getTimestampPath();
        Node1Worker::startVelodyneLidar1 = status;
    } else if (getId() == "001") {
        Node1Worker::velodyneLidar2Runtime = runtimeFile;
        Node1Worker::velodyneLidar2DataPath = _parameters.getDataPath();
        Node1Worker::velodyneLidar2TimestampPath = _parameters.getTimestampPath();
        Node1Worker::startVelodyneLidar2 = status;
    } else if (getId() == "002") {
        Node1Worker::ousterLidarRuntime = runtimeFile;
        Node1Worker::ousterLidarDataPath = _parameters.getDataPath();
        Node1Worker::ousterLidarTimestampPath = _parameters.getTimestampPath();
        Node1Worker::startOusterLidar = status;
    }
}

void Lidar::_createOutputDirectory()
{
    QString directory = _parameters.getDataPath();
    QDir dir(directory);
    if (!dir.exists()) {
        if (dir.mkpath(directory)) {
            if (_logger)
                _logger->debug("Generated the data directory: {}", directory.toStdString());

        } else {
            if (_logger)
                _logger->debug("Cannot create data directory: {}", directory.toStdString());
        }
    }
}

void Lidar::start()
{
    _initRuntimeFile();

    _createOutputDirectory();

    _updateStatus(true);
}

void Lidar::stop()
{
    _updateStatus(false);
}

void Lidar::setPath(const QString &path)
{
    _parameters.setPath(path);
}

QString Lidar::getId()
{
    return _parameters.getId();
}

bool Lidar::isRunning()
{
    if (getId() == "000") {
        return Node1Worker::startVelodyneLidar1;
    } else if (getId() == "001") {
        return Node1Worker::startVelodyneLidar2;
    } else if (getId() == "002") {
        return Node1Worker::startOusterLidar;
    }
    return false;
}

QString Lidar::getName()
{
    return "Lidar";
}

QString Lidar::getStatus()
{
    return (isRunning()) ? "Running" : "Stopped";
}
#endif
