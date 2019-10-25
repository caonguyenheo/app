#include "gps.h"
#include "common_gps.h"

#if ENABLE_ROS_VERSION == 1
Gps::Gps(const GpsParameter &param, QObject *parent)
    : Source(parent)
{
    _parameters = param;
    _logger = LoggerManager::getGpsLogger(_parameters.getId());

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, QOverload<>::of(&Gps::slotWriteBufferToFile));
}

Gps::~Gps() {}

void Gps::_initRuntimeFile()
{
    QString runtimeFile;
    runtimeFile.append("gps").append(_parameters.getId()).append(".runtime");

    QString key = _parameters.getDataPath();
    key.remove("/");
    // Create gpsxxx.runtime if it is not available
    if (!QFile::exists(runtimeFile)) {
        QSettings runtime(runtimeFile, QSettings::IniFormat);
        // The next index for data file will be 0. Therefore, set -1 to dataPath as a key.
        runtime.setValue(key, "-1");
    }
}

void Gps::_updateStatus(bool status)
{
    QString runtimeFile;
    runtimeFile.append("gps").append(_parameters.getId()).append(".runtime");

    if (getId() == "000") {
        Node1Worker::gpsRuntime = runtimeFile;
        Node1Worker::gpsDataPath = _parameters.getDataFilePath();
        Node1Worker::gpsTimestampPath = _parameters.getTimestampPath();
        Node1Worker::startGps = status;
        Node1Worker::gpsInsstdevValue = "null null null null null null";
        Node1Worker::gpsRowCount = 0;
    }
}

void Gps::_createOutputDirectory()
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
void Gps::_writeDataFormatGPS()
{
    QString gpsDataformatPath =_parameters.getDataformatPath();
    QFile dataformatFile(gpsDataformatPath);
    if (dataformatFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&dataformatFile);
        if (_logger) {
            _logger->debug("Successfuly to open: " + gpsDataformatPath.toStdString());
        }

        for (int i=0; i<dataformat.size(); i++) {
            out << dataformat.at(i) << "\n";
        }

        if (_logger) {
            _logger->debug("Successfuly Write To File Dataformat");
        }

        qDebug()<<"Successfuly Open And Write To File Dataformat";

        dataformatFile.close();
    }
    else {
        if (_logger) {
            _logger->debug("Fail to open: " + gpsDataformatPath.toStdString());
        }
        qDebug()<<"Fail to open dataformat";
    }

}

// Each 1s, use timer to write buffer data
void Gps::slotWriteBufferToFile()
{
    QStringList tsList;
    QStringList dataList;
    Node1Worker::getGpsBuffer(tsList, dataList);

    // Write timestamp buffer to timestamps.txt file
    if (tsList.count() > 0) {
        QFile timestampFile(Node1Worker::gpsTimestampPath);
        if (timestampFile.open(QIODevice::Append | QIODevice::Text)) {
            QTextStream out(&timestampFile);
            out << tsList.join("\n");
            out << "\n";
            timestampFile.close();
        }
        else {
            _logger->debug("Fail to open: " + Node1Worker::gpsTimestampPath.toStdString());
        }
    }

    // Write timestamp buffer to timestamps.txt file
    if (dataList.count() > 0) {
        QFile dataFile(Node1Worker::gpsDataPath);
        if (dataFile.open(QIODevice::Append | QIODevice::Text)) {
            QTextStream out(&dataFile);
            out << dataList.join("\n");
            out << "\n";
            dataFile.close();
        }
        else {
            _logger->debug("Fail to open: " + Node1Worker::gpsDataPath.toStdString());
        }
    }
}

void Gps::start()
{
    _initRuntimeFile();

    _createOutputDirectory();

    _updateStatus(true);

    _writeDataFormatGPS();

    if (m_timer) {
        m_timer->start(1000);
    }
}

void Gps::stop()
{
    _updateStatus(false);

    // Stop timer and save last section
    if (m_timer) {
        m_timer->stop();
    }
    QTimer::singleShot(500, this, QOverload<>::of(&Gps::slotWriteBufferToFile));
}

void Gps::setPath(const QString &path)
{
    _parameters.setPath(path);
}

QString Gps::getId()
{
    return _parameters.getId();
}

bool Gps::isRunning()
{
    if (getId() == "000") {
        return Node1Worker::startGps;
    }
    return false;
}

QString Gps::getName()
{
    return "Gps";
}

QString Gps::getStatus()
{
    return (isRunning()) ? "Running" : "Stopped";
}
#endif
