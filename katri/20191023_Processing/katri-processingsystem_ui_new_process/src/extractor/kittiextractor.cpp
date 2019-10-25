#include "kittiextractor.h"
#include <QDir>
#include <QThread>
#include <QDebug>
#include "common.h"

KittiExtractor::KittiExtractor(QObject *parent,
                               QString basedKittiPath,
                               QString extKittiPath)
    : QObject (parent)
    , _isExtracting(false)
    , _basedKittiPath()
    , _extractionKittiPath()
{
    setBasedKittiPath(basedKittiPath);
    setExtractionKittiPath(extKittiPath);
}

KittiExtractor::~KittiExtractor()
{
}

bool KittiExtractor::setBasedKittiPath(QString basedKittiPath)
{
    if (QDir(basedKittiPath).exists()) {
        _basedKittiPath = basedKittiPath;
        return true;
    }
    else {
        qDebug() << "Not existed KITTI data path" << basedKittiPath;
        return false;
    }
}

bool KittiExtractor::setExtractionKittiPath(QString extKittiPath)
{
    QDir dir(extKittiPath);
    if (dir.exists()) {
        _extractionKittiPath = "";
        qDebug() << "Existed, can not extracting KITTI data to" << extKittiPath;
        return false;
    }
    else {
        if (dir.mkpath(extKittiPath)) {
            _extractionKittiPath = extKittiPath;
            return true;
        }
        else {
            _extractionKittiPath = "";
            qDebug() << "Can not creating KITTI data path" << extKittiPath;
            return false;
        }
    }
}

bool KittiExtractor::execExtracting(QString start, QString end)
{
    if (_basedKittiPath.compare("") != 0 && _extractionKittiPath.compare("") != 0) {
        QDateTime tmpStart = QDateTime::fromString(start, TIMESTAMP_FORMAT);
        QDateTime tmpend   = QDateTime::fromString(end,   TIMESTAMP_FORMAT);
        if ( tmpStart.isNull() == false && tmpStart.isValid()
          && tmpend.isNull()   == false && tmpend.isValid()
        ) {
            qDebug() << "Extracting data between" << start << "and" << end;
            return this->execExtracting(tmpStart, tmpend);
        }
        else {
            qDebug() << "Can not extract data. Start point =" << start << "End point =" << end;
            return false;
        }
    }
    else {
        return false;
    }
}

bool KittiExtractor::execExtracting(QDateTime start, QDateTime end)
{
    //  Scan list of KITRI dataset
    //  Foreach (KITTI dataset) {
    //      Scan list of data source
    //      Foreach (data source) {
    //          Call dataSourceExtractor()
    //      }
    //  }

    if (_basedKittiPath.compare("") != 0 && _extractionKittiPath.compare("") != 0) {
        // Camera
        const QStringList cameraFilters("image_*");
        QStringList cameraList = QDir(_basedKittiPath).entryList(cameraFilters, QDir::Dirs);
        Q_FOREACH(QString camPath, cameraList) {
            camPath = _basedKittiPath + "/" + camPath;
            qDebug() << camPath;
            multiDataExtractor(camPath, start, end);
        }

        // GPS
        const QStringList gpsFilters("gnss_*");
        QStringList gpsList = QDir(_basedKittiPath).entryList(gpsFilters, QDir::Dirs);
        Q_FOREACH(QString gpsPath, gpsList) {
            gpsPath = _basedKittiPath + "/" + gpsPath;
            qDebug() << gpsPath;
            singleDataExtractor(gpsPath, start, end);
        }

        // Lidar
        const QStringList velodyneFilters("vlp16_*");
        QStringList velodyneList = QDir(_basedKittiPath).entryList(velodyneFilters, QDir::Dirs);
        Q_FOREACH(QString velodynePath, velodyneList) {
            velodynePath = _basedKittiPath + "/" + velodynePath;
            qDebug() << velodynePath;
            multiDataExtractor(velodynePath, start, end);
        }

        // Ouster
        const QStringList ousterFilters("os1_*");
        QStringList ousterList = QDir(_basedKittiPath).entryList(ousterFilters, QDir::Dirs);
        Q_FOREACH(QString ousterPath, ousterList) {
            ousterPath = _basedKittiPath + "/" + ousterPath;
            qDebug() << ousterPath;
            multiDataExtractor(ousterPath, start, end);
        }

        // Radar
        const QStringList radarFilters("radar_*");
        QStringList radarList = QDir(_basedKittiPath).entryList(radarFilters, QDir::Dirs);
        Q_FOREACH(QString radarPath, radarList) {
            radarPath = _basedKittiPath + "/" + radarPath;
            qDebug() << radarPath;
            multiDataExtractor(radarPath, start, end);
        }

        // CCan
        const QStringList ccanFilters("ccan");
        QStringList ccanList = QDir(_basedKittiPath).entryList(ccanFilters, QDir::Dirs);
        Q_FOREACH(QString ccanPath, ccanList) {
            ccanPath = _basedKittiPath + "/" + ccanPath;
            qDebug() << ccanPath;
            singleDataExtractor(ccanPath, start, end);
        }
        return true;
    }
    else {
        return false;
    }
}

bool KittiExtractor::multiDataExtractor(QString orgDSPath, QDateTime start, QDateTime end)
{
    // Destination data source path
    QString destDSPath = orgDSPath;
    destDSPath.replace(_basedKittiPath, _extractionKittiPath);
    destDSPath.append("/data/");

    // Get timestamps*.txt list
    const QStringList tsNameFilters("timestamps*.txt");
    QStringList tsFileNameList = QDir(orgDSPath).entryList(tsNameFilters, QDir::Files);
    Q_FOREACH(QString orgTsPath, tsFileNameList) {
        int orgDataIndex  = -1;
        int destDataIndex = -1;

        // Get timestamps.txt path
        orgTsPath = orgDSPath + "/" + orgTsPath;
        //qDebug() << "tsFilePath =" << orgTsPath;

        QString destTsPath = orgTsPath;
        destTsPath.replace(_basedKittiPath, _extractionKittiPath);
        //qDebug() << "tsDestinationPath =" << destTsPath;

        // Read timestamp list
        QStringList timestampList;
        readFileBuffer(orgTsPath.toUtf8().data(), timestampList);
        Q_FOREACH(QString timestamp, timestampList) {
            QString tmp = timestamp;
            if (tmp.replace(" ","").compare("") != 0) // Remove all blank data row
            {
                orgDataIndex++;
                QString truncTimestamp = timestamp;
                truncTimestamp.truncate(TIMESTAMP_FORMAT.length());

                // Read timestamp verify it in Range
                QDateTime dateTime = QDateTime::fromString(truncTimestamp, TIMESTAMP_FORMAT);
                if (start <= dateTime && dateTime <= end) {
                    destDataIndex++;

                    // Create destination dir
                    QDir dir(destDSPath);
                    bool existDestDir = dir.exists();
                    if (!existDestDir) {
                        existDestDir = dir.mkpath(destDSPath);
                    }

                    if (existDestDir) {
                        // Copy timestamp to destination
                        QFile destTsFile(destTsPath);
                        if (destTsFile.open(QIODevice::Append | QIODevice::Text)) {
                            QTextStream out(&destTsFile);
                            out << timestamp << "\n";
                            destTsFile.close();
                            qDebug() << "Write timestamp to" << destTsPath;
                        }
                        else {
                            qDebug() << "Can not open" << destTsPath;
                        }

                        QString indexString = destTsPath.mid(destTsPath.lastIndexOf("/") + 1);
                        indexString.replace(".txt", "");
                        indexString.replace("timestamps", "");
                        indexString.replace("_", "");

                        if (indexString.length() == 10) {
                            indexString.append("_");
                        }
                        else {
                            indexString = "";
                        }

                        // Copy frame data to destimation
                        QString oldName = indexString + QString("%1").arg(orgDataIndex, 10, 10, QLatin1Char('0'));
                        QString destName = indexString + QString("%1").arg(destDataIndex, 10, 10, QLatin1Char('0'));

                        const QStringList frameFileFilters
                                = {oldName+".png", oldName+".txt", oldName+".bin"};
                        QStringList frameFileList = QDir(orgDSPath + "/data/").entryList(frameFileFilters, QDir::Files);
                        Q_FOREACH(QString frameName, frameFileList) {
                            QString orgFramePath = orgDSPath + "/data/" + frameName;
                            QString destFramePath = destDSPath + frameName.replace(oldName, destName);
                            if (QFile::copy(orgFramePath, destFramePath)) {
                                qDebug() << "Copy success:" << orgFramePath << "=>" << destFramePath;
                            }
                            else {
                                qDebug() << "Copy Error:" << orgFramePath << "=>" << destFramePath;
                                return false;
                            }
                        }
                    }
                    else {
                        qDebug() << "Can not access to" << destDSPath;
                        return false;
                    }
                }
                else {
                    qDebug() << "Out of Range" << timestamp;
                }
            }
        }
    }
    return true;
}

bool KittiExtractor::singleDataExtractor(QString orgPath, QDateTime start, QDateTime end)
{
    // Destination data source path
    QString destDSPath = orgPath;
    destDSPath.replace(_basedKittiPath, _extractionKittiPath);
    destDSPath.append("/data/");

    // Get timestamps.txt file
    int orgDataIndex  = -1;

    // Get timestamps.txt path
    QString orgTsPath = orgPath + "/" + "timestamps.txt";
    QString destTsPath = orgTsPath;
    destTsPath.replace(_basedKittiPath, _extractionKittiPath);

    // Get data path
    QString dataFile = "0000000000.txt";
    {
        QString tmp = orgPath;
        tmp.replace(_basedKittiPath, "");
        if (tmp.contains("ccan"))
            dataFile = "ccan.txt";
        if (tmp.contains("gnss"))
            dataFile = "gnss.txt";
    }

    QString orgDataPath = orgPath + "/data/" + dataFile;
    QString destDataPath = orgDataPath;
    destDataPath.replace(_basedKittiPath, _extractionKittiPath);

    // Read timestamp list
    QStringList timestampList = readFileBuffer(orgTsPath.toUtf8().data());
    QStringList dataList      = readFileBuffer(orgDataPath.toUtf8().data());
    for(int i = 0; i < timestampList.count(); i++) {
        QString tmp = timestampList[i];
        if (tmp.replace(" ","").compare("") != 0) // Remove all blank data row
        {
            orgDataIndex++;
            QString truncTimestamp = timestampList[i];
            truncTimestamp.truncate(TIMESTAMP_FORMAT.length());

            // Read timestamp verify it in Range
            QDateTime dateTime = QDateTime::fromString(truncTimestamp, TIMESTAMP_FORMAT);
            if (start <= dateTime && dateTime <= end) {
                // Create destination dir
                QDir dir(destDSPath);
                bool existDestDir = dir.exists();
                if (!existDestDir) {
                    existDestDir = dir.mkpath(destDSPath);
                }

                if (existDestDir) {
                    // Copy timestamp to destination
                    QFile destTsFile(destTsPath);
                    if (destTsFile.open(QIODevice::Append | QIODevice::Text)) {
                        QTextStream out(&destTsFile);
                        out << timestampList[i] << "\n";
                        destTsFile.close();
                        qDebug() << "Write timestamp to" << destTsPath;
                    }
                    else {
                        qDebug() << "Can not open" << destTsPath;
                    }

                    // Copy data to destination
                    QFile destDataFile(destDataPath);
                    if (destDataFile.open(QIODevice::Append | QIODevice::Text)) {
                        QTextStream out(&destDataFile);
                        out << dataList[i] << "\n";
                        destDataFile.close();
                        qDebug() << "Write data to" << destDataPath;
                    }
                    else {
                        qDebug() << "Can not open" << destDataPath;
                    }
                }
                else {
                    qDebug() << "Can not access to" << destDSPath;
                    return false;
                }
            }
            else {
                qDebug() << "Out of Range" << timestampList[i];
            }
        }
    }
    return true;
}
