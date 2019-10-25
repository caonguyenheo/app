#include "dataSourceManager.h"


const QString DATE_FORMAT = "yyyy_MM_dd";
const int PROCESS_TIMEOUT = 28800000; // 8h * 3600s * 1000ms

DataSourceManager::DataSourceManager()
{
    _logger = LoggerManager::getAppLogger();

    /*
    if (!m_imgExt) {
        m_imgExt = new ImageExtraction();
    }
    */
}

void DataSourceManager::copy(const Command _cmd)
{
    bool relayStatus = true;
    QString currentDate = QDate::currentDate().toString(DATE_FORMAT);

    if (_cmd.isRelayCommand() || _cmd.isStartNewKittiCommand())
    {
        if (_cmd.getType() == Command::Type::ALL)
        {
            pathkititar =AppConfig::instance().getRootPath().append("/").append(currentDate+".kat");
            qDebug()<<"pathkititar"<<pathkititar;
            QDir path_dir(pathkititar);
            if(!path_dir.exists()) {
                qDebug()<<"path not exists";
            }
            else {
                QString data_path = AppConfig::instance().getRootPath().append("/");
                QString data_path1 = AppConfig::instance().getRootPath();
                QString NodeName  = AppConfig::instance().getNodeName();
                qDebug()<<"path exists"<<data_path1;
                qDebug()<<"NodeName"<<NodeName;

                //this is mount from NAS
                QString sharedir_path = AppConfig::instance().getShareDirPath().append("/");

                if (QFile::exists(data_path+AppConfig::instance().getNodeName()+"_"+ currentDate +".tar"))
                {
                    QFile::remove(data_path+AppConfig::instance().getNodeName()+"_"+ currentDate +".tar");
                    qDebug()<<"Remove success at local";
                }
                if (QFile::exists(sharedir_path + AppConfig::instance().getNodeName() + "_" + currentDate + ".tar"))
                {
                    QFile::remove(sharedir_path + AppConfig::instance().getNodeName() + "_" + currentDate + ".tar");
                    qDebug()<<"Remove success at NAS";
                }
                /*
                //Find and Remove ALL File Raw
                QDirIterator it(pathkititar, QStringList() << "*.raw", QDir::Files, QDirIterator::Subdirectories);
                while (it.hasNext())
                {
                    QFile(it.next()).remove();
                }
                if (_logger) {
                    _logger->debug("Remove ALL FiLe Raw Done");
                }
                */

                // Create tar file
                QProcess tar;
                tar.setProcessChannelMode(QProcess::MergedChannels);
                QString program = "tar";
                QStringList arguments = {"-cf",data_path+AppConfig::instance().getNodeName()+"_"+ currentDate +".tar","-C",data_path1,currentDate+".kat"};
                qDebug() << arguments;
                tar.start(program, arguments);

                bool result = tar.waitForStarted() && tar.waitForFinished(PROCESS_TIMEOUT);
                if (!result)
                {
                    relayStatus = false;
                    if (_logger) {
                        _logger->debug("Create Tar file fail");
                    }
                    return;
                }
                else {
                    if (_logger) {
                        _logger->debug("Create Tar file successfuly");
                    }

                    // Copy file to NAS
                    QString localFile = data_path + AppConfig::instance().getNodeName()+"_"+ currentDate +".tar";
                    QString nasFile   = sharedir_path + AppConfig::instance().getNodeName()+"_"+ currentDate +".tar";
                    result = QFile::copy(localFile, nasFile);

                    // Remove local tar file
                    QFile(localFile).remove();

                    if (result) {
                        if (!AppConfig::instance().getKeepKittiData()) {
                            // Remove Kitti folder
                            path_dir.removeRecursively();
                            qDebug() << "Remove KITTI folder:" << pathkititar;
                        }
                        if (_logger) {
                            _logger->debug("Copy tar file to NAS successfuly");
                        }
                    }
                    else {
                        relayStatus = false;
                        if (_logger) {
                            _logger->debug("Copy tar file to NAS Fail");
                        }
                    }
                }
            }
            _tcpclient->relayState(relayStatus);
        }
    }
}

void DataSourceManager::_stopMe(Source *source, const Command _cmd)
{
    // Make sure instance available
    if (source) {
        // Stop anyway
        source->stop();

        if (_logger)QString::fromStdString(_cmd.getId());
            _logger->debug("_stopMe {} with Id {} is stopped.",
                           source->getName().toStdString(),
                           source->getId().toStdString());
    } else {
        if (_logger)
            _logger->debug("{} with Id {} is not available for stopping.",
                           Command::getTypeName(_cmd.getType()),
                           _cmd.getId());
    }
}

void DataSourceManager::_startMe(Source *source, const Command _cmd)
{
    // Make sure instance available
    if (source) {
        if (_cmd.isStartNewKittiCommand()) {
            if (source->isRunning()) {
                // Stop it
                source->stop();

                if (_logger)
                    _logger->debug("Stop NEW KITTI in _StartMe");
            }
        }

        if (source->isRunning()) {
            if (_logger)
                _logger->debug("{} with Id {} is {}. Make sure you stop it before starting.",
                               source->getName().toStdString(),
                               source->getId().toStdString(),
                               source->getStatus().toStdString());
            return;
        }

        // Set kitti path here
        source->setPath(_getKittiPath(_cmd));

        source->start();

        if (_logger)
            _logger->debug("Start Me {} with Id {} is started.",
                           source->getName().toStdString(),
                           source->getId().toStdString());
    } else {
        if (_logger)
            _logger->debug("{} with Id {} is not available for recording.",
                           Command::getTypeName(_cmd.getType()),
                           _cmd.getId());
    }
}

void DataSourceManager::stop(const Command _cmd)
{
    if (_cmd.getAction() == Command::Action::STOP) {
        if (_cmd.getType() == Command::Type::CAMERA) {
            Camera *cam = _cameraInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _stopMe(cam, _cmd);
            /*
            qDebug()<<"STOP----------------------------";
            //Send signal for extract frames from file raw
            if (m_imgExt) {
                if(cam) {
                    QString kittiFolderPath = cam->_parameters.getDataSetPath();
                    qDebug() << "kittiFolderPath =" << kittiFolderPath;
                    QStringList rawPathList;
                    QDirIterator it(kittiFolderPath, QStringList() << "*.raw", QDir::Files, QDirIterator::Subdirectories);
                    while (it.hasNext())
                        rawPathList << it.next();
                    qDebug() << "rawPathList =" << rawPathList;

                    m_imgExt->setPath(rawPathList);
                    emit m_imgExt->signalExtractframe();
                    if(_logger)
                        _logger->debug("Extracting RAW video");
                }
            }
            */

        } else if (_cmd.getType() == Command::Type::CAN) {
            Can *can = _canInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _stopMe(can, _cmd);
        } else if (_cmd.getType() == Command::Type::LIDAR) {
#if ENABLE_ROS_VERSION == 1
            Lidar *lidar = _lidarInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _stopMe(lidar, _cmd);
#endif
        } else if (_cmd.getType() == Command::Type::GNSS) {
#if ENABLE_ROS_VERSION == 1
            Gps *gps = _gpsInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _stopMe(gps, _cmd);
#endif
        }
    }
}

void DataSourceManager::start(const Command _cmd)
{
    if (_cmd.isStartCommand() || _cmd.isStartNewKittiCommand()) {
        if (_cmd.getType() == Command::Type::CAMERA) {
            // Start only Camera process instance
            Camera *cam = _cameraInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            //qDebug()<<"START----------------------------";
            _startMe(cam, _cmd);

        } else if (_cmd.getType() == Command::Type::CAN) {
            // Start only CAN process instance
            Can *can = _canInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _startMe(can, _cmd);

        } else if (_cmd.getType() == Command::Type::LIDAR) {
#if ENABLE_ROS_VERSION == 1
            Lidar *lidar = _lidarInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _startMe(lidar, _cmd);
#endif
        } else if (_cmd.getType() == Command::Type::GNSS) {
#if ENABLE_ROS_VERSION == 1
            Gps *gps = _gpsInstances.value(QString::fromStdString(_cmd.getId()), nullptr);
            _startMe(gps, _cmd);
#endif
        }
    }
}

DataSourceManager &DataSourceManager::lookup()
{
    // Look up Camera sources
    {
        QSettings *cameraSource = new QSettings(cameraSourceConfig, QSettings::IniFormat);
        QHash<QString, QString> cameraIOSources;

        QStringList keys = cameraSource->allKeys();
        for (int i = 0; i < keys.size(); ++i) {
            cameraIOSources.insert(keys.at(i), cameraSource->value(keys.at(i)).toString());
        }

        if (keys.size() > 0) {
            _sources.insert(CAMERA, cameraIOSources);
        }
        delete cameraSource;
    }

    // Look up CAN sources
    {
        QSettings *canSource = new QSettings(canSourceConfig, QSettings::IniFormat);
        QHash<QString, QString> canIOSources;

        QStringList keys = canSource->allKeys();
        for (int i = 0; i < keys.size(); ++i) {
            canIOSources.insert(keys.at(i), canSource->value(keys.at(i)).toString());
        }

        if (keys.size() > 0) {
            _sources.insert(CAN, canIOSources);
        }
        delete canSource;
    }

#if ENABLE_ROS_VERSION == 1
    // Look up Lidar sources
    {
        QSettings *lidarSource = new QSettings(lidarSourceConfig, QSettings::IniFormat);
        QHash<QString, QString> lidarIOSources;

        QStringList keys = lidarSource->allKeys();
        for (int i = 0; i < keys.size(); ++i) {
            lidarIOSources.insert(keys.at(i), lidarSource->value(keys.at(i)).toString());
        }

        if (keys.size() > 0) {
            _sources.insert(LIDAR, lidarIOSources);
        }
        delete lidarSource;
    }
#endif

#if ENABLE_ROS_VERSION == 1
    // Look up Gps sources
    {
        QSettings *gpsSource = new QSettings(gpsSourceConfig, QSettings::IniFormat);
        QHash<QString, QString> gpsIOSources;

        QStringList keys = gpsSource->allKeys();
        for (int i = 0; i < keys.size(); ++i) {
            gpsIOSources.insert(keys.at(i), gpsSource->value(keys.at(i)).toString());
        }

        if (keys.size() > 0) {
            _sources.insert(GPS, gpsIOSources);
        }

        delete gpsSource;
    }
#endif

    return *this;
}

QString DataSourceManager::_getKittiPath(const Command _cmd)
{
    Runtime runtime;
    // Identify Kitti path for saving the data
    if (_cmd.isStartNewKittiCommand() || _cmd.isStartCommand()) {
        QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
        QString path;
        path.append(currentDate)
            .append(".kat")
            .append("/")
            .append(currentDate)
            .append("_drive_")
            .append(runtime.formatSequence(_cmd.getKittiNo()));

        return AppConfig::instance().getRootPath().append("/").append(path);
    }

    return "";
}

DataSourceManager &DataSourceManager::createSources()
{
    QHashIterator<int, QHash<QString, QString>> sourceIterator(_sources);
    while (sourceIterator.hasNext()) {
        sourceIterator.next();
        QHashIterator<QString, QString> j(sourceIterator.value());
        while (j.hasNext()) {
            j.next();
            QString id = j.key();
            QString src = j.value();

            QHash<QString, QString> param;
            param.insert(ID, id);
            param.insert(SRC, src);

            if (sourceIterator.key() == CAMERA) {
                // Create Camera instance if not available
                if (!_cameraInstances.contains(id)) {
                    DataSourceFactory<Camera, CameraParameter> factory;
                    Camera *cam = factory.createSourceInstance(param);
                    cam->setNode(_node);
                    _cameraInstances.insert(id, cam);
                }
            } else if (sourceIterator.key() == CAN) {
                // Create CAN instance if not available
                if (!_canInstances.contains(id)) {
                    DataSourceFactory<Can, CanParameter> factory;
                    Can *can = factory.createSourceInstance(param);

                    _canInstances.insert(id, can);
                }
            } else if (sourceIterator.key() == LIDAR) {
#if ENABLE_ROS_VERSION == 1
                // Create Lidar instance
                DataSourceFactory<Lidar, LidarParameter> factory;
                Lidar *lidar = factory.createSourceInstance(param);
                _lidarInstances.insert(id, lidar);
#endif
            } else if (sourceIterator.key() == GPS) {
#if ENABLE_ROS_VERSION == 1
                // Create Gps instance
                DataSourceFactory<Gps, GpsParameter> factory;
                Gps *gps = factory.createSourceInstance(param);
                _gpsInstances.insert(id, gps);
#endif
            }
        }
    }
    return *this;
}

DataSourceManager &DataSourceManager::instance()
{
    static DataSourceManager instance;
    return instance;
}

void DataSourceManager::setNode(NodeWorker *node)
{
    _node = node;
}

void DataSourceManager::setSocket(NodeWorker *tcpclient)
{
    _tcpclient = tcpclient;
};

/*
void DataSourceManager::CheckNumberframe(QString Path_folder, QString Path_timestamp, QString Filename)
{
    //Count number file in folder
    QDir dir(Path_folder);
    dir.setFilter( QDir::AllEntries | QDir::NoDotAndDotDot );
    uint total_files = dir.count(); // -1 -> file.raw

    //Count number line in file timestamp
    QFile file(Path_timestamp);
    uint line_count = 0;
    file.open(QIODevice::ReadOnly); //| QIODevice::Text)
    QString line[100];
    QTextStream in(&file);
    while( !in.atEnd()){
        line[line_count]=in.readLine();
        line_count++;
    }
    qDebug()<<"total_line = "<<total_files;

    qDebug()<<"line_count = "<<line_count;

    if(total_files != line_count){
        qDebug()<<"Extract Frame error";
    }
    else{
        qDebug()<<"Extract Frame OK";
        //Remove file.RAW
        QFile(Filename).remove();
    }
}
*/
