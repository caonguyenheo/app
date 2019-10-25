#include "node1Worker.h"

#if ENABLE_ROS_VERSION == 1
bool Node1Worker::startVelodyneLidar1 = false;
bool Node1Worker::startVelodyneLidar2 = false;
bool Node1Worker::startOusterLidar = false;
QString Node1Worker::velodyneLidar1DataPath = "";
QString Node1Worker::velodyneLidar2DataPath = "";
QString Node1Worker::ousterLidarDataPath = "";
QString Node1Worker::velodyneLidar1TimestampPath = "";
QString Node1Worker::velodyneLidar2TimestampPath = "";
QString Node1Worker::ousterLidarTimestampPath = "";
QString Node1Worker::velodyneLidar1Runtime = "";
QString Node1Worker::velodyneLidar2Runtime = "";
QString Node1Worker::ousterLidarRuntime = "";

bool Node1Worker::startGps = false;
QString Node1Worker::gpsRuntime = "";
QString Node1Worker::gpsDataPath = "";
QString Node1Worker::gpsTimestampPath = "";
QString Node1Worker::gpsInsstdevValue = "null null null null null null";
QStringList Node1Worker::gpsTimestampBuffer;
QStringList Node1Worker::gpsDataBuffer;
QMutex Node1Worker::_mutexGps;

int Node1Worker::gpsRowCount = 0;

Node1Worker::Node1Worker(int argc, char **argv, QObject *parent)
    : QThread(parent)
{
    _argc = argc;
    _argv = argv;

    _logger = LoggerManager::getAppLogger();
}

Node1Worker::~Node1Worker()
{
    delete _argv;
}

void Node1Worker::recvPointCloud2VelodyneLidar1Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (startVelodyneLidar1) {
        QMutexLocker lock(&_mutexVelodyne1);
        _writeLidarData(*msg, velodyneLidar1Runtime, velodyneLidar1TimestampPath, velodyneLidar1DataPath);
    }
}

void Node1Worker::recvPointCloud2VelodyneLidar2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (startVelodyneLidar2) {
        QMutexLocker lock(&_mutexVelodyne2);

        _writeLidarData(*msg, velodyneLidar2Runtime, velodyneLidar2TimestampPath, velodyneLidar2DataPath);
    }
}

void Node1Worker::recvPointCloud2OusterCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (startOusterLidar) {
        QMutexLocker lock(&_mutexOuster);
        _writeLidarData(*msg, ousterLidarRuntime, ousterLidarTimestampPath, ousterLidarDataPath);
    }
}

void Node1Worker::_writeLidarData( const sensor_msgs::PointCloud2 msg,
                                   QString runtimeFile,
                                   QString timestampPath,
                                   QString dataPath )
{
    int index = 0;
    QString key = dataPath;
    key.remove("/");
    QSettings runtime(runtimeFile, QSettings::IniFormat);
    if (runtime.contains(key)) {
        index = runtime.value(key).toInt() + 1;
    }

    // Update current index
    runtime.clear(); // clear to store only index of current data path
    runtime.setValue(key, QString::number(index));

    sensor_msgs::PointCloud pointCloud;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pointCloud);

    // Write timestamp to timestamps.txt file
    QFile timestampFile(timestampPath);
    if (timestampFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&timestampFile);

        //get timestamp
        uint64_t nsec = pointCloud.header.stamp.toNSec();
        QDateTime t = QDateTime::fromMSecsSinceEpoch(nsec / 1000000);

        //convert to time format
        out << t.toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
        timestampFile.close();
    }
    else {
        _logger->debug("Fail to open: " + timestampPath.toStdString());
    }

    // Write velodyne to xxxxxxxxx.txt file
    QString data_name = QString("%1.txt").arg(index, 10, 10, QLatin1Char('0'));
    QFile dataFile(dataPath + "/" + data_name);

    //identify the path of file and file format
    if (dataFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&dataFile);

        // Write pointcloud value
        for (unsigned long i = 0; i < pointCloud.points.size(); i++) {
            geometry_msgs::Point32 point = pointCloud.points[i];

            // write x,y,z to file
            QString numx = QString::number(double(point.x), 'f', 3);
            QString numy = QString::number(double(point.y), 'f', 3);
            QString numz = QString::number(double(point.z), 'f', 3);
            out << numx << " " << numy << " " << numz;

            // Get r if it is available
            QString numr = "calculate me, please!";
            if (numr == "available") {
                out << " " << numr;
            }
            out << "\n";
        }
        dataFile.close();
    }
    else {
        _logger->debug("Fail to open: " + dataPath.toStdString() + "/" + data_name.toStdString());
    }
}

void Node1Worker::recvGpsInspvaCallback(const novatel_gps_msgs::InspvaConstPtr &msg)
{
    if (startGps) {
        const novatel_gps_msgs::Inspva msg_clone = novatel_gps_msgs::Inspva(*msg);
        qDebug() << "GPS row index =" << gpsRowCount++;
        _writeGpsData(msg_clone);
    }
}

void Node1Worker::recvGpsInsstdevCallback(const novatel_gps_msgs::InsstdevConstPtr &msg)
{
    if (startGps) {
        // Write Insstdev value to temporary buffer
        QString tmpStr;
        QTextStream out(&tmpStr);
        out << msg->latitude_dev << " " << msg->longitude_dev << " " << msg->height_dev << " "
            << msg->roll_dev << " " << msg->pitch_dev << " " << msg->azimuth_dev;
        gpsInsstdevValue = tmpStr;
    }
}

void Node1Worker::_writeGpsData( const novatel_gps_msgs::Inspva msg)
{
    _mutexGps.lock();
    //get timestamp and add to timestamp buffer
    uint64_t nsec = msg.header.stamp.toNSec();
    QDateTime dt = QDateTime::fromMSecsSinceEpoch(nsec / 1000000);
    //qDebug() << "gpsTimestampBuffer =" << dt;
    gpsTimestampBuffer.append(dt.toString("yyyy-MM-dd HH:mm:ss.zzz"));

    //get data and add to data buffer
    QString dataStr;
    QTextStream out(&dataStr);
    out << msg.latitude << " " << msg.longitude << " " << msg.height << " ";
    out << msg.roll << " " << msg.pitch << " " << msg.azimuth << " ";
    out << msg.east_velocity << " " << msg.north_velocity << " " << msg.up_velocity << " ";
    out << gpsInsstdevValue;
    //qDebug() << "gpsDataBuffer =" << dataStr;
    gpsDataBuffer.append(dataStr);
    _mutexGps.unlock();
}

void Node1Worker::getGpsBuffer(QStringList &tsBuf, QStringList &dataBuf)
{
    _mutexGps.lock();
    // get timestamp buffer
    tsBuf.clear();
    tsBuf.append(gpsTimestampBuffer);
    gpsTimestampBuffer.clear();

    // get data buffer
    dataBuf.clear();
    dataBuf.append(gpsDataBuffer);
    gpsDataBuffer.clear();
    _mutexGps.unlock();
}

void Node1Worker::run()
{
    try {
        // ROS 1 node
        ros::init(_argc, _argv, AppConfig::instance().getNodeName().toStdString());

        ros::NodeHandle ros1_node;

        _velodyneLidar1Subcriber
            = ros1_node.subscribe("/ns1/velodyne_points",
                                10,
                                &Node1Worker::recvPointCloud2VelodyneLidar1Callback,
                                this);

        _velodyneLidar2Subcriber
            = ros1_node.subscribe("/ns2/velodyne_points",
                                10,
                                &Node1Worker::recvPointCloud2VelodyneLidar2Callback,
                                this);

        _ousterLidarSubcriber
            = ros1_node.subscribe("/points_raw",
                                10,
                                &Node1Worker::recvPointCloud2OusterCallback,
                                this);

        _gpsInspvaSubcriber
            = ros1_node.subscribe("/inspva",
                                10,
                                &Node1Worker::recvGpsInspvaCallback,
                                this);

        _gpsInsstdevSubcriber
            = ros1_node.subscribe("/insstdev",
                                10,
                                &Node1Worker::recvGpsInsstdevCallback,
                                this);

        ros::spin();
        ros::shutdown();

    } catch (std::exception &e) {
        if (_logger)
            _logger->debug("Got exception when starting a Ros Node: {}", e.what());
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when starting a Ros Node");
    }
}

void Node1Worker::shutDown()
{
    ros::shutdown();
}
#endif
