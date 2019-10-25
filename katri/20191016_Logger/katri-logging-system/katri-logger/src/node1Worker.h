#ifndef NODE1WORKER_H
#define NODE1WORKER_H
#include "config.h"

#if ENABLE_ROS_VERSION == 1
#include "QHash"
#include "QMutex"
#include "QMutexLocker"
#include "QThread"
#include "appConfig.h"
#include "geometry_msgs/Point32.h"
#include "loggerManager.h"
#include "ros/node_handle.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "novatel_gps_msgs/Inspva.h"
#include "novatel_gps_msgs/Insstdev.h"

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;

class Node1Worker : public QThread
{
    Q_OBJECT
public:
    Node1Worker(int, char **, QObject *parent = nullptr);
    ~Node1Worker() override;
    void run() override;
    static void shutDown();
    static bool startVelodyneLidar1;
    static bool startVelodyneLidar2;
    static bool startOusterLidar;
    static QString velodyneLidar1Runtime;
    static QString velodyneLidar2Runtime;
    static QString ousterLidarRuntime;
    static QString velodyneLidar1DataPath;
    static QString velodyneLidar2DataPath;
    static QString ousterLidarDataPath;
    static QString velodyneLidar1TimestampPath;
    static QString velodyneLidar2TimestampPath;
    static QString ousterLidarTimestampPath;

    static bool startGps;
    static QString gpsRuntime;
    static QString gpsDataPath;
    static QString gpsTimestampPath;
    static QString gpsInsstdevValue;
    static QStringList gpsTimestampBuffer;
    static QStringList gpsDataBuffer;
    static QMutex _mutexGps;

    static int gpsRowCount;

private:
    int _argc;
    char **_argv = nullptr;
    ros::Subscriber _velodyneLidar1Subcriber;
    ros::Subscriber _velodyneLidar2Subcriber;
    ros::Subscriber _ousterLidarSubcriber;

    ros::Subscriber _gpsInspvaSubcriber;
    ros::Subscriber _gpsInsstdevSubcriber;

    std::shared_ptr<spdlog::logger> _logger = nullptr;

    mutable QMutex _mutexVelodyne1;
    mutable QMutex _mutexVelodyne2;
    mutable QMutex _mutexOuster;

private:
    void recvPointCloud2VelodyneLidar2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void recvPointCloud2VelodyneLidar1Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void recvPointCloud2OusterCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void _writeLidarData( const sensor_msgs::PointCloud2 msg,
                          QString runtimeFile,
                          QString timestampPath,
                          QString dataPath );

    void recvGpsInspvaCallback(const novatel_gps_msgs::InspvaConstPtr &msg);
    void recvGpsInsstdevCallback(const novatel_gps_msgs::InsstdevConstPtr &msg);
    void _writeGpsData( const novatel_gps_msgs::Inspva msg);

public:
    static void getGpsBuffer(QStringList &tsBuf, QStringList &dataBuf);

};
#endif
#endif // NODE1WORKER_H
