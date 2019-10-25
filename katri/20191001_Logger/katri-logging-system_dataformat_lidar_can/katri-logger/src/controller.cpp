#include "controller.h"

Controller::Controller(int argc, char **argv, QObject *parent) : QObject(parent)
{
    _argc = argc;
    _argv = argv;
    _node = new NodeWorker(_argc, _argv, this);
#if ENABLE_ROS_VERSION == 1
    _node1 = new Node1Worker(_argc, _argv, this);
    _velodyneLaunch = new VelodyneLaunch(this);
    _ousterLaunch = new OusterLaunch(this);
    _gpsLaunch = new GpsLaunch(this);
#endif
    // register meta type for Command before esbtablish connection
    qRegisterMetaType<Command>();

    connect(_node, &NodeWorker::messageReceived, this, &Controller::doIt);

    _logger = LoggerManager::getAppLogger();
}

Controller::~Controller()
{
    delete  _node;
    delete _argv;
    QFile file("ping.txt");
    if (file.exists()) {
        file.remove();
    }

#if ENABLE_ROS_VERSION == 1
    delete _velodyneLaunch;
    delete _ousterLaunch;
    delete _gpsLaunch;
    delete _node1;
#endif
}

void Controller::startWork()
{
    // start subscriber node for receiving commands
    _node->start();
    QFile file("ping.txt");
    if (file.exists()) {
        file.remove();
    }
    file.open(QIODevice::WriteOnly);
    file.close();

#if ENABLE_ROS_VERSION == 1
    // Start velodyne thread
    _velodyneLaunch->start();

    // Wait for 2 seconds for being sure roscore up by velodyne launch
    QThread::sleep(2);

    // Start ouster thread
    _ousterLaunch->start();

    // Start gnss thread
    _gpsLaunch->start();

    _node1->start();
#endif
    // Pass a node to data source manager
    DataSourceManager::instance().setNode(_node);

    // look up and create data sources
    DataSourceManager::instance().lookup().createSources();
}

void Controller::doIt(const Command command)
{
    switch (command.getAction()) {
    case Command::Action::START_NEW_KITTI:
    case Command::Action::START:
        // Initialize the data sources and start recording
        DataSourceManager::instance().start(command);
        break;
    case Command::Action::STOP:
        DataSourceManager::instance().stop(command);
        break;
    case Command::Action::COPY:
        DataSourceManager::instance().copy(command);
        break;
    default:
        break;
    }
}
