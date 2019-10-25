#include "velodyneLaunch.h"

#if ENABLE_ROS_VERSION == 1

VelodyneLaunch::VelodyneLaunch(QObject *parent)
    : QThread(parent)
{
    _logger = LoggerManager::getAppLogger();
}
VelodyneLaunch::~VelodyneLaunch()
{
    if (_process)
        delete _process;
}
void VelodyneLaunch::run()
{
    QStringList arguments;
    arguments << "-c";
    arguments << "./velodyne.launch.sh";

    _process = new QProcess();
    _process->setWorkingDirectory(QDir::currentPath());
    _process->setArguments(arguments);
    _process->setProgram("/bin/bash");

    //connect(_process, SIGNAL(readyReadStandardOutput()), this, SLOT(captureDataOutput()));

    connect(_process,
            QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
            [=](QProcess::ProcessError error) {
                QString msg;
                if (error == QProcess::ProcessError::FailedToStart) {
                    msg.append("Failed to start");
                } else if (error == QProcess::ProcessError::Crashed) {
                    msg.append("Process crashed");
                } else if (error == QProcess::ProcessError::Timedout) {
                    msg.append("Process timeout");
                } else if (error == QProcess::ProcessError::ReadError) {
                    msg.append("Read error");
                } else if (error == QProcess::ProcessError::WriteError) {
                    msg.append("Write error");
                } else if (error == QProcess::ProcessError::UnknownError) {
                    msg.append("Unknown error");
                }
                if (_logger)
                    _logger->debug("Velodyne process occurred error: {}", msg.toStdString());
            });

    connect(_process,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [=](int exitCode, QProcess::ExitStatus exitStatus) {
                QString msg;
                if (exitStatus == QProcess::ExitStatus::NormalExit) {
                    msg.append("Normal exit");
                } else if (exitStatus == QProcess::ExitStatus::CrashExit) {
                    msg.append("Crash exit");
                }

                if (_logger)
                    _logger->debug(
                        "Velodyne process finished with exit code: {} and exist status: {}.",
                        exitCode,
                        msg.toStdString());
            });

    _process->start();

    // Keep running in event loop
    exec();
}

//void VelodyneLaunch::captureDataOutput()
//{
//    QString output(_process->readAll());
//}

#endif
