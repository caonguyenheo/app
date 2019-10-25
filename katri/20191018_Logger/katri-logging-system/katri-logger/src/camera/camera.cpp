#include "camera.h"

Camera::Camera(const CameraParameter &parameters, QObject *parent)
    : Source(parent)
{
    _parameters = parameters;
    _logger = LoggerManager::getCameraLogger(parameters.getId());
}

Camera::~Camera()
{
    if (_cameraProcess != nullptr) {
        delete _cameraProcess;
    }
}

void Camera::setPath(const QString &path)
{
    _parameters.setPath(path);
}

QString Camera::getId()
{
    return _parameters.getId();
}

bool Camera::isRunning()
{
    return _running;
}

QString Camera::getName()
{
    return "Camera";
}

QString Camera::getStatus()
{
    return (_running) ? "Running" : "Stopped";
}

QStringList Camera::_buildArguments()
{
    QStringList arguments;
    arguments << "--capture";
    arguments << "--size=" + _parameters.getSize();
    arguments << "--format=" + _parameters.getFormat();
    arguments << "--file=" + _parameters.getFilePath();
    arguments << "--timestamp=" + _parameters.getTimestampPath();
    arguments << _parameters.getSrc();

    return arguments;
}

void Camera::_createOutputDirectoryStructure()
{
    // Create $DATAPATH$/YYYY_MM_DD.kat/YYYY_MM_DD_drive_xxxx/image_x/data path
    QString directory = _parameters.getDataSetPath();
    directory = directory.append("/data");
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

void Camera::_triggerTimer()
{
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, QOverload<>::of(&Camera::_captureImage));
    _timer->start(500);
}

void Camera::_captureImage()
{
    if (QFileInfo::exists(_parameters.getFilePath())) {
        if (!_capturing) {
            _capturing = true;
            auto worker = std::make_shared<CaptureImageWorker>(_parameters.getId(),
                                                               _parameters.getSize(),
                                                               _parameters.getFormat(),
                                                               _parameters.getFilePath());

            worker->capture(_node);
            _capturing = false;
        }
    }
}

void Camera::_triggerProcess()
{
    // Start from scraft. Don't move it anywhere
    _cameraProcess = new QProcess(this);
    _cameraProcess->setProgram(_parameters.getProgram());
    _cameraProcess->setArguments(_buildArguments());
    connect(_cameraProcess,
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
                    _logger->debug("Error occurred: {}", msg.toStdString());
            });
    connect(_cameraProcess,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [=](int exitCode, QProcess::ExitStatus exitStatus) {
		//Restart while camera is not connected
                if (exitCode == QProcess::ProcessError::Crashed) {
                    //qDebug()<<"Camera Disconnected";
                    //qDebug()<<"No recording";
                    if (_cameraProcess != nullptr) {
                        _cameraProcess->kill();
                        _cameraProcess->start();
                    }
                return;
                }

                disconnect(_cameraProcess,
                           QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
                           nullptr,
                           nullptr);
                disconnect(_cameraProcess,
                           QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                           nullptr,
                           nullptr);

                delete _cameraProcess;
                _cameraProcess = nullptr;

                //_timer->stop();

                delete _timer;
                _timer = nullptr;

                QString msg;
                if (exitStatus == QProcess::ExitStatus::NormalExit) {
                    msg.append("Normal exit");
                } else if (exitStatus == QProcess::ExitStatus::CrashExit) {
                    msg.append("Crash exit");
                }
                if (_logger)
                    _logger
                        ->debug("Camera process finished with exit code: {} and exist status: {}.",
                                exitCode,
                                msg.toStdString());

                // Mark a process is stopped
                _iAmStopped();

                emit stopped();
            });

    _cameraProcess->start();
}

void Camera::start()
{
    _createOutputDirectoryStructure();

    _triggerProcess();

    _triggerTimer();

    // Everything is fine, set true to indicate the process is running
    _iAmRunning();
}

void Camera::stop()
{
    if (_cameraProcess) {
        if (_cameraProcess->state() != QProcess::NotRunning) {
            // Notify the process to stop
            _cameraProcess->terminate();
        }
    }

    QEventLoop loop;
    connect(this, &Camera::stopped, &loop, &QEventLoop::quit);
    // Wait for signal stopped
    loop.exec();

    if (_logger)
        _logger->debug("{} with Id {} is stopped.", getName().toStdString(), getId().toStdString());
}

void Camera::_iAmRunning()
{
    _running = 1;
}

void Camera::_iAmStopped()
{
    _running = 0;
}

void Camera::setNode(NodeWorker *node)
{
    _node = node;
}
