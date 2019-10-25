#include "can.h"

Can::Can(const CanParameter &parameters, QObject *parent) : Source(parent)
{
    _parameters = parameters;
    _logger = LoggerManager::getCanLogger(parameters.getId());
}

Can::~Can()
{
    if (_canProcess != nullptr){
        delete _canProcess;
    }
}

QStringList Can::_buildArguments()
{
    // vcan0 -tA -p "/media/nvme/2019_09_25.kat/2019_09_25_drive_00000/ccan"
    QStringList arguments;
//    QStringList TxtFilters = {"*.txt"};
//    QString loadToPath = _parameters.getCanDirectoryPath() + "/data";
//    QStringList kittiFileList = QDir(loadToPath).entryList(TxtFilters, QDir::Files);
    arguments << _parameters.getSrc();
    arguments << "-tA";
//    arguments << QString("-I") + QString::number(kittiFileList.count());
//    qDebug()<<"kittiFileList:"<<arguments;
    arguments << QString("-p");
    arguments << _parameters.getCanDirectoryPath();
//    qDebug()<<"getCanDirectoryPath:"<<arguments;
    return arguments;
}

QString Can::_buildNewFileName(const QString &name)
{
    return name.rightJustified(8, '0', true).append(".txt");
}

void Can::_createOutputDirectory()
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

        // Calculate file name
        _parameters.setFileName(_buildNewFileName("0"));
    } else {
        // Calculate file name
        QStringList f("*.txt");
        dir.setNameFilters(f);
        dir.setFilter(QDir::Files);
        dir.setSorting(QDir::SortFlag::Time);
        QStringList files = dir.entryList();
        if (files.empty()) {
            _parameters.setFileName(_buildNewFileName("0"));
        } else {
            _parameters.setFileName(_buildNewFileName(
                QString::number(files.at(0).left(files.at(0).lastIndexOf(".")).toLong() + 1)));
        }
    }
}

void Can::_triggerProcess()
{
    // Start from scraft. Don't move it anywhere
    _canProcess = new QProcess(this);
    _canProcess->setProgram(_parameters.getProgram());
    _canProcess->setArguments(_buildArguments());
    connect(_canProcess,
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
    connect(_canProcess,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [=](int exitCode, QProcess::ExitStatus exitStatus) {
                disconnect(_canProcess,
                           QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
                           nullptr,
                           nullptr);

                disconnect(_canProcess,
                           QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                           nullptr,
                           nullptr);

                // It is safe to kill the process here
                _canProcess->kill();

                delete _canProcess;
                _canProcess = nullptr;

                QString msg;
                if (exitStatus == QProcess::ExitStatus::NormalExit) {
                    msg.append("Normal exit");
                } else if (exitStatus == QProcess::ExitStatus::CrashExit) {
                    msg.append("Crash exit");
                }
                if (_logger)
                    _logger
                        ->debug("CAN process finished with exit code: {} and exist status: {}.",
                                exitCode,
                                msg.toStdString());

                // Mark a process is stopped
                _iAmStopped();
            });

    _canProcess->start();
}

void Can::start()
{
    _createOutputDirectory();

    _triggerProcess();

    // Everything is fine, set true to indicate the process is running
    _iAmRunning();
}

void Can::_iAmRunning()
{
    _running = true;
}

void Can::_iAmStopped()
{
    _running = false;
}

void Can::setPath(const QString &path)
{
    _parameters.setPath(path);
}

QString Can::getId()
{
    return _parameters.getId();
}

bool Can::isRunning()
{
    return _running;
}

QString Can::getName()
{
    return "CAN";
}

QString Can::getStatus()
{
    return (_running) ? "Running" : "Stopped";
}

void Can::stop()
{
    if (_canProcess){
        if (_canProcess->state() != QProcess::NotRunning)
            _canProcess->terminate();
    }

    // Everything is fine, set false to indicate the process is stopped
    _iAmStopped();
}

//void Can::captureDataOutput(){
//    QString output(_canProcess->readAll());
//    QRegularExpression rx("^\\s{1}\\((?<ts>\\d{4}-\\d{2}-\\d{2}\\s{1}\\d{2}:\\d{2}:\\d{2}.\\d{6})\\)\\s{2}(?<src>\\w+)\\s{2}(?<id>\\d+)\\s{3}\\[(?<length>\\d+)\\]\\s{2}(?<data>.*)$");
//    QRegularExpressionMatch m = rx.match(output);
//    if (m.hasMatch()) {
//        // Dir value uses to calculate storage path. So it must be set to the value getting
//        // from application configuration with a pattern "can_{device_id}_{can_id_in_data}"
//        _parameters.setDir(
//            AppConfig::instance().getCanMappingValue(_parameters.getId() + "_" + m.captured("id")));

//        _createOutputDirectory();

//        QFile timestamp(_parameters.getTimestampPath());
//        if (timestamp.open(QIODevice::Append | QIODevice::Text))
//        {
//            QTextStream out(&timestamp);
//            out << m.captured("ts") << "\n";
//        }

//        QFile data(_parameters.getFilePath());
//        if (data.open(QIODevice::Append | QIODevice::Text))
//        {
//            QTextStream out(&data);
//            out << m.captured("data") << "\n";
//        }
//    }
//}

//void Can::captureErrorOutput(){
//    QString output(_canProcess->readAllStandardOutput());
//    if(_logger)
//        _logger->error("Error occurred: {}", output.toStdString());
//}
