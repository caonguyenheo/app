#ifndef CAMERA_H
#define CAMERA_H

#include "QAtomicInt"
#include "QDateTime"
#include "QDir"
#include "QEventLoop"
#include "QFileInfo"
#include "QMutex"
#include "QMutexLocker"
#include "QObject"
#include "QProcess"
#include "QRegularExpression"
#include "QRegularExpressionMatch"
#include "QTimer"
#include "QtGlobal"
#include "cameraParameter.h"
#include "captureImageWoker.h"
#include "../loggerManager.h"
#include "../nodeWorker.h"
#include "../source.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

class Camera: public Source
{
    Q_OBJECT
public:
    explicit Camera(const CameraParameter &, QObject *parent = nullptr);
    ~Camera() override;
    void start() override;
    void stop() override;

    bool isRunning() override;
    QString getId() override;
    QString getName() override;
    QString getStatus() override;
    void setPath(const QString &) override;
    void setNode(NodeWorker *);

private:
    CameraParameter _parameters;
    QProcess *_cameraProcess = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;
    // bool volatile _running = false;
    QAtomicInt _running = 0;
    bool volatile _capturing = false;
    QTimer *_timer = nullptr;
    NodeWorker *_node = nullptr;

private:
    void _triggerTimer();
    void _triggerProcess();
    QStringList _buildArguments();
    void _createOutputDirectoryStructure();
    void _iAmRunning();
    void _iAmStopped();
    void _captureImage();

signals:
    void stopped();
};
#endif // CAMERA_H
