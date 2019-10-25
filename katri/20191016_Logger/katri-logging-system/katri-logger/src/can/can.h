#ifndef CAN_H
#define CAN_H
#include "QDateTime"
#include "QDir"
#include "QObject"
#include "QProcess"
#include "QRegularExpression"
#include "QRegularExpressionMatch"
#include "QtGlobal"
#include "../appConfig.h"
#include "canParameter.h"
#include "../loggerManager.h"
#include "../source.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

class Can: public Source
{
    Q_OBJECT
public:
    explicit Can(const CanParameter &, QObject *parent = nullptr);
    ~Can() override;
    void start() override;
    void stop() override;
    bool isRunning() override;
    QString getId() override;
    QString getName() override;
    QString getStatus() override;
    void setPath(const QString &) override;

private:
    CanParameter _parameters;
    QProcess *_canProcess = nullptr;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

    bool volatile _running = false;

private:
    void _triggerProcess();
    QStringList _buildArguments();
    void _createOutputDirectory();
    void _iAmRunning();
    void _iAmStopped();
    QString _buildNewFileName(const QString &);

//private Q_SLOTS:
//    void captureDataOutput();
//    void captureErrorOutput();
};

#endif // CAN_H
