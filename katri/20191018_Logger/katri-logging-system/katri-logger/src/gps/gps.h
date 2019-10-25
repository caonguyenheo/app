#ifndef GPS_H
#define GPS_H

#include "../config.h"
#if ENABLE_ROS_VERSION == 1
#include "QDir"
#include "gpsParameter.h"
#include "../loggerManager.h"
#include "../node1Worker.h"
#include "../source.h"
#include <QTimer>

class Gps : public Source
{
    Q_OBJECT
public:
    explicit Gps(const GpsParameter &, QObject *parent = nullptr);
    ~Gps() override;
    void start() override;
    void stop() override;

    bool isRunning() override;
    QString getId() override;
    QString getName() override;
    QString getStatus() override;
    void setPath(const QString &) override;

private:
    void _createOutputDirectory();
    void _updateStatus(bool status);
    void _initRuntimeFile();
    void _writeDataFormatGPS();
    void slotWriteBufferToFile();

private:
    GpsParameter _parameters;
    std::shared_ptr<spdlog::logger> _logger = nullptr;
    QTimer *m_timer;
};
#endif
#endif // GPS_H
