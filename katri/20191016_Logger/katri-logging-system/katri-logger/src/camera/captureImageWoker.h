#ifndef CAPTUREIMAGEWOKER_H
#define CAPTUREIMAGEWOKER_H
#include <QGlib/Connect>
#include <QGlib/Error>
#include <QGst/Bus>
#include <QGst/Init>
#include <QGst/Message>
#include <QGst/Parse>
#include <QGst/Pipeline>
#include <QGst/Utils/ApplicationSink>
#include <QGst/Utils/ApplicationSource>

#include "QBuffer"
#include "QIODevice"
#include "QImage"
#include "QObject"
#include "imageCaptureSink.h"
#include "../nodeWorker.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include <QGst/Buffer>
#include <QGst/ClockTime>
#include <QGst/ElementFactory>
#include <QGst/Event>
#include <QGst/Query>
#include <QGst/Sample>

class CaptureImageWorker : public QObject
{
public:
    explicit CaptureImageWorker(const QString &id,
                                const QString &size,
                                const QString &format,
                                const QString &path);
    ~CaptureImageWorker();
    void capture(NodeWorker *);

private:
    QString _id;
    QString _size;
    QString _format;
    QString _path;
    QGst::Utils::ApplicationSource m_src;
    ImageCaptureSink m_sink;
    QGst::PipelinePtr m_pipeline;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

private:
    void setPosition(const QTime &pos);
    QTime length();
    void onBusMessage(const QGst::MessagePtr &message);
};
#endif // CAPTUREIMAGEWOKER_H
