
#include "captureImageWoker.h"

CaptureImageWorker::CaptureImageWorker(const QString &id,
                                       const QString &size,
                                       const QString &format,
                                       const QString &path)
{
    _id = id;
    _size = size;
    _format = format;
    _path = path;

    _logger = LoggerManager::getCameraLogger(id);
}

CaptureImageWorker::~CaptureImageWorker()
{
    if (m_pipeline) {
        m_pipeline->setState(QGst::StateNull);
        m_pipeline.clear();
    }
}

void CaptureImageWorker::capture(NodeWorker *node)
{
    try {
        QGst::init(0, 0);

        /* Source pipeline */
        QString pipe1Descr = QString("filesrc location=\"%1\" ! "
                                     "videoparse width=%2 height=%3 framerate=30/1 format=5 ! "
                                     "videoconvert ! pngenc ! "
                                     "appsink name=\"mysink\"")
                                 .arg(_path, _size.split("x").value(0), _size.split("x").value(1));

        m_pipeline = QGst::Parse::launch(pipe1Descr).dynamicCast<QGst::Pipeline>();
        m_sink.setElement(m_pipeline->getElementByName("mysink"));
        QGlib::connect(m_pipeline->bus(), "message::error", this, &CaptureImageWorker::onBusMessage);
        m_pipeline->bus()->addSignalWatch();

        // Start videoplayer
        m_pipeline->setState(QGst::StatePlaying);

        // Seeking to last frame
        quint64 lastFrame = quint64(QGst::ClockTime::fromTime(length())) - 33000000;
        QTime time = QGst::ClockTime(lastFrame).toTime();

        if (!time.isNull()) {
            setPosition(time);
            m_pipeline->setState(QGst::StatePaused);

            QGst::SamplePtr convertedSample = m_sink.pullPreroll();

            if (convertedSample) {
                QGst::BufferPtr buffer;
                QGst::MapInfo mapInfo;

                // Get Buffer frame
                buffer = convertedSample->buffer();
                buffer->map(mapInfo, QGst::MapRead);

                QImage snapShot;
                snapShot.loadFromData((const uchar *) mapInfo.data(), int(mapInfo.size()));

                QImage scaled = snapShot.scaled(384, 216);

                QByteArray arr;
                QBuffer buff(&arr);
                buff.open(QIODevice::WriteOnly);
                scaled.save(&buff, "jpg");

                node->sendImage(arr);

                m_pipeline->setState(QGst::StateNull);

                buffer->unmap(mapInfo);
                convertedSample.clear();
                buffer.clear();
            }
            else {
                if (_logger)
                    _logger->debug("convertedSample is null");
            }
        } else {
            if (_logger)
                _logger->debug("time is null");
            m_pipeline->setState(QGst::StateNull);
        }

    } catch (std::exception &e) {
        if (_logger)
            _logger->debug("Got exception when capturing image: {}", e.what());
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when capturing image");
    }
}

void CaptureImageWorker::setPosition(const QTime &pos)
{
    QGst::SeekEventPtr evt = QGst::SeekEvent::create(1.0,
                                                     QGst::FormatTime,
                                                     QGst::SeekFlagFlush,
                                                     QGst::SeekTypeSet,
                                                     qint64(QGst::ClockTime::fromTime(pos)),
                                                     QGst::SeekTypeNone,
                                                     qint64(QGst::ClockTime::None));

    m_pipeline->sendEvent(evt);
}

QTime CaptureImageWorker::length()
{
    if (m_pipeline) {
        //here we query the pipeline about the content's duration
        //and we request that the result is returned in time format
        QGst::DurationQueryPtr query = QGst::DurationQuery::create(QGst::FormatTime);
        m_pipeline->query(query);
        return QGst::ClockTime(quint64(query->duration())).toTime();
    } else {
        return QTime(0, 0);
    }
}

void CaptureImageWorker::onBusMessage(const QGst::MessagePtr &message)
{
    switch (message->type()) {
    case QGst::MessageEos:
        //quit();
        m_pipeline->setState(QGst::StateNull);
        break;
    case QGst::MessageError:
        qCritical() << message.staticCast<QGst::ErrorMessage>()->error();
        break;
    default:
        break;
    }
}
