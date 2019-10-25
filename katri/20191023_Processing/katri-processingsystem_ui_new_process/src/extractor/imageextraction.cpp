#include "imageextraction.h"
#include <QGlib/Connect>
#include <QGlib/Error>
#include <QGst/Bus>
#include <QGst/Init>
#include <QGst/Parse>
#include <QGst/Buffer>
#include <QGst/ClockTime>
#include <QGst/ElementFactory>
#include <QGst/Event>
#include <QGst/Query>
#include <QGst/Sample>
#include <QDebug>
#include <QFile>
#include "navigationpanel.h"

// HD camera
const QString cam0 = "image_0";
const QString cam1 = "image_1";
const QString cam2 = "image_2";
const QString cam3 = "image_3";

ImageExtraction::ImageExtraction(QObject *parent)
    : QObject (parent)
{
    QGst::init();
    connect(this, SIGNAL(signalParseFirstVideo()), SLOT(slotParseFirstVideo()));
}

ImageExtraction::~ImageExtraction()
{
    // Stop pipeline
    if (m_pipeline) {
        m_pipeline->setState(QGst::StateNull);
        m_pipeline.clear();
    }
}

void ImageExtraction::setPath(QStringList pathList)
{
    _rawPathList.append(pathList);
}

void ImageExtraction::setPath(QString path)
{
    _rawPathList.append(path);
}

void ImageExtraction::slotParseFirstVideo()
{
    qDebug() << "Before get first:" << _rawPathList;
    if (_rawPathList.count() > 0) {
        QString path(_rawPathList.front());
        _rawPathList.pop_front();
        imageExtractor(path);
    }
    else {
        qDebug() << "End of list";
        NavigationPanel *parent = dynamic_cast<NavigationPanel*>(this->parent());
        if (parent) {
            emit parent->signalEnableAllButton(true);
        }
    }
    qDebug() << "After get first:" << _rawPathList;
}

bool ImageExtraction::imageExtractor(QString path)
{
    currentRawFile = path;

    // Verify camera type
    bool isHDCam = false;
    if ( path.contains(cam0) || path.contains(cam1)
      || path.contains(cam2) || path.contains(cam3) )
    {
        isHDCam = true;
    }

    // Stop pipeline
    if (m_pipeline) {
        m_pipeline->setState(QGst::StateNull);
        m_pipeline.clear();
    }

    int width  = 1920;
    int height = 1080;
    if (isHDCam) {
        width = 1024;
        height = 769;
    }

    if (!QFile(path).exists()) {
        qDebug() << "Raw file is not existed:" << path;
        emit this->signalParseFirstVideo();
        return false;
    }

    // Get raw folder path
    int lastSplash = path.lastIndexOf(".raw");
    QString rawFolderPath = path;
    rawFolderPath.truncate(lastSplash);

    // Create pipeline
    /*QString pipe1Descr =
            QString("filesrc location=%1 ! ").arg(path) +
            QString("videoparse width=%1 height=%2 framerate=30/1 format=5 ! ").arg(width, height) +
            QString("videoconvert ! pngenc ! multifilesink location=%1_").arg(rawFolderPath) +
            QString("%0.10d.png");

    qDebug() << pipe1Descr;*/
    m_pipeline = QGst::Pipeline::create();
    QGst::ElementPtr m_source  = QGst::ElementFactory::make("filesrc");
    QGst::ElementPtr m_parse   = QGst::ElementFactory::make("videoparse");
    QGst::ElementPtr m_convert = QGst::ElementFactory::make("videoconvert");
    QGst::ElementPtr m_scale   = QGst::ElementFactory::make("videoscale");
    QGst::ElementPtr capsfilter= QGst::ElementFactory::make("capsfilter", "capsfilter");
    QGst::ElementPtr m_jpegenc = QGst::ElementFactory::make("jpegenc");
    QGst::ElementPtr m_sink    = QGst::ElementFactory::make("multifilesink");

    m_source->setProperty("location", path);
    m_parse->setProperty("width", width);
    m_parse->setProperty("height", height);
    m_parse->setProperty("framerate", "60/1");
    m_parse->setProperty("format", 5);
    capsfilter->setProperty("caps", QGst::Caps::fromString("video/x-raw,width=640,height=360"));

    m_jpegenc->setProperty("quality", 80);
    QString jpgFile = QString("%1_").arg(rawFolderPath) + QString("%0.10d.jpg");
    m_sink->setProperty("location", jpgFile);

    m_pipeline->add(m_source, m_parse, m_convert, m_scale, capsfilter, m_jpegenc, m_sink);
    m_pipeline->linkMany(m_source, m_parse, m_convert, m_scale, capsfilter, m_jpegenc, m_sink);

    if (m_pipeline) {
        qDebug() << "Create pipeline successfully:" << path;
        QGst::BusPtr bus = m_pipeline->bus();
        bus->addSignalWatch();
        QGlib::connect(bus, "message", this, &ImageExtraction::onBusMessage);
        bus.clear();
        m_pipeline->setState(QGst::StatePlaying);
        return true;
    }
    else {
        qDebug() << "Can not create pipeline:" << path;

        // Remove RAW file after extracted
        if (QFile(currentRawFile).exists()) {
            QFile(currentRawFile).remove();
        }
        currentRawFile = "";

        emit this->signalParseFirstVideo();
        return false;
    }
}

void ImageExtraction::onBusMessage(const QGst::MessagePtr & message)
{
    switch (message->type()) {
    case QGst::MessageEos:
        m_pipeline->setState(QGst::StateNull);
        m_pipeline.clear();
        qDebug() << "Extract RAW video done";

        // Remove RAW file after extracted
        if (QFile(currentRawFile).exists()) {
            QFile(currentRawFile).remove();
        }
        currentRawFile = "";

        // Process next file
        emit this->signalParseFirstVideo();
        break;
    case QGst::MessageError:
        qCritical() << message.staticCast<QGst::ErrorMessage>()->error();
        break;
    default:
        break;
    }
}
