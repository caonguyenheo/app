#ifndef IMAGEEXTRACTION_H
#define IMAGEEXTRACTION_H
#include <QString>
#include <QGst/Message>
#include <QGst/Pipeline>

class ImageExtraction : public QObject
{
    Q_OBJECT

public:
    ImageExtraction(QObject *parent = nullptr);
    ~ImageExtraction();
    void setPath(QStringList pathList);
    void setPath(QString path);
    bool imageExtractor(QString path);
private:
    void onBusMessage(const QGst::MessagePtr & message);

Q_SIGNALS:
    void signalExtractframe();

public Q_SLOTS:
    void slotParseFirstVideo();
private:
    QGst::PipelinePtr m_pipeline;
    QStringList _rawPathList;
};

#endif // IMAGEEXTRACTION_H
