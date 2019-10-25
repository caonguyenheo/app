#ifndef IMAGECAPTURESINK_H
#define IMAGECAPTURESINK_H

#include "QGst/Utils/ApplicationSink"
#include <QGst/Utils/ApplicationSource>

class ImageCaptureSink : public QGst::Utils::ApplicationSink
{
public:
    ImageCaptureSink();
    virtual QGst::FlowReturn newSample();
};

#endif // IMAGECAPTURESINK_H
