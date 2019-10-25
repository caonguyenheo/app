#include "imageCaptureSink.h"

ImageCaptureSink::ImageCaptureSink()
    : QGst::Utils::ApplicationSink()
{

}

QGst::FlowReturn ImageCaptureSink::newSample()
{
    return QGst::FlowOk;
}
