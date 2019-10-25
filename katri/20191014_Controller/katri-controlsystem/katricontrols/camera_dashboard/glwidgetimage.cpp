#include "glwidgetimage.h"
#include <QDebug>
#include <QFile>

GLWidgetImage::GLWidgetImage(QWidget *parent) :
    QOpenGLWidget(parent)
{
    isRendering = false;
}

GLWidgetImage::~GLWidgetImage()
{
}

void GLWidgetImage::paintEvent(QPaintEvent* e)
{
    QOpenGLWidget::paintEvent(e);
    QPainter painter(this);
    painter.drawImage(this->rect(), m_image);
}


void GLWidgetImage::display(const QImage &img)
{
   m_image = img;
   this->update();
}

void GLWidgetImage::setDataCameraView(QByteArray &data)
{
    if (isRendering == false) {
        // Block rendering image thread
        isRendering = true;

        try {
            // Set frame data to video player comtrol
            QImage pixmap;
            pixmap.loadFromData(data, "JPG");
            pixmap.save("/home/kevin/file.jpg");
            display(pixmap);
        }
        catch (...) {
            qDebug()<<"Skip frame render: No.= " << this->objectName();
        }

        isRendering = false;
    }
    else {
        qDebug()<<"Skip frame render: No.= " << this->objectName();
    }
}
