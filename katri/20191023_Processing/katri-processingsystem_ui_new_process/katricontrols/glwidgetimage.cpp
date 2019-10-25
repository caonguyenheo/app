#include "glwidgetimage.h"
#include <QDebug>
#include <QFile>

GLWidgetImage::GLWidgetImage(QWidget *parent) :
    QOpenGLWidget(parent)
{
}

GLWidgetImage::~GLWidgetImage()
{
}

void GLWidgetImage::paintEvent(QPaintEvent* e)
{
    QOpenGLWidget::paintEvent(e);
    QPainter painter(this);
    painter.drawImage(this->rect(), m_image);
    emit signalFinishedRender();
}


void GLWidgetImage::display(const QImage& img)
{
   m_image = img;
   this->update();
}

void GLWidgetImage::slotReceiveTimestamp(int key, QString value)
{
    Q_UNUSED(key);
    display(QImage(value));
//    qDebug() << "Render image [" << key << "]:" << value;
}
