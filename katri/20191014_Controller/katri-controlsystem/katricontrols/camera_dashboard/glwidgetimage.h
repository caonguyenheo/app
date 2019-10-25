#ifndef QLWIDGETIMAGE_H
#define QLWIDGETIMAGE_H
#include <QOpenGLWidget>
#include <QPainter>

class GLWidgetImage : public  QOpenGLWidget
{
    Q_OBJECT
public:
     GLWidgetImage(QWidget* parent = nullptr);
      ~GLWidgetImage() override;
    void paintEvent(QPaintEvent*) override;
    void display(const QImage &img);
    void setDataCameraView(QByteArray &data);

private:
    QImage m_image;
    bool isRendering;
};

#endif // SVMIMAGE_H
