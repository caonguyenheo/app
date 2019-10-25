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
    void display(const QImage& img);

Q_SIGNALS:
    void signalFinishedRender();

public Q_SLOTS:
    void slotReceiveTimestamp(int,QString);

private:
    QImage m_image;
};

#endif // SVMIMAGE_H
