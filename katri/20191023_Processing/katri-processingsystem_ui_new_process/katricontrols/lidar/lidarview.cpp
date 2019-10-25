#include "lidarview.h"
#include <QGridLayout>
#include <QDebug>
#include <QFile>
#include "common.h"

const int UPPER_BOUNDARY = 10000;

LidarView::LidarView(QWidget *parent) :
    QWidget(parent)
{
    m_3dScatter = new Q3DScatter();
    m_scatterData = new ScatterDataModifier(m_3dScatter);
    QWidget *lidarWidget = QWidget::createWindowContainer(m_3dScatter , this);

    QGridLayout *layout = new QGridLayout();
    layout->setSpacing(0);
    layout->setMargin(2);
    layout->addWidget(lidarWidget);
    this->setLayout(layout);
}

LidarView::~LidarView()
{
    delete m_3dScatter;
    delete m_scatterData;
}

void LidarView::paintEvent(QPaintEvent* e)
{
    QWidget::paintEvent(e);
//    QPainter painter(this);
//    painter.drawImage(this->rect(), m_image);
}

void LidarView::slotReceiveTimestamp(int key, QString value)
{
    Q_UNUSED(key);
//    QTime now = QTime::currentTime();
    readDataFile(value);
    startRender();
    emit signalFinishedRender();
//    qDebug() << "Render frame [" << key << "]:" << value << now.msecsTo(QTime::currentTime());
}

//void popRandom(QStringList &input, QStringList &output)
//{
//    int count = input.count();
//    for (int i = 0; i < UPPER_BOUNDARY; i++) {
//        int mod = std::rand() % count;
//        QString get = input.at(mod);
//        if(!output.contains(get)) {
//            output.append(get);
//        }
//    }
//}

void LidarView::readDataFile(QString path)
{
    currentData.clear();
    QStringList total;
    readFileBuffer(path.toStdString().c_str(), total);

    int count = total.count();
    int div = int(ceil(double(count) / UPPER_BOUNDARY));
    for (int i = 0; i < count; i++) {
        if ((i%div) == 0) {
            QStringList split = total.at(i).split(" ");
            if (split.count() >= 3) {
                currentData.append(QVector3D(split[0].toFloat(),split[1].toFloat(),split[2].toFloat()));
            }
        }
    }

//    if (total.count() > UPPER_BOUNDARY) {
//        QStringList output;
//        popRandom(total, output);

//        Q_FOREACH(QString str, output)
//        {
//            QStringList split = str.split(" ");
//            if (split.count() >= 3) {
//                currentData.append(QVector3D(split[0].toFloat(),split[1].toFloat(),split[2].toFloat()));
//            }
//        }
//    }
//    else {
//    Q_FOREACH(QString str, total) {
//        QStringList split = str.split(" ");
//        if (split.count() >= 3) {
//            currentData.append(QVector3D(split[0].toFloat(),split[1].toFloat(),split[2].toFloat()));
//        }
//    }
}

void LidarView::startRender()
{
    if (currentData.count() > 0) {
        m_scatterData->SetData(&currentData, false);
    }
}
