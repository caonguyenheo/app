#include "playbackcontrol.h"
#include "ui_playbackcontrol.h"
#include <QTime>
#include <QDebug>
#include <QMouseEvent>

PlaybackControl::PlaybackControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlaybackControl)
{
    ui->setupUi(this);

    ui->seekbar->setStyleSheet( "QSlider::groove:horizontal {border: 2px solid blue; height: 1px;background: green;margin: 0px 1px;}"
                                "QSlider::handle:horizontal {background:transparent;border-left: 7px solid red;width: 0px;height: 100px;margin:-5px -3px;border-radius:0px;}"
                                "QSlider::add-page:qlineargradient {background: white;}"
                                "QSlider::sub-page:qlineargradient {background: red;}"
                                );

    ui->playBtn->setStyleSheet( "QPushButton{"
                                "background-image: url(:/img/icon/Play-icon.png);"
                                "border-radius: 1px;"
                                "}"

                                "QPushButton:checked{"
                                "background-image: url(:/img/icon/Pause-icon.png);"
                                "border-radius:10px;"
                                "font: 14px 'Roboto';"
                                "}"
                                );

    ui->fullscreenBtn->setStyleSheet("QPushButton{"
                               "background-image: url(:/img/icon/Full screen-icon.png);"
                               "border-radius: 1px;"
                               "}"
                                );

    m_seekCurrent = new QLabel("00:00:00", this);
    m_seekCurrent->setStyleSheet( "color: #b9b9b9;"
                                  "background:transparent;"
                                  "font: 14px 'Roboto';"
                                  "font-weight: 500;"
                                );

    m_seekTotal = new QLabel(" / 00:00:00", this);
    m_seekTotal->setStyleSheet( "color: white;"
                               "background:transparent;"
                               "font: 14px 'Roboto';"
                               "font-weight: 500;"
                             );

    ui->seekbar->setMinimum(0);
    ui->seekbar->setTickInterval(1);
//    ui->seekbar->setTracking(false);
//    ui->seekbar->installEventFilter(this);
//    ui->playBtn->setCheckable(true);
    connect(ui->seekbar, SIGNAL(sliderReleased()), this, SLOT(slotSliderRelease()));
    connect(ui->seekbar, SIGNAL(sliderPressed()), this, SLOT(slotSliderRelease()));

    stateClickFullOrScale = 0;
    connect(ui->fullscreenBtn, SIGNAL(released()), this, SLOT(slotClickFullOrScale()));
}

PlaybackControl::~PlaybackControl()
{
    delete ui;
}

void PlaybackControl::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if (m_seekCurrent != nullptr && m_seekTotal != nullptr) {
        QPoint point, point1;
        // Set current text position
        point = QPoint(this->rect().width() - 161, this->rect().height() - 20);
        m_seekCurrent->setGeometry(point.x(), point.y(), 57, m_seekCurrent->geometry().height());
        // Set total text position
        point1 = QPoint(point.x() + m_seekCurrent->geometry().width(), point.y());
        m_seekTotal->setGeometry(point1.x(), point1.y(), m_seekTotal->geometry().width(), m_seekTotal->geometry().height());
    }
}

//bool PlaybackControl::eventFilter(QObject *obj, QEvent *event)
//{
//    QSlider* slider = qobject_cast<QSlider*>(obj);
//    if (slider && event->type() == QEvent::MouseButtonRelease)
//    {
//        if (m_length != 0) {
//            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
//            int x = mouseEvent->pos().x();
//            int value = (slider->maximum() - slider->minimum()) * x / slider->width() + slider->minimum();

//            if (slider->value() != value) {
//                qDebug()<< "SLIDER MouseButtonRelease " << value;
//                slider->setValue(value);
//                //emit ui->seekbar->sliderMoved(value);
//                emit signalMouseReleaseSlider(double(x) / slider->width());
//                return false;
//            }
//        }
//        return true;
//    }
//    else {
//        return parent()->eventFilter(obj, event);
//    }
//}

void PlaybackControl::controlConnection(const QObject *receiver, const char *member)
{
    qDebug() << "controlConnection";
    connect(ui->playBtn, SIGNAL(clicked(bool)), receiver, member);
}

void PlaybackControl::seekingBarConnection(const QObject *receiver, const char *member)
{
    qDebug() << "seekingBarConnection";
    connect(this, SIGNAL(signalMouseReleaseSlider(int)), receiver, member);
}

void PlaybackControl::slotReceiveEOP(bool status)
{
    ui->playBtn->blockSignals(true);
    ui->playBtn->setChecked(!status);
    ui->playBtn->blockSignals(false);
}

void PlaybackControl::slotSyncSeekbarRange(int interval, int index)
{
    ui->seekbar->setMinimum(0);
    ui->seekbar->setMaximum(index);
    ui->seekbar->setValue(0);

    // Current time
    QTime t(0,0);
    m_seekCurrent->setText(t.toString("HH:mm:ss"));

    // Total time
    t = t.addMSecs(interval);
    m_seekTotal->setText(" / " + t.toString("HH:mm:ss"));
}

void PlaybackControl::slotReceivePosition(int interval, int index)
{
    ui->seekbar->setValue(index);

    QTime t(0,0);
    t = t.addMSecs(interval);
    m_seekCurrent->setText(t.toString("HH:mm:ss"));
}

void PlaybackControl::slotClickFullOrScale()
{
    qDebug()<<"[Debug] slot FULL SCREEN";
    emit signalFullOrScaleScreen(stateClickFullOrScale);
}

void PlaybackControl::slotgetIconFullOrScaleScreen(int state)
{
    qDebug()<<"[Debug] slot get Icon SCREEN state = "<<state;
    stateClickFullOrScale = state;
    if(state == 0) {
        ui->fullscreenBtn->setStyleSheet("QPushButton{background-image: url(:/img/icon/Full screen-icon.png);border-radius: 1px;}");
    }
    else {
        ui->fullscreenBtn->setStyleSheet("QPushButton{background-image: url(:/img/icon/Exit Fullscreen.png);border-radius: 1px;}");
    }
}

void PlaybackControl::slotSliderRelease()
{
    int value = ui->seekbar->value();
    emit signalMouseReleaseSlider(value);
}
