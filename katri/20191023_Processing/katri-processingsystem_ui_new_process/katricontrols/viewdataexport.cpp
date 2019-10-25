#include <QTimer>

#include "viewdataexport.h"
#include "ui_viewdataexport.h"

#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QtQml/qqmlcontext.h>
#include <QQmlProperty>

#include "common.h"

ViewDataExport::ViewDataExport(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::viewdataexport)
{
    ui->setupUi(this);

    this->setStyleSheet("border:1px solid #585858;background-color:#191919; border-radius: 4px;");
    ui->label->setStyleSheet("min-height:27px; max-width:114px; margin-bottom:-3px; color:#acacac;font:14px 'Roboto'; border:1px solid #585858;border-top-left-radius:4px;border-top-right-radius: 4px;background-color:#191919; padding:0 18px; border-bottom-width:0px;");
    ui->label->setText(" Data export ");
    {
        camera = new exportdata_left(this);
        camera->setView_exportname("Camera");
        camera->setView_exportimage("/img/icon/Cam-Exp.jpg");

        ui->gridLayout_2->addWidget(camera,0,0,1,1);

        svm = new exportdata_left(this);
        svm->setView_exportname("SVM");
        svm->setView_exportimage("/img/icon/SVM-Exp.jpg");
        ui->gridLayout_2->addWidget(svm,0,1,1,1);

        gps = new exportdata_left(this);
        gps->setView_exportname("GPS");
        gps->setView_exportimage("/img/icon/GPS-Exp.jpg");
        ui->gridLayout_2->addWidget(gps,0,2,1,1);

//        // Set stretch layout
//        ui->gridLayout_2->setStretch(0, 1);
//        ui->gridLayout_2->setStretch(1, 1);
//        ui->gridLayout_2->setStretch(2, 1);
    }

    {
        radar = new exportdata_left(this);
        radar->setView_exportname("Radar");
        radar->setView_exportimage("/img/icon/Radar-Exp.jpg");
        ui->gridLayout_2->addWidget(radar,1,0,1,1);

        lidar = new exportdata_left(this);
        lidar->setView_exportname("Lidar");
        lidar->setView_exportimage("/img/icon/Lidar-Exp.jpg");
        ui->gridLayout_2->addWidget(lidar,1,1,1,1);

        canbus = new exportdata_left(this);
        canbus->setView_exportname("Can Bus");
        canbus->setView_exportimage("/img/icon/CanBus-Exp.jpg");
        ui->gridLayout_2->addWidget(canbus,1,2,1,1);

        // Set stretch layout
        ui->gridLayout_2->setColumnStretch(0, 1);
        ui->gridLayout_2->setColumnStretch(1, 1);
        ui->gridLayout_2->setColumnStretch(2, 1);

    }

    lblicon = new QLabel(this);
    lblmsg = new QLabel(this);

    lblicon->setStyleSheet("border : none ; border-image: url(:/icon/icon/notification/Success-icon.svg) 0 0 0 0 stretch stretch;");
    lblmsg->setText("Data are exported successfully");
    lblmsg->setStyleSheet("background-color:transparent;border:none;color : green;");

    QPoint point = ui->horizontalLayout_message->geometry().topLeft();
    lblicon->setGeometry(point.x()+30, point.y()+9, 14 , 14);
    lblicon->setMaximumSize(QSize(14,14));
    lblmsg->setGeometry(point.x()+50, point.y()+9, 200, 14);
    lblmsg->setMaximumSize(QSize(200,14));

    lblicon->hide();
    lblmsg->hide();

    QQuickView *rangeSliderView = new QQuickView();
    QWidget *rangeSliderWidget = QWidget::createWindowContainer(rangeSliderView, this);
    rangeSliderWidget->setFixedHeight(30);
    rangeSliderView->setResizeMode(QQuickView::SizeRootObjectToView);
    rangeSliderView->setSource(QUrl(QStringLiteral("qrc:/qml/qml/RangeSlider.qml")));
    _rangeSlider = rangeSliderView->rootObject();
    ui->gridLayout_2->addWidget(rangeSliderWidget,2,0,1,3);


    QHBoxLayout* hlayout1 = new QHBoxLayout();
    hlayout1->setObjectName(QString::fromUtf8("hlayout1"));
    QSpacerItem *hSpacer1 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    startLabel = new QLabel(this);
    startLabel->setFixedWidth(160);
    startLabel->setStyleSheet("background-color:transparent; border:none; color:white; min-height:27px; max-width:114px; font:14px 'Roboto';");
    hlayout1->addWidget(startLabel);
    hlayout1->addItem(hSpacer1);
    ui->gridLayout_2->addLayout(hlayout1,3,0,1,1);

    QHBoxLayout* hlayout2 = new QHBoxLayout();
    hlayout2->setObjectName(QString::fromUtf8("hlayout2"));
    QSpacerItem *hSpacer2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    endLabel = new QLabel(this);
    endLabel->setFixedWidth(160);
    endLabel->setStyleSheet("background-color:transparent; border:none; color:white; min-height:27px; max-width:114px; font:14px 'Roboto';");
    hlayout2->addItem(hSpacer2);
    hlayout2->addWidget(endLabel);
    ui->gridLayout_2->addLayout(hlayout2,3,2,1,1);

    connect(_rangeSlider, SIGNAL(signalStartChange(double)),
            this, SLOT(slotStartSliderValueChanged(double)));

    connect(_rangeSlider, SIGNAL(signalEndChange(double)),
            this, SLOT(slotEndSliderValueChanged(double)));

    connect(this, SIGNAL(signalMessageSuccess(int)), SLOT(slotMessageSuccess(int)));
}

ViewDataExport::~ViewDataExport()
{
    delete ui;
}

void ViewDataExport::selectConnection(const QObject *receiver, const char *member)
{
    this->camera->selectConnection(receiver,member);
    this->svm->selectConnection(receiver,member);
    this->gps->selectConnection(receiver,member);
    this->lidar->selectConnection(receiver,member);
    this->radar->selectConnection(receiver,member);
    this->canbus->selectConnection(receiver,member);
}

void ViewDataExport::slotMessageSuccess(int time)
{
    qDebug() << "Successfully message is displayed";
    lblicon->show();
    lblmsg->show();
    QTimer::singleShot(time, this, SLOT(hideMessageExportStatus()));
}

void ViewDataExport::hideMessageExportStatus()
{
    qDebug() << "Successfully message is hidden";
    lblicon->hide();
    lblmsg->hide();
}

void ViewDataExport::slotStartSliderValueChanged(double value)
{
    qint64 millisecondsDiff = minTS.msecsTo(maxTS);
    qint64 currentMSec = qint64((value / 1000) * millisecondsDiff);
    QDateTime current = minTS;
    current = current.addMSecs(currentMSec);

    startLabel->setText(current.toString(TIMESTAMP_FORMAT));
    qDebug() << value;
}

void ViewDataExport::slotEndSliderValueChanged(double value)
{
    qint64 millisecondsDiff = minTS.msecsTo(maxTS);
    qint64 currentMSec = qint64((value / 1000) * millisecondsDiff);
    QDateTime current = minTS;
    current = current.addMSecs(currentMSec);

    endLabel->setText(current.toString(TIMESTAMP_FORMAT));
    qDebug() << value;
}

QString ViewDataExport::getStartTimestamp()
{
    return startLabel->text();
}

QString ViewDataExport::getEndTimestamp()
{
    return endLabel->text();
}

void ViewDataExport::setMinTimestamp(QDateTime min)
{
    minTS = min;
    QQmlProperty(_rangeSlider, "first.value").write(0);
    startLabel->setText(minTS.toString(TIMESTAMP_FORMAT));
}
void ViewDataExport::setMaxTimestamp(QDateTime max)
{
    maxTS = max;
    QQmlProperty(_rangeSlider, "second.value").write(1000);
    endLabel->setText(maxTS.toString(TIMESTAMP_FORMAT));
}
