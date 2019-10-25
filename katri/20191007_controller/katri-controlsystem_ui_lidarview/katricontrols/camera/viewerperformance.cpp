#include "viewerperformance.h"
#include "ui_viewerperformance.h"
#include "graph/navigationmanager.h"
#include "ui_navigationmanager.h"
#include <QDebug>

viewerperformance::viewerperformance(QWidget *parent, int id) :
    QWidget(parent),
    ui(new Ui::viewerperformance),
    device_id(id)
{
    ui->setupUi(this);
    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
    setWindowFlags( Qt::Dialog
                  | Qt::CustomizeWindowHint
                  | Qt::WindowTitleHint
                  | Qt::WindowCloseButtonHint
                  | Qt::WindowMinimizeButtonHint
                  | Qt::WindowMaximizeButtonHint
                  );
    initvalue();
    addbuttonrecord();
    //connect(this,SIGNAL(signalChangeStatus()),parent,SLOT(slotStatusChanged()));
}

viewerperformance::~viewerperformance()
{
    delete ui;
}

void viewerperformance::initvalue()
{
    //Example
    ui->label_IP->setText("Ip");
    ui->label_IP->setStyleSheet("font:12px 'Roboto'; color:#9a9a9a; border:0;height:20px;");

    ui->label_IVSB->setText("I-VSB #10");
    ui->label_IVSB->setStyleSheet("font:14px 'Roboto'; color:#272727; border:0; font-weight:bold;height:20px;");

    ui->label_IP_number->setText("192.168.0.0");
    ui->label_IP_number->setStyleSheet("font-family: 'Roboto'; font-size:10px; color:#323030; border:0; background-color:#d2d2d3;height:20px;");

}

void viewerperformance::addbuttonrecord()
{
    rec = new RecordButton(this,device_id);
    connect(rec->getRecordButton(), SIGNAL(clicked(bool)), this, SLOT(slotRecordClicked(bool)));
//    rec->setStyleSheet_performance_window();
    rec->setName("");
}

void viewerperformance::updateStyle(bool status)
{
    if(status){
        ui->info->setStyleSheet("border:2px solid #ec883c;background-color:#f5f6fa;border-radius:0;");
    }
    else{
        ui->info->setStyleSheet("border:2px solid #ababab;background-color:#f5f6fa;border-radius:0;");
    }
}

void viewerperformance::setActivate(bool status)
{
    rec->setActive(status);
}

void viewerperformance::setChecked(bool status)
{
    qDebug()<<"viewerperformance setChecked status = "<<status;
    updateStyle(status);
    rec->setChecked(status);
}

bool viewerperformance::isActivated()
{
    return rec->getStatusButton();
}

void viewerperformance::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if (rec != nullptr) {
        QPoint point = QPoint(this->rect().width() - 72, 29);
        rec->setGeometry(point.x(), point.y(), rec->geometry().width(), rec->geometry().height());
    }
}

void viewerperformance::setIP(QString name)
{
    ui->label_IP_number->setText(name);
    ui->label_IP_number->setStyleSheet("font: 10px 'Roboto'; color:#323030; border:0; background-color:#d2d2d3;height:20px;padding-left:7px;");
}

void viewerperformance::setname(QString name)
{
    ui->label_IVSB->setText(name);
    ui->label_IVSB->setStyleSheet("font:14px 'Roboto'; color:#272727; border:0; font-weight:bold;height:20px;");
}

void viewerperformance::setDeviceId(int id)
{
    device_id = id;
}

int viewerperformance::getDeviceId()
{
    return device_id;
}

QWidget *viewerperformance::getVideoObject()
{
    return widgetVideo;
}

void viewerperformance::addwidgetright(QWidget * widget)
{
    ui->informationLayout->addWidget(widget, 0, 0);
    ui->informationLayout->setMargin(0);
}

void viewerperformance::addwidgetleft(QWidget * widget)
{
    ui->videoLayout->addWidget(widget, 0, 0);
    ui->videoLayout->setMargin(0);
    ui->videoLayout->setContentsMargins(0,0,0,0);
    widgetVideo = widget;
}

void viewerperformance::slotRecordClicked(bool status)
{
    Q_UNUSED(status);
    emit signalChangeStatus(isActivated(), getDeviceId());
    updateStyle(status);
}

void viewerperformance::sendCommandConnection(const QObject *receiver, const char *member)
{
    connect(this, SIGNAL(signalChangeStatus(bool, int)), receiver, member);
}
