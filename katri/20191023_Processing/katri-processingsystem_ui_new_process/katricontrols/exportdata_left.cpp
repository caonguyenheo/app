#include "exportdata_left.h"
#include "ui_exportdata_left.h"
#include <QDebug>

exportdata_left::exportdata_left(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exportdata_left)
{
    ui->setupUi(this);

    this->setStyleSheet("border:none; border-radius:0");
}

exportdata_left::~exportdata_left()
{
    delete ui;
}
void exportdata_left::setView_exportname(QString name)
{
    ui->lb_camera->setText(name);
}
void exportdata_left::setView_exportimage(QString path)
{
    QString style = "border-image:url(:";
    style += path;
    style += ")";
    style += "0 0 0 0 stretch stretch;";
//    qDebug() << style;
    ui->widget_camera->setStyleSheet(style);
}

bool exportdata_left::getStatus()
{
    return ui->cb_camera->isChecked();
}

void exportdata_left::selectConnection(const QObject *receiver, const char *member)
{
    connect(ui->cb_camera,SIGNAL(clicked(bool)),receiver,member);
}
