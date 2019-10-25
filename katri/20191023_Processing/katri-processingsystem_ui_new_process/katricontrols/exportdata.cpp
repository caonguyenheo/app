#include "exportdata.h"
#include "ui_exportdata.h"
#include <QGraphicsOpacityEffect>
#include <QDebug>

OptionExportData::OptionExportData(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exportdata)
{
    ui->setupUi(this);
//    this->setStyleSheet("border:none;");
//    setStyleSheet( "background-color:#272727;" );
//    ui->cb_rosbag2->setStyleSheet("QCheckBox {color: white; font: 13pt ;}QCheckBox::indicator {width: 15px; height: 15px; border-radius:3px;}QCheckBox::indicator::checked {background-image: url(:/img/icon/Right-select.png);}QCheckBox::indicator::unchecked {background-image: url(:/img/icon/None-select.png);}");
    ui->btn_choose->setIcon(QIcon(QPixmap(":/img/icon/choose-folder.png")));
    ui->btn_choose->setIconSize(QSize(18,25));

    QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->lineEdit_exportformat);
    dbEffect->setOpacity(0.5);
    ui->lineEdit_exportformat->setGraphicsEffect(dbEffect);
    ui->lineEdit_exportformat->setEnabled(false);

    ui->lineEdit_image->setEnabled(false);
    QGraphicsOpacityEffect* dbEffect1 = new QGraphicsOpacityEffect(ui->lineEdit_image);
    dbEffect1->setOpacity(0.5);
    ui->lineEdit_image->setGraphicsEffect(dbEffect1);

    ui->lineEdit_export->setEnabled(false);

    setEnableLidar(false);
    setEnableImage(false);
    setEnableRosbag(false);
    connect(ui->cb_extraction, SIGNAL(stateChanged(int)), this, SLOT(slotStateChanged(int)));
}

OptionExportData::~OptionExportData()
{
    delete ui;
}

void OptionExportData::openDirConnection(const QObject *receiver, const char *member)
{
    connect(ui->btn_choose,SIGNAL(pressed()),receiver,member);
}

void OptionExportData::exportConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_export,SIGNAL(pressed()),receiver,member);
}

void OptionExportData::cancelExportConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_cancel,SIGNAL(pressed()),receiver,member);
}

void OptionExportData::setDirectory(QString dir)
{
    ui->lineEdit_export->setText(dir);
}

void OptionExportData::setEnableLidar(bool status)
{
    ui->lineEdit_exportformat->setEnabled(false);
    ui->cb_xyz->setEnabled(status);
    ui->cb_pcd->setEnabled(status);
    ui->cb_ply->setEnabled(status);
    ui->cb_las->setEnabled(status);

    if(!status) {
        QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->lineEdit_exportformat);
        dbEffect->setOpacity(0.5);
        ui->lineEdit_exportformat->setGraphicsEffect(dbEffect);

        QGraphicsOpacityEffect* dbEffect1 = new QGraphicsOpacityEffect(ui->cb_xyz);
        dbEffect1->setOpacity(0.5);
        ui->cb_xyz->setGraphicsEffect(dbEffect1);

        QGraphicsOpacityEffect* dbEffect2 = new QGraphicsOpacityEffect(ui->cb_pcd);
        dbEffect2->setOpacity(0.5);
        ui->cb_pcd->setGraphicsEffect(dbEffect2);

        QGraphicsOpacityEffect* dbEffect3 = new QGraphicsOpacityEffect(ui->cb_ply);
        dbEffect3->setOpacity(0.5);
        ui->cb_ply->setGraphicsEffect(dbEffect3);

        QGraphicsOpacityEffect* dbEffect4 = new QGraphicsOpacityEffect(ui->cb_las);
        dbEffect4->setOpacity(0.5);
        ui->cb_las->setGraphicsEffect(dbEffect4);
    }
    else {
        ui->cb_xyz->setGraphicsEffect(nullptr);
        ui->cb_pcd->setGraphicsEffect(nullptr);
        ui->cb_ply->setGraphicsEffect(nullptr);
        ui->cb_las->setGraphicsEffect(nullptr);
    }
}

void OptionExportData::setEnableImage(bool status)
{
    ui->lineEdit_image->setEnabled(false);
    ui->cb_yuv->setEnabled(status);
    ui->cb_rgb->setEnabled(status);
    ui->cb_bayer->setEnabled(status);

    if(!status) {
        QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->lineEdit_image);
        dbEffect->setOpacity(0.5);
        ui->lineEdit_image->setGraphicsEffect(dbEffect);

        QGraphicsOpacityEffect* dbEffect1 = new QGraphicsOpacityEffect(ui->cb_yuv);
        dbEffect1->setOpacity(0.5);
        ui->cb_yuv->setGraphicsEffect(dbEffect1);

        QGraphicsOpacityEffect* dbEffect2 = new QGraphicsOpacityEffect(ui->cb_rgb);
        dbEffect2->setOpacity(0.5);
        ui->cb_rgb->setGraphicsEffect(dbEffect2);

        QGraphicsOpacityEffect* dbEffect3 = new QGraphicsOpacityEffect(ui->cb_bayer);
        dbEffect3->setOpacity(0.5);
        ui->cb_bayer->setGraphicsEffect(dbEffect3);
    }
    else {
        ui->cb_yuv->setGraphicsEffect(nullptr);
        ui->cb_rgb->setGraphicsEffect(nullptr);
        ui->cb_bayer->setGraphicsEffect(nullptr);
    }
}

void OptionExportData::setEnableRosbag(bool status)
{
    ui->lineEdit_ros->setEnabled(status);
    ui->lineEdit_ros2->setEnabled(status);
    ui->cb_rosbag->setEnabled(status);
    ui->cb_rosbag2->setEnabled(status);

    if(!status) {
        QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->lineEdit_image);
        dbEffect->setOpacity(0.5);
        ui->lineEdit_ros->setGraphicsEffect(dbEffect);

        QGraphicsOpacityEffect* dbEffect1 = new QGraphicsOpacityEffect(ui->cb_yuv);
        dbEffect1->setOpacity(0.5);
        ui->lineEdit_ros2->setGraphicsEffect(dbEffect1);

        QGraphicsOpacityEffect* dbEffect2 = new QGraphicsOpacityEffect(ui->cb_rgb);
        dbEffect2->setOpacity(0.5);
        ui->cb_rosbag->setGraphicsEffect(dbEffect2);

        QGraphicsOpacityEffect* dbEffect3 = new QGraphicsOpacityEffect(ui->cb_bayer);
        dbEffect3->setOpacity(0.5);
        ui->cb_rosbag2->setGraphicsEffect(dbEffect3);
    }
    else {
        ui->lineEdit_ros->setGraphicsEffect(nullptr);
        ui->lineEdit_ros2->setGraphicsEffect(nullptr);
        ui->cb_rosbag->setGraphicsEffect(nullptr);
        ui->cb_rosbag2->setGraphicsEffect(nullptr);
    }
}

QString OptionExportData::getFolderPath()
{
    return ui->lineEdit_export->text();
}

bool OptionExportData::isExtraction()
{
    return ui->cb_extraction->isChecked();
}

void OptionExportData::slotStateChanged(int status)
{
    if (status == 0) {
        // Unchecked = 0
        setEnableLidar(true);
        setEnableImage(true);
        setEnableRosbag(true);
    }
    else {
        // Checked != 0
        setEnableLidar(false);
        setEnableImage(false);
        setEnableRosbag(false);
    }
}
