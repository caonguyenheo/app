#include "record.h"
#include "ui_record.h"
#include "common/common.h"

record::record(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::record)
{
    ui->setupUi(this);
    this->InitUi();
}

record::~record()
{
    delete ui;
}

void record::InitUi()
{
    //set name of tab
    ui->tabWidget->setTabText(0, NAME_TABSYSTEMSTATUS);
    ui->tabWidget->setTabText(1, NAME_TABRECORD);
    ui->tabWidget->setTabText(2, NAME_TABRELAYALL);
}

/*
* @return list contanning the checkbox
*/
QList<QCheckBox*> record::getListCheckbox()
{
    QList <QCheckBox*> listchecboxdeviced;

    //checkbox part Lidar
    listchecboxdeviced.append(ui->cb_front_lidar);
    listchecboxdeviced.append(ui->cb_top_lidar);
    listchecboxdeviced.append(ui->cb_rear_lidar);

    //checkbox part Serial
    listchecboxdeviced.append(ui->cb_can_serial);
    listchecboxdeviced.append(ui->cb_mobileye_serial);
    listchecboxdeviced.append(ui->cb_weather_serial);

    //checkbox part Radar
    listchecboxdeviced.append(ui->cb_frontcenter_radar);
    listchecboxdeviced.append(ui->cb_frontleft_radar);
    listchecboxdeviced.append(ui->cb_frontright_radar);
    listchecboxdeviced.append(ui->cb_rearleft_radar);
    listchecboxdeviced.append(ui->cb_rearright_radar);

    //checkbox part FHD Camera
    listchecboxdeviced.append(ui->cb_front_fhd);
    listchecboxdeviced.append(ui->cb_left_fhd);
    listchecboxdeviced.append(ui->cb_right_fhd);
    listchecboxdeviced.append(ui->cb_rear_fhd);

    //checkbox part SVM Camera
    listchecboxdeviced.append(ui->cb_front_svm);
    listchecboxdeviced.append(ui->cb_left_svm);
    listchecboxdeviced.append(ui->cb_right_svm);
    listchecboxdeviced.append(ui->cb_rear_svm);

    return listchecboxdeviced;
}

/*
* @checked the checkbox is ischecked
*/
void record::clickcheckbox(QCheckBox *cb_box, const QObject *receiver, const char *member)
{
    connect(cb_box, SIGNAL(clicked(bool)), receiver, member);
}
