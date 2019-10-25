#include "lidar.h"
#include "ui_lidar.h"

LiDar::LiDar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LiDar)
{
    ui->setupUi(this);
}

LiDar::~LiDar()
{
    delete ui;
}
