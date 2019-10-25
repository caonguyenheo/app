#include "record.h"
#include "ui_record.h"

record::record(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::record)
{
    ui->setupUi(this);
}

record::~record()
{
    delete ui;
}
