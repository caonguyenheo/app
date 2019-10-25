#include "viewcommand.h"
#include "ui_viewcommand.h"

viewcommand::viewcommand(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::viewcommand)
{
    ui->setupUi(this);
}

viewcommand::~viewcommand()
{
    delete ui;
}
