#include "backpanel.h"
#include "ui_backpanel.h"

BackPanel::BackPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BackPanel)
{
    ui->setupUi(this);

    this->setStyleSheet("background-color: #363636;");
    ui->pushButtonBack->setIcon(QIcon(":/img/icon/back-icon.png"));
    ui->pushButtonBack->setIconSize(QSize(21,11));
    ui->pushButtonBack->setStyleSheet("border:none; color:white; font:13px 'Roboto';");
}

BackPanel::~BackPanel()
{
    delete ui;
}

void BackPanel::backConnection(const QObject *receiver, const char *member)
{
    connect(ui->pushButtonBack, SIGNAL(released()), receiver, member);
}
