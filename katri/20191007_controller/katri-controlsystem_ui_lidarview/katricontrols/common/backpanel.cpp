#include "backpanel.h"
#include "ui_backpanel.h"

BackPanel::BackPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BackPanel)
{
    ui->setupUi(this);

    this->setStyleSheet("background-color: #363636;");
    QIcon icon(":/img/icon/back-icon.png");
    ui->pushButtonBack->setIcon(icon);
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

void BackPanel::setDoublePanel()
{
    ui->widget_2->setStyleSheet("background-color: rgb(48, 48, 48);");
}

QPushButton* BackPanel::backButton()
{
    return ui->pushButtonBack;
}
