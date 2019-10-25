#include "menubar.h"
#include "ui_menubar.h"
#include <QDebug>
#include <QDragEnterEvent>

MenuBar::MenuBar(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MenuBar)
{
    ui->setupUi(this);
    this->setStyleSheet("background-color : white; border-color: white;");
    ui->lbl_name->setText("KATRI DATA LOGGER FOR CAR");
    ui->lbl_name->setContentsMargins(9,0,0,0);
    ui->lbl_name->setStyleSheet("font : 12px 'Roboto;");
    ui->btnExit->setFixedHeight(ui->lbl_name->height());
    ui->btnMinimum->setFixedHeight(ui->lbl_name->height());
    ui->btnMaximum->setFixedHeight(ui->lbl_name->height());
    ui->btnExit->setIcon(QIcon(":/img/icon/Close-ic.png"));
    ui->btnMinimum->setIcon(QIcon(":/img/icon/Minimize-ic.png"));
    ui->btnMaximum->setIcon(QIcon(":/img/icon/Maximize-ic.png"));


    ui->btnExit->setStyleSheet("padding-left:10px;padding-right:16px;border:0");
    ui->btnMinimum->setStyleSheet("padding-left:10px;padding-right:10px;border:0");
    ui->btnMaximum->setStyleSheet("padding-left:10px;padding-right:10px;border:0");

    pBtnMax = ui->btnMaximum;
}

MenuBar::~MenuBar()
{
    delete ui;
}

void MenuBar::exitConnection(const QObject *receiver, const char *member)
{
    connect(ui->btnExit, SIGNAL(clicked()), receiver, member);
}

void MenuBar::minimumSizeConnection(const QObject *receiver, const char *member)
{
    connect(ui->btnMinimum, SIGNAL(clicked()), receiver, member);
}

void MenuBar::maximumSizeConnection(const QObject *receiver, const char *member)
{
    connect(ui->btnMaximum, SIGNAL(clicked()), receiver, member);
}
