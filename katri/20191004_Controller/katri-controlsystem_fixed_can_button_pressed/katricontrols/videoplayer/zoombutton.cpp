#include "zoombutton.h"
#include "ui_zoombutton.h"

zoombutton::zoombutton(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::zoombutton)
{
    ui->setupUi(this);
    this->setStyleSheet("background-color: transparent;");
    ui->fullscreenBtn->setStyleSheet("QPushButton{"
                               "background-image: url(:/img/icon/fullscreen_small_icon.png);"
                               "border-radius: 1px;"
                               "}"
                                );
    fullscreen = ui->fullscreenBtn;
}

zoombutton::~zoombutton()
{
    delete ui;
}

void zoombutton::fullScreenConnection(const QObject *receiver, const char *member)
{
    connect(ui->fullscreenBtn, SIGNAL(clicked()), receiver, member);
}

