#include "singlemessage.h"
#include <QHBoxLayout>

SingleMessage::SingleMessage(QWidget *parent) :
    QWidget (parent)
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    this->setLayout(layout);
    layout->setContentsMargins(10,0,0,0);

    // Create Message icon
    lblicon = new QLabel(this);
    lblicon->setFixedSize(QSize(14,14));
    lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/Success-icon.png) 0 0 0 0 stretch stretch;");
    layout->addWidget(lblicon);

    // Create Message content
    lblmsg = new QLabel(this);
    lblmsg->setText("Data transferred successfully");
    lblmsg->setStyleSheet("background-color:transparent;border:none; color : green; font:12px 'Roboto';");
    layout->addWidget(lblmsg);
}

void SingleMessage::setTextMessage(StatusIcon status){
    //qDebug()<<" STATUS Message "<<status;
    switch (status) {
    case INFO:
        break;
    case SUCCESS:
        this->lblmsg->setText("Data transferred successfully");
        break;
    case WARNING:
        this->lblmsg->setText("Your storage space has reached limit. Please remove unused items or purchase more space");
        break;
    case ERROR:
        this->lblmsg->setText("Failed to transfer. Please try again");
        break;
    default:
        break;
    }
}

void SingleMessage::setStateIcon(StatusIcon status){
    //qDebug()<<" STATUS ICON "<<status;
    switch (status) {
    case INFO:
        lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/Success-icon.png) 0 0 0 0 stretch stretch;");
        break;
    case SUCCESS:
        lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/Success-icon.png) 0 0 0 0 stretch stretch;");
        break;
    case WARNING:
        lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/warning_icon.png) 0 0 0 0 stretch stretch;");
        break;
    case ERROR:
        lblicon->setStyleSheet("border : none ; border-image: url(:/img/icon/error_icon.png) 0 0 0 0 stretch stretch;");
        break;
    default:
        break;
    }
}

