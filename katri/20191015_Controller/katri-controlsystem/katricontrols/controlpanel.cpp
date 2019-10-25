#include "controlpanel.h"
#include "ui_controlpanel.h"

ControlPanel::ControlPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlPanel)
{
    ui->setupUi(this);
    ui->widget->setStyleSheet("background-color:#2f2f2f; font-family:'Roboto';");
    ui->pb_relay->setCheckable(true);
    ui->pb_record->setCheckable(true);
    ui->pb_stop->setCheckable(true);


    myMovie = new QMovie(":/img/icon/Relaying-btn.gif");
    connect(myMovie,SIGNAL(frameChanged(int)),this,SLOT(setButtonIcon()));
    // if movie doesn't loop forever, force it to.
    if (myMovie->loopCount() != -1)
        connect(myMovie,SIGNAL(finished()),myMovie,SLOT(start()));
}

ControlPanel::~ControlPanel()
{
    delete ui;
}

void ControlPanel::recordAllConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_record, SIGNAL(clicked(bool)), receiver, member);
}

void ControlPanel::stopAllConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_stop, SIGNAL(clicked(bool)), receiver, member);
}

void ControlPanel::relayAllConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_relay, SIGNAL(clicked(bool)), receiver, member);
}


void ControlPanel::StartRecord(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->pb_stop->setChecked(0);
        ui->pb_stop->clicked(0);
        ui->pb_record->setEnabled(0);
        ui->pb_stop->setEnabled(1);
        ui->pb_relay->setEnabled(0);
        //ui->label_relay->setStyleSheet("QLabel{color:#6e6e6e;font: 14px 'Roboto'; font-weight:bold;}");
        updateLayoutStyles(false, ui->label_relay);

    }
}

void ControlPanel::StopRecord(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->pb_record->setChecked(0);
        ui->pb_record->clicked(0);
        ui->pb_record->setEnabled(1);
        ui->pb_stop->setEnabled(0);
        ui->pb_relay->setEnabled(1);
        updateLayoutStyles(true, ui->label_relay);
    }
}

void ControlPanel::RelayRecord(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->pb_relay->setEnabled(0);
        ui->pb_record->setEnabled(1);
        ui->pb_stop->setEnabled(0);
    }
}

void ControlPanel::Stoprecording()
{
    ui->pb_stop->setChecked(0);
    ui->pb_stop->clicked(0);
    ui->pb_record->setChecked(1);
    ui->pb_record->clicked(1);
    ui->pb_record->setEnabled(0);
    ui->pb_stop->setEnabled(0);
    ui->pb_relay->setEnabled(0);
//    ui->label_relay->setStyleSheet("QLabel{color:#ffffff;font: 14px 'Roboto'; font-weight:bold;}");
    updateLayoutStyles(true, ui->label_relay);

}

void ControlPanel::updateStatusRecordButton(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->label_record->setStyleSheet("color : #ec883e;font: 14px 'Roboto'; font-weight:bold;");
    }
    else{
//        ui->label_record->setStyleSheet("color : white;font: 14px 'Roboto'; font-weight:bold;");
        updateLayoutStyles(true, ui->label_record);
    }
}
void ControlPanel::updateStatusStopButton(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->label_stop->setStyleSheet("color : #fc1218;font: 14px 'Roboto'; font-weight:bold;");
        ui->pb_stop->setStyleSheet("QPushButton{background-image: url(:/img/icon/Stop-btn-V2.jpg);border:0;font-size:1px;}\n"
                                   "QPushButton::checked{\n"
                                   "background-image: url(:/img/icon/Stopped-btn-V2.jpg);}\n"
                                   "QPushButton:disabled {background-image: url(:/img/icon/Stopped-btn-V2.jpg);}");
    }
    else{
//        ui->label_stop->setStyleSheet("color : white;font: 14px 'Roboto'; font-weight:bold;");
        updateLayoutStyles(true, ui->label_stop);
    }
}

void ControlPanel::updateStatusRelayButton(QString Text)
{
    ui->label_relay->setText(Text);
}

void ControlPanel::setRelayEnabled (bool status)
{
    ui->pb_relay->setEnabled(status);
}

void ControlPanel::startRelayingStatus()
{
    qDebug() << "Start Relaying process";
    //    set UI Layout Styles
    ui->pb_record->setEnabled(0);
    updateLayoutStyles(false, ui->label_record);
    ui->pb_stop->setEnabled(0);
    updateLayoutStyles(false, ui->label_stop);
    //ui->label_relay->setEnabled(0);
    updateLayoutStyles(false, ui->label_relay);

    myMovie->start();

    //QTimer::singleShot(time, this, SLOT(stopRelayingStatus()));
}

void ControlPanel::setButtonIcon()
{
    ui->pb_relay->setIcon(QIcon(myMovie->currentPixmap()));
    ui->pb_relay->setIconSize(QSize(145,100));
}

void ControlPanel::stopRelayingStatus()
{
    qDebug() << "Stop Relaying process";
    ui->pb_record->blockSignals(true);
    ui->pb_stop->blockSignals(true);
    ui->pb_relay->blockSignals(true);

    ui->pb_record->setChecked(0);
    ui->pb_record->clicked(0);
    ui->pb_record->setEnabled(1);
    ui->pb_stop->setEnabled(1);
    //ui->pb_relay->setEnabled(0);
    ui->pb_relay->setChecked(0);
    ui->pb_relay->clicked(0);

    ui->pb_record->blockSignals(false);
    ui->pb_stop->blockSignals(false);
    ui->pb_relay->blockSignals(false);

//    ui->label_record->setStyleSheet("QLabel{color:#ffffff;}");
//    ui->label_stop->setStyleSheet("QLabel{color:#ffffff;}");
//    ui->label_relay->setStyleSheet("QLabel{color:#ffffff;}");
//    set UI Layout Styles
    updateLayoutStyles(true, ui->label_record);
    updateLayoutStyles(true, ui->label_stop);
    updateLayoutStyles(true, ui->label_relay);

    myMovie->stop();
    ui->pb_relay->setIcon(QIcon());
}

//void ControlPanel::stoploading()
//{
//    myMovie->stop();
//    ui->pb_relay->setIcon(QIcon());
//}

void ControlPanel::updateLayoutStyles(bool active, QLabel *label){
    if(active){
        label->setStyleSheet("color:#ffffff;font: 14px 'Roboto'; font-weight:bold;");
    }
    else {
        label->setStyleSheet("color:#6e6e6e;font: 14px 'Roboto'; font-weight:bold;");
    }
}
