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

void ControlPanel::UpdateStatusButton(int status)
{
    if (status == INIT) {
        /*
         * START All = ENABLE
         * STOP All = ENABLE
         * RELAY All = ENABLE
         */

        //qDebug() <<"Status = INIT";
        ui->pb_record->setEnabled(true);
        ui->pb_stop->setEnabled(true);
        ui->pb_relay->setEnabled(true);
    }
    else if (status == RECORD_ALL) {
        /*
         * START All = DISABLE
         * STOP All = ENABLE
         * RELAY All = DISABLE
         */

        //qDebug() <<"Status = RECORD_ALL";
        ui->pb_stop->setChecked(false);
        ui->pb_stop->clicked(false);
        ui->pb_record->setEnabled(false);
        ui->pb_record->setChecked(false);
        ui->pb_stop->setEnabled(true);
        ui->pb_relay->setEnabled(false);
        updateLayoutStyles(false, ui->label_relay);
        updateStatusRecordButton(true);
    }
    else if (status == STOP_ALL) {
        /*
         * START All = ENABLE
         * STOP All = DISABLE
         * RELAY All = ENABLE
         */

        //qDebug() <<"Status = STOP_ALL";
        ui->pb_record->setChecked(false);
        ui->pb_record->clicked(false);
        ui->pb_record->setEnabled(true);
        ui->pb_stop->setEnabled(false);
        ui->pb_relay->setEnabled(true);
        updateLayoutStyles(true, ui->label_relay);
    }
    else if (status == DETECT_RECORD) {
        /*
         * START All = ENABLE
         * STOP All = ENABLE
         * RELAY All = DISABLE
         */

        //qDebug() <<"Status = DETECT_RECORD";
        ui->pb_stop->setChecked(false);
        ui->pb_stop->clicked(false);
        ui->pb_record->setEnabled(true);
        ui->pb_stop->setEnabled(true);
        ui->pb_relay->setEnabled(false);
        updateLayoutStyles(false, ui->label_relay);
        updateStatusRecordButton(false);
    }
    else if (status == DETECT_STOP) {
        /*
         * START All = ENABLE
         * STOP All = DISABLE
         * RELAY All = ENABLE
         */

        //qDebug() <<"Status = DETECT_STOP";
        ui->pb_record->setChecked(false);
        ui->pb_record->setEnabled(true);
        ui->pb_stop->setChecked(true);
        ui->pb_stop->setEnabled(false);
        ui->pb_relay->setEnabled(true);
        updateLayoutStyles(true, ui->label_relay);
        updateStatusStopButton(true);
    }
    else if (status == RELAYING) {
        /*
         * START All = DISABLE
         * STOP All = DISABLE
         * RELAY All = ENABLE
         */

        //qDebug() << "Start Relaying process";
        // set UI Layout Styles
        ui->pb_record->setEnabled(0);
        ui->pb_stop->setEnabled(0);
        updateLayoutStyles(false, ui->label_record);
        updateLayoutStyles(false, ui->label_stop);
        updateLayoutStyles(false, ui->label_relay);
        updateStyleRelay(true);
        myMovie->start();
    }
    else if (status == RELAY_STOP) {
        /*
         * START All = ENABLE
         * STOP All = ENABLE
         * RELAY All = ENABLE
         */

        //qDebug() << "Stop Relaying process";
        ui->pb_record->blockSignals(true);
        ui->pb_stop->blockSignals(true);
        ui->pb_relay->blockSignals(true);

        ui->pb_record->setChecked(0);
        ui->pb_record->clicked(0);
        ui->pb_record->setEnabled(1);
        ui->pb_stop->setEnabled(1);
        ui->pb_relay->setChecked(0);
        ui->pb_relay->clicked(0);        

        ui->pb_record->blockSignals(false);
        ui->pb_stop->blockSignals(false);
        ui->pb_relay->blockSignals(false);

        // set UI Layout Styles
        updateLayoutStyles(true, ui->label_record);
        updateLayoutStyles(true, ui->label_stop);
        updateLayoutStyles(true, ui->label_relay);
        updateStyleRelay(false);

        myMovie->stop();
        ui->pb_relay->setIcon(QIcon());
    }
}

void ControlPanel::updateStatusRecordButton(bool status)
{
    Q_UNUSED(status);
    if (status){
        ui->label_record->setStyleSheet("color : #ec883e;font: 14px 'Roboto'; font-weight:bold;");
        ui->pb_record->setStyleSheet("QPushButton{background-image: url(:/img/icon/Record-btn-V2.jpg);border:0;font-size:1px;}\n"
                                   "QPushButton::checked{\n"
                                   "background-image: url(:/img/icon/Recording-btn-V2.jpg);}\n"
                                   "QPushButton:disabled {background-image: url(:/img/icon/Recording-btn-V2.jpg);}");
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

void ControlPanel::setRelayEnabled (bool status)
{
    ui->pb_relay->setEnabled(status);
}

void ControlPanel::setButtonIcon()
{
    ui->pb_relay->setIcon(QIcon(myMovie->currentPixmap()));
    ui->pb_relay->setIconSize(QSize(145,100));
}

void ControlPanel::updateStyleRelay(bool status)
{
    if (status) {
        ui->pb_record->setStyleSheet("QPushButton{background-image: url(:/img/icon/Record-btn-inv.jpg);border:0;font-size:1px;}\n");
        ui->pb_stop->setStyleSheet("QPushButton{background-image: url(:/img/icon/Stop-btn-inv.jpg);border:0;font-size:1px;}\n");
    }
    else {
        ui->pb_record->setStyleSheet("QPushButton{background-image: url(:/img/icon/Record-btn-V2.jpg);border:0;font-size:1px;}\n");
        ui->pb_stop->setStyleSheet("QPushButton{background-image: url(:/img/icon/Stop-btn-V2.jpg);border:0;font-size:1px;}\n");
    }
}

void ControlPanel::updateLayoutStyles(bool active, QLabel *label){
    if(active){
        label->setStyleSheet("color:#ffffff;font: 14px 'Roboto'; font-weight:bold;");
    }
    else {
        label->setStyleSheet("color:#6e6e6e;font: 14px 'Roboto'; font-weight:bold;");
    }
}
