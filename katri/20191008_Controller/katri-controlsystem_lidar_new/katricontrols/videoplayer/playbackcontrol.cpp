#include "playbackcontrol.h"
#include "ui_playbackcontrol.h"

PlaybackControl::PlaybackControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlaybackControl)
{
    ui->setupUi(this);
    this->setStyleSheet("background-color: transparent;");

    ui->seekbar->setStyleSheet(  "QSlider::groove:horizontal {border: 2px solid blue; height: 1px;background: green;margin: 0px 1px;}"
                                "QSlider::handle:horizontal {background:transparent;border-left: 7px solid red;width: 0px;height: 100px;margin:-5px -3px;border-radius:0px;}"
                                "QSlider::add-page:qlineargradient {background: white;}"
                                "QSlider::sub-page:qlineargradient {background: red;}"
                                );

    ui->playBtn->setStyleSheet( "QPushButton{"
                                "background-image: url(:/img/icon/Play-icon.png);"
                                "border-radius: 1px;"
                                "}"

                                "QPushButton:hover{"
                                "background-image: url(:/img/icon/Pause-icon.png);"
                                "border-radius:10px;"
                                "font: 14px 'Roboto';"
                                "}"
                                );

//    ui->volumeBtn->setStyleSheet("QPushButton{"
//                                "background-image: url(:/img/icon/volume-icon.png);"
//                                "border-radius: 1px;"
//                                "}"
//                                );

    ui->fullscreenBtn->setStyleSheet("QPushButton{"
                               "background-image: url(:/img/icon/Exit-Fullscreen-icon.png);"
                               "border-radius: 1px;"
                               "}"
                                );
    setPlaySeekbar(true);
    connect(ui->playBtn, SIGNAL(clicked()), this, SLOT(clickPlayBtn()));
    fullscreen = ui->fullscreenBtn;
    isPlayed = false;
}

PlaybackControl::~PlaybackControl()
{
    delete ui;
}

void PlaybackControl::fullScreenConnection(const QObject *receiver, const char *member)
{
    connect(ui->fullscreenBtn, SIGNAL(clicked()), receiver, member);
}
/**
 * @brief PlaybackControl::clickPlayBtn
 * set stylesheet button when click play or pause
 */
void PlaybackControl::clickPlayBtn()
{
    if(isPlayed == false){
        ui->playBtn->setStyleSheet( "QPushButton{"
                                    "background-image: url(:/img/icon/Pause-icon.png);"
                                    "border-radius: 1px;"
                                    "}"

                                    "QPushButton:hover{"
                                    "background-image: url(:/img/icon/Play-icon.png);"
                                    "border-radius:10px;"
                                    "font: 14px 'Roboto';"
                                    "}"
                                    );
        isPlayed = true;
    }
    else {
        ui->playBtn->setStyleSheet( "QPushButton{"
                                    "background-image: url(:/img/icon/Play-icon.png);"
                                    "border-radius: 1px;"
                                    "}"

                                    "QPushButton:hover{"
                                    "background-image: url(:/img/icon/Pause-icon.png);"
                                    "border-radius:10px;"
                                    "font: 14px 'Roboto';"
                                    "}"
                                    );
        isPlayed = false;
    }
}
void PlaybackControl::setPlaySeekbar(bool isEnable)
{
    ui->seekbar->setEnabled(isEnable);
    ui->seekbar->setValue(ui->seekbar->minimum());
    if(isEnable == false){
        ui->seekbar->setValue(ui->seekbar->maximum());
    }
}

