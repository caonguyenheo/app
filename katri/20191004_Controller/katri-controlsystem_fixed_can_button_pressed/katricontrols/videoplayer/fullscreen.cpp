#include "fullscreen.h"

FullScreen::FullScreen(QWidget *parent) :
    QWidget(parent)
{
    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
    setWindowFlags( Qt::Dialog
                  | Qt::CustomizeWindowHint
                  );

    // Set background color
    setStyleSheet( "background-color:rgb(0,0,0);" );
    this->setMinimumSize(1280,720);
    this->setupUI();
}

FullScreen::~FullScreen()
{

}

void FullScreen::setupUI()
{
}


void FullScreen::setbackground(QString path)
{
    QLabel *label = new QLabel(this);
    QGridLayout *gLayout = new QGridLayout(this);
    this->setLayout(gLayout);
    gLayout->addWidget(label);
    playback = new PlaybackControl(this);
//    gLayout->addWidget(playback);
    QRect rect = this->rect();
    playback->setGeometry(rect.x()+40,rect.y()+rect.height()-80,rect.width()-80,playback->rect().height());
    playback->setPlaySeekbar(false);
    QPixmap pixmap(path);
    label->setPixmap(pixmap);
    label->setScaledContents(true);
    label->setStyleSheet("border:1px solid #88888b;");
    this->layout()->setMargin(0);
    playback->fullScreenConnection(this,SLOT(slotClickedMaximumSizeButton()));
    this->showMaximized();
}

void FullScreen::resizeEvent( QResizeEvent *e )
{
    Q_UNUSED(e);
    if(playback != nullptr){
        QRect rect = this->rect();
        playback->setGeometry(rect.x()+40,rect.y()+rect.height()-80,rect.width()-80,playback->rect().height());
        playback->setPlaySeekbar(false);
    }
}

void FullScreen::slotClickedMaximumSizeButton()
{
    this->hide();
}

