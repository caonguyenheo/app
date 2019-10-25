#include "video.h"

Video::Video(QWidget *parent) :
    QWidget(parent)
{
    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
    setWindowFlags( Qt::Dialog
                  | Qt::CustomizeWindowHint
                  | Qt::WindowTitleHint
                  | Qt::WindowCloseButtonHint
                  | Qt::WindowMinimizeButtonHint
                  | Qt::WindowMaximizeButtonHint
                  );

    this->setMinimumSize(250,180);
    this->setupUI();
    frameId = -1;
    isRendering = false;
    graphicsImage = nullptr;
}

Video::~Video()
{
    if (zoom) delete zoom;
    if (graphicsImage) delete graphicsImage;
    if (fullscreen) delete fullscreen;
}

void Video::setupUI()
{
}


void Video::setbackground()
{
    if(graphicsImage != nullptr) {
        delete graphicsImage;
        graphicsImage = nullptr;
    }

    // Init video player
    graphicsImage = new QLabel(this);
//    graphicsImage->setStyleSheet("QLabel{border:0px solid green;background-color:rgb(127, 127, 127);}");
    graphicsImage->setStyleSheet("QLabel{border:0px solid green;}");
    graphicsImage->setContentsMargins(0, 0, 0, 0);
    graphicsImage->setScaledContents(true);
    QGridLayout *gLayout = new QGridLayout(this);
    this->setLayout(gLayout);
    gLayout->addWidget(graphicsImage);

    // Init zoom button
    zoom = new zoombutton(this);
    QRect rect = this->rect();
    zoom->setGeometry(rect.x(),rect.y(),rect.width(),zoom->rect().height()+260);
    zoom->setGeometry(rect.x(),rect.y(),rect.width(),zoom->rect().height());
    zoom->fullScreenConnection(this, SLOT(slotClickedMaximumSizeButton()));
}

void Video::setDataCameraView(QByteArray &data)
{
    frameId++;
    if (isRendering == false /*&& frameId%5 == 0*/) {
        // Block rendering image thread
        isRendering = true;

        try {
            // Set frame data to video player comtrol
            QPixmap pixmap;
            int width = graphicsImage->rect().width();
            int height = graphicsImage->rect().height();
            qDebug() <<width << height;
            pixmap.loadFromData(data);
            pixmap = pixmap.scaled(width, height);
            graphicsImage->setPixmap(pixmap);
//            graphicsImage->setMinimumSize(240,160);

        } catch (...) {
            qDebug()<<"Exception: " << frameId << ": ";
        }
        isRendering = false;
    }
    else {
        qDebug()<<"Skip frame: No.= " << frameId;
    }
}

void Video::resizeEvent( QResizeEvent *e )
{
    Q_UNUSED(e);
    if(zoom != nullptr){
        QRect rect = this->rect();
        zoom->setGeometry(rect.x(),rect.y(),rect.width(),zoom->rect().height());
    }
}

void Video::slotClickedMaximumSizeButton()
{
    if(fullscreen != nullptr){
        delete fullscreen;
        fullscreen = nullptr;
    }
    fullscreen = new FullScreen(this);
//    fullscreen->setbackground(pathImage);
    fullscreen->showMaximized();
}


