#include "imageview.h"
#include "../common/common.h"
#include<bits/stdc++.h>
#include <QtDebug>
#include "ui_imageview.h"
#include <QColor>

const int VIEWER_NUM = 4;

ImageView::ImageView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageView)
{
    ui->setupUi(this);

    // Create 4 camera views
    for (int i = 0; i < VIEWER_NUM; i++) {
        GLWidgetImage* glWidget = new GLWidgetImage(ui->imagePanel);
        m_viewList.insert(i, glWidget);
    }
}

ImageView::~ImageView()
{
    delete ui;
}

void ImageView::setViewName(QString &name)
{
    if (name.compare("") != 0) {
        ui->imageLabel->setText(name);
        this->setStyleMainScreenLayout();
    }
}

void ImageView::setViewColor(QColor &color)
{
    ui->imagePanel->setStyleSheet( "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: rgb("
                                        +QString::number(color.red()) + ","
                                        +QString::number(color.green()) + ","
                                        +QString::number(color.blue()) + ");"
                                      );
}

void ImageView::setStyleMainScreenLayout()
{
    ui->imageLabel->setStyleSheet( "color: white;"
                                      "font: 14px 'Roboto';"
                                      "font-weight:500;"
                                      "padding-bottom:7px;"
                                    );
    ui->imageLabel->setFixedHeight(30);
}

void ImageView::setStyleFullScreenLayout()
{
    ui->imageLabel->setStyleSheet( "color: white;"
                                    "font: 32px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:5px;"
                                    );
    ui->imageLabel->setFixedHeight(55);
}

void ImageView::appendStyleSheet(QString stylesheet)
{
    ui->imagePanel->setStyleSheet(ui->imagePanel->styleSheet().append(stylesheet));
}

void ImageView::setImageSimulation(bool display)
{
    if(lbicon == nullptr) {
        lbicon = new QLabel(this);
        movie = new QMovie(":/img/icon/Loading_iCon.gif");
        lbicon->setAttribute( Qt::WA_TranslucentBackground, true );
        lbicon->setMovie(movie);
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28 , 28);
        lbicon->setMaximumSize(QSize(28,28));
    }
    if(lbicon != nullptr) {
        if(display) {
            lbicon->show();
            movie->start();
        }
        else {
            lbicon->hide();
            movie->stop();
        }
    }

}

void ImageView::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    if(lbicon != nullptr) {
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28 ,28);
    }
}

void ImageView::setCameraLayout()
{
    // Add GridLayout

    gridLayout = new QGridLayout();
    gridLayout->setContentsMargins(15,30,15,30);
    gridLayout->setVerticalSpacing(10);
    gridLayout->setHorizontalSpacing(10);
    //gridLayout->setMargin(15);

    ui->verticalLayout->addItem(gridLayout);
    // Configuration layout
    int maxView = VIEWER_NUM;
    int getWidth = int(ceil(sqrt(maxView)));
    for (int i = 0; i < maxView; i++) {
        int h = (i) / getWidth;
        int w = (i) % getWidth;

        GLWidgetImage *glWidget = m_viewList[i];
        if (glWidget) {
            QWidget *widget = new QWidget(ui->imagePanel);
            QGridLayout *layout = new QGridLayout();
            layout->setSpacing(0);
            layout->setMargin(2);
            layout ->addWidget(glWidget);
            widget->setLayout(layout);
            widget->setStyleSheet("border-style:solid;"
                                  "border-width:2px;"
                                  "border-color:" +
                                  convertColorToString(PLAYERBORDERCOLOR) + ";"
                                  "background-color:" +
                                  convertColorToString(PLAYERBACKGROUNDCOLOR) + ";");

            gridLayout->addWidget(widget, h, w, 1, 1);
        }
    }
}

void ImageView::setSVMLayout()
{
//    QHBoxLayout *hLayout = new QHBoxLayout(ui->playbackPanel);
    hLayout = new QHBoxLayout();
    hLayout->setObjectName(QString::fromUtf8("hLayout"));
    hLayout->setContentsMargins(6,26,6,26);
    //ui->imagePanel->setLayout(hLayout);
    ui->verticalLayout->addItem(hLayout);
    // mini layout 1
//    QVBoxLayout *vLayout1 = new QVBoxLayout();
    vLayout1 = new QVBoxLayout();
    vLayout1->setObjectName(QString::fromUtf8("v1"));
    vLayout1->setContentsMargins(0,47,0,47);
    vLayout1->addWidget(m_viewList[1]);
    hLayout->addLayout(vLayout1);

    // mini layout 2
    QVBoxLayout *vLayout2 = new QVBoxLayout();
    vLayout2->setObjectName(QString::fromUtf8("v2"));
    vLayout2->addWidget(m_viewList[0]);
    vLayout2->addWidget(m_viewList[2]);

    hLayout->addLayout(vLayout2);

    // mini layout 2
//    QVBoxLayout *vLayout3 = new QVBoxLayout();
    vLayout3 = new QVBoxLayout();
    vLayout3->setObjectName(QString::fromUtf8("v3"));
    vLayout3->setContentsMargins(0,47,0,47);
    vLayout3->addWidget(m_viewList[3]);

    hLayout->addLayout(vLayout3);

    // Set stretch layout
    hLayout->setStretch(0, 720);
    hLayout->setStretch(1, 1280);
    hLayout->setStretch(2, 720);
    hLayout->setSpacing(10);
}

void ImageView::setDataCamera(int deviceId, QByteArray &data)
{
    qDebug()<<"[viewdatacamera---Data Camera---]:device id: "<< deviceId;
    GLWidgetImage *videoObj = dynamic_cast<GLWidgetImage*>(m_viewList[deviceId]);
    if (videoObj) {
        videoObj->setDataCameraView(data);
    }
}
