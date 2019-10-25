#include "canbusview.h"
#include "ui_canbusview.h"
#include "common.h"
#include <QDebug>

CANBusView::CANBusView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CANBusView)
{
    ui->setupUi(this);
    setCanLayout();
    m_dataCache = new QStringList();
    m_dataRowCount = 13;
}

CANBusView::~CANBusView()
{
    delete ui;
}

void CANBusView::setCanLayout()
{
    CANBusLabel = new QLabel(ui->CANBusViewPanel);

    CANBusLabel->setStyleSheet( "color: black;"
                                "font: 8px 'Roboto';"
                                "font-weight:500;"
                                "padding-bottom:4px;"
                                "background-color:white;");


   CANBusLabel->setScaledContents(true);
   CANBusLabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
   CANBusLabel->setAlignment(Qt::AlignTop);
   hLayout = new QHBoxLayout();
   hLayout->setObjectName(QString::fromUtf8("hLayout"));
   hLayout->setContentsMargins(0,0,0,0);
   hLayout->addWidget(CANBusLabel,Qt::AlignHorizontal_Mask);

   ui->verticalLayout->addItem(hLayout);
}


void CANBusView::setViewName(QString &name)
{
    if (name.compare("") != 0) {
        ui->CANBusViewLabel->setText(name);
        this->setStyleMainScreenLayout();
    }
}

void CANBusView::setViewColor(QColor &color)
{
    ui->CANBusViewPanel->setStyleSheet( "border-image:none;"
                                        "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: rgb("
                                        +QString::number(color.red()) + ","
                                        +QString::number(color.green()) + ","
                                        +QString::number(color.blue()) + ");"
                                      );
}


void CANBusView::setStyleMainScreenLayout()
{
    //title
    ui->CANBusViewLabel->setStyleSheet( "color: white;"
                                    "font: 14px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:7px;"
                                    );
    ui->CANBusViewLabel->setFixedHeight(29);
    //data text
    CANBusLabel->setStyleSheet("font: 12px 'Roboto';"
                                "font-weight:500;"
                                "padding-bottom:7px;"
                                );

    //remove shadow background
    ui->CANBusViewPanel->setStyleSheet( "border-image:none;"
                                        "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: white;"
                                      );

}
void CANBusView::setStyleFullScreenLayout()
{
    //title
    ui->CANBusViewLabel->setStyleSheet( "color: white;"
                                    "font: 32px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:5px;"
                                    );
    ui->CANBusViewLabel->setFixedHeight(55);
    //data text
    CANBusLabel->setStyleSheet( "font: 20px 'Roboto';"
                                "font-weight:500;"
                                "padding-bottom:7px;"
                               );
    //remove shadow background
    ui->CANBusViewPanel->setStyleSheet( "border-image:none;"
                                        "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: white;"
                                      );
}

void CANBusView::appendStyleSheet(QString stylesheet)
{
    ui->CANBusViewPanel->setStyleSheet(ui->CANBusViewPanel->styleSheet().append(stylesheet));
}

void CANBusView::setImageSimulation(bool display)
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

void CANBusView::resizeEvent(QResizeEvent *event)
{
    if(event->size().width() <= 170)
    {
        CANBusLabel->setStyleSheet( "color: black;"
                                    "font: 10px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:2px;"
                                    "padding-top:2px;"
                                    "background-color:white;");
    }
    else if(event->size().width() <= 360)
    {
        CANBusLabel->setStyleSheet( "color: black;"
                                    "font: 12px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-top:4px;"
                                    "background-color:white;");
    }
//    else if(event->size().width() <= 437)
//    {
//        CANBusLabel->setStyleSheet( "color: black;"
//                                    "font: 16px 'Roboto';"
//                                    "font-weight:500;"
//                                    "padding-top:10px;"
//                                    "background-color:white;");
//    }
    else if(event->size().width() <= 720)
    {
        CANBusLabel->setStyleSheet( "color: black;"
                                    "font: 14px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-top:4px;"
                                    "background-color:white;");
    }
    else if(event->size().width() <= 1530)
    {
        CANBusLabel->setStyleSheet( "color: black;"
                                    "font: 20px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-top:4px;"
                                    "background-color:white;");
    }
    if(lbicon != nullptr) {
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28, 28);
    }
}

void CANBusView::slotReceiveTimestamp(int key, QString value)
{
    Q_UNUSED(key);
    QString show = appendDataToCache(value);
    CANBusLabel->setText(show);
    emit signalFinishedRender();
}

QString CANBusView::appendDataToCache(QString value)
{
    m_dataCache->push_back(value);

    // Keep smallest Cache
    if (m_dataCache->count() > m_dataRowCount) {
        m_dataCache->pop_front();
    }

    return m_dataCache->join("\n");
}
