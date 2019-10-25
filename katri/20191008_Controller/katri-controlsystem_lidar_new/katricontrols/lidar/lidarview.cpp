#include "lidarview.h"
#include "ui_lidarview.h"

LidarView::LidarView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarView)
{
    ui->setupUi(this);
    this->InitControlUI();
}

LidarView::~LidarView()
{
    delete ui;
}

void LidarView::InitControlUI()
{
    setViewName("Lidar live stream");
    QColor color(86,86,86);
    setViewColor(color);

    //lidar 1
    ui->lidarWidget_1->setStyleSheet("border-image: url(:/img/image/lidar_3.png) 0 0 0 0 stretch stretch;");

    //lidar 2
    ui->lidarWidget_2->setStyleSheet("border-image: url(:/img/image/lidar_2.png) 0 0 0 0 stretch stretch;");

    //lidar 3
    ui->lidarWidget_3->setStyleSheet("border-image: url(:/img/image/lidar_1.png) 0 0 0 0 stretch stretch;");

}

void LidarView::backButtonConnection(const QObject *receiver, const char *member)
{
    connect(ui->Backpanel->backButton(), SIGNAL(released()), receiver, member);
}

void LidarView::setViewName(QString name)
{
    if (name.compare("") != 0)
    {
        ui->labellidarview->setText(name);
        this->setStyleMainScreenLayout();
    }
}

void LidarView::setStyleMainScreenLayout()
{
    ui->labellidarview->setStyleSheet( "color: white;"
                                    "font: 14px 'Roboto Medium';"
                                    "padding-bottom:0px;"
                                    );
    ui->labellidarview->setFixedHeight(30);
}

void LidarView::setViewColor(QColor &color)
{
    ui->lidarWidget->setStyleSheet( "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: rgb("
                                        +QString::number(color.red()) + ","
                                        +QString::number(color.green()) + ","
                                        +QString::number(color.blue()) + ");"
                                      );
}
