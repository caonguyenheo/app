#include "simulationview.h"
#include "ui_simulationview.h"
#include "common.h"

SimulationView::SimulationView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SimulationView)
{
    ui->setupUi(this);
    //this->setMinimumSize(170,180);

}

SimulationView::~SimulationView()
{
    delete ui;
}

void SimulationView::setViewName(QString &name)
{
    if (name.compare("") != 0) {
        ui->simulationLabel->setText(name);
        this->setStyleMainScreenLayout();
    }
}
void SimulationView::setImage(QLabel *image)
{
   ui->verticalLayout_3->addWidget(image);
}


void SimulationView::setViewColor(QColor &color)
{
    ui->simulationPanel->setStyleSheet( "border-style:solid;"
                                        "border-width:4px;"
                                        "border-color:" + convertColorToString(MAINPANELBORDERCOLOR) + ";"
                                        "background-color: rgb("
                                        +QString::number(color.red()) + ","
                                        +QString::number(color.green()) + ","
                                        +QString::number(color.blue()) + ");"
                                      );
}

void SimulationView::setStyleMainScreenLayout()
{
    ui->simulationLabel->setStyleSheet( "color: white;"
                                    "font: 14px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:7px;"
                                    );
    ui->simulationLabel->setFixedHeight(29);
}
void SimulationView::setStyleFullScreenLayout()
{
    ui->simulationLabel->setStyleSheet( "color: white;"
                                    "font: 32px 'Roboto';"
                                    "font-weight:500;"
                                    "padding-bottom:5px;"
                                    );
    ui->simulationLabel->setFixedHeight(55);
}


void SimulationView::appendStyleSheet(QString stylesheet)
{
    ui->simulationPanel->setStyleSheet(ui->simulationPanel->styleSheet().append(stylesheet));
}

void SimulationView::setImageSimulation(bool display)
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
void SimulationView::resizeEvent(QResizeEvent *event)
{    
    if(lbicon != nullptr) {
        QPoint point = this->rect().center();
        lbicon->setGeometry(point.x()-14, point.y()+2, 28 ,28);
    }
}
