#include "recordbutton.h"
#include "controll.h"
#include "ui_recordbutton.h"
#include <QFontDatabase>

RecordButton::RecordButton(QWidget *parent, uint id) :
    QWidget(parent),
    ui(new Ui::RecordButton),
    m_parent(parent),
    btn_id(id)
{
    ui->setupUi(this);
//    connect(ui->btnRec, SIGNAL(clicked(bool)), m_parent, SLOT(slotRecordClicked(bool)));
}

void RecordButton::setStyleSheet()
{
    ui->label->setStyleSheet("QLabel{"
                             "background-color:transparent;"
                             "qproperty-alignment: 'AlignCenter';"
                             //"text-decoration: underline;"
                             "border : none;"
                             "font-size: 13px;"
                             "font-weight:bold;"
                             "color : white;"
                             "}");
}

void RecordButton::setLabelStyleSheet()
{
    //if (QFontDatabase::addApplicationFont(":/fonts/Roboto/Roboto-Bold.ttf") > 0)
        ui->label->setStyleSheet("QLabel{"
                             "background-color:transparent;"
                             "qproperty-alignment: 'AlignCenter';"
                             "border : none;"
                             "font: 13px 'Roboto';"
                             "font-weight:bold;"
                             "color : #ffffff;"
                             "}");

}

void RecordButton::setStyleSheet_performance_window()
{
   // this->setStyleSheet("background-color:transparent");
    ui->label->setStyleSheet("QLabel{"
                             "qproperty-alignment: 'AlignCenter';"
                             "text-decoration: underline;"
                             "border : none;"
                             "font-size: 13px;"
                             "font-weight:bold;"
                             "color : white;"
                             "background-color:transparent"
                             "}");
    ui->btnRec->setCheckable(true);
    ui->btnRec->setStyleSheet("QPushButton{background-color:transparent;background-image: url(:/img/icon/RecNonselect-btn.png); border:none;font-size:1px;}"
                              "QPushButton:checked{background-image: url(:/img/icon/RecSelected-btn.png);border:none;font-size:1px;}"
                              );
}

void RecordButton::setName(QString name)
{
    ui->label->setText(name);
}

RecordButton::~RecordButton()
{
    delete ui;
}

uint RecordButton::getID()
{
    return btn_id;
}

void RecordButton::setActive(bool active)
{
    ui->btnRec->setChecked(active);
    emit ui->btnRec->clicked(active);
}

void RecordButton::setChecked(bool active)
{
    ui->btnRec->setChecked(active);
}

bool RecordButton::getStatusButton()
{
    return ui->btnRec->isChecked();
}

QPushButton* RecordButton::getRecordButton()
{
    return ui->btnRec;
}

void RecordButton::setCenterPoint(QPoint pCenter)
{
    p_old = ui->btnRec->geometry().center();
    tl_old = this->rect().topLeft();
    QPoint tl_new = pCenter;
    tl_new.setX(pCenter.x()-(p_old.x() - tl_old.x()));
    tl_new.setY(pCenter.y()-(p_old.y() - tl_old.y()));
    this->setGeometry(tl_new.x(),tl_new.y(),this->rect().width(),this->rect().height());
}

void RecordButton::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    ControlL *parent = dynamic_cast<ControlL*>(m_parent);

    if(parent != nullptr) {
        p_old = ui->btnRec->geometry().center();
        tl_old = this->rect().topLeft();
        QPoint pCenter = m_parent->rect().center();
        QPoint tl_new = pCenter;
        tl_new.setX(pCenter.x()-(p_old.x() - tl_old.x()));
        tl_new.setY(pCenter.y()-(p_old.y() - tl_old.y()));
        this->setGeometry(tl_new.x(),tl_new.y(),this->rect().width(),this->rect().height());
    }
}

void RecordButton::setCallback(QObject * obj)
{
    Q_UNUSED(obj);
//    connect(ui->btnRec, SIGNAL(clicked(bool)), obj, SLOT(slotRecordClicked(bool)));
}

