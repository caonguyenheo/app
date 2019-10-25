#include "controll.h"
#include "ui_controll.h"
#include "common/common.h"
#include <QSpacerItem>
#include <QDebug>
#include <QFontDatabase>

ControlL::ControlL(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlL)
{
    ui->setupUi(this);
    this->setViewColor();
    gLayout = new QGridLayout(ui->widgetButton);
//    ui->gridLayout->setColumnStretch(0,1);
//    ui->gridLayout->setColumnStretch(1,8);
//    ui->gridLayout->setColumnStretch(2,1);
    ui->gridLayout->setRowStretch(0,1);
    ui->gridLayout->setRowStretch(1,8);
    ui->gridLayout->setRowStretch(2,1);
    this->setStyleSheet("border:1px solid red");
    ui->lblTimer->setStyleSheet("color:white; border:0; font-size:12px;");
    connect(this,SIGNAL(signalUpdateStatus()),parent,SLOT(slotUpdateStatus()));
    m_isAllChecked = false;
    ui->lblLidarlive->setHidden(true);
}

void ControlL::setName(QString name)
{
    ui->lblname->setText(name);
}

void ControlL::setType(Command::Type type)
{
    m_Type = type;
}

Command::Type ControlL::getType()
{
    return m_Type;
}

void ControlL::setViewChartAction(QString name, const QObject *receiver, const char *member)
{
    ui->labelView->setText(name);
    ui->labelView->setFont(QFont(":/fonts/Roboto/Roboto-Bold.ttf"));
    ui->labelView->setStyleSheet("QLabel{"
                                 "qproperty-alignment: 'AlignRight';"
                                 "text-decoration: underline;"
                                 "border : none;"
                                 "color : #ffffff;"
                                 "font: 12px 'Roboto';"
                                 "background-color: none;"
                                 "}");
    ui->labelView->installEventFilter(this);

    connect(this, SIGNAL(viewChartRelease()), receiver, member);
}

void ControlL::setViewCameraAction(QString name, const QObject *receiver, const char *member)
{
    ui->labelView->setText(name);
    ui->labelView->installEventFilter(this);
    connect(this, SIGNAL(viewCameraRelease()), receiver, member);
}

void ControlL::setLidarLiveStreamAction(QString name, const QObject *receiver, const char *member)
{
    ui->lblLidarlive->setText(name);
    ui->lblLidarlive->setFont(QFont(":/fonts/Roboto/Roboto-Bold.ttf"));
    ui->lblLidarlive->setStyleSheet("QLabel{"
                                 "qproperty-alignment: 'AlignRight';"
                                 "text-decoration: underline;"
                                 "border : none;"
                                 "color : #ffffff;"
                                 "font: 12px 'Roboto';"
                                 "background-color: none;"
                                 "}");
    ui->lblLidarlive->installEventFilter(this);

    connect(this, SIGNAL(viewLidarLiveStreamRelease()), receiver, member);
}

void ControlL::setViewColor()
{
    this->setStyleSheet("border:1px solid red");
    ui->controllwiget->setStyleSheet("QWidget{"
                         "border:2px solid #a6a6a6;"
                         "background-color: qlineargradient(x1: 0, y0: 1 x2: 1, y2: 0, stop: 0 #3a388b,stop: 1 #0d8a6f);"
                         "}"
                        );

    ui->lblname->setStyleSheet("QLabel{"
                            // "border-style:rectangle;"
                             "border:0;"
                             "font-size: 13px;"
                             "padding-left: 8px;"
                             "padding-right: 8px;"
                             "background-color: #ffffff;"
                             "font: 13px 'Roboto';"
                             "font-weight:bold;"
                             "color:#272727;"
                             "margin-top:2px;"
                             "margin-left:2px;"
                             "qproperty-alignment: 'AlignCenter';"
                             "}");



    ui->labelView->setStyleSheet("QLabel{"
                                 "qproperty-alignment: 'AlignRight';"
                                 "text-decoration: underline;"
                                 "border : none;"
                                 "color : white;"
                                 "font: 12px 'Roboto';"
                                 "font-weight:normal;"
                                 "background-color: none;"
                                 "}");
    ui->widgetButton->setStyleSheet("border:0;background-color:none;");
}

void ControlL::updateStyle(bool status)
{
    if(status==true){
        this->setStyleSheet("border:1px solid blue");
        ui->controllwiget->setStyleSheet("QWidget{"
                            "border:2px solid #ec883c;"
                            "background-color: qlineargradient(x1: 0, y0: 1 x2: 1, y2: 0, stop: 0 #3a388b,stop: 1 #0d8a6f);"
                            "}"
                           );
    }
    else{
        this->setStyleSheet("QWidget{border:2px solid red;");

        ui->controllwiget->setStyleSheet("QWidget{"
                            "border:2px solid #a6a6a6;"
                            "background-color: qlineargradient(x1: 0, y0: 1 x2: 1, y2: 0, stop: 0 #3a388b,stop: 1 #0d8a6f);"
                            "}"
                            );
    }
}


void ControlL::AddRecordButton(QStringList names)
{
    for(int i = 0; i < names.size(); i++)
    {
        RecordButton *rec = new RecordButton(ui->widgetButton, i);
        rec->setName(names.at(i));
        rec->setLabelStyleSheet();
        int rowIdx = 0;
        int colIdx = i;
        if(i > 3){
            rowIdx = 1;
            colIdx -= 4;
            if (names.size() == 7) {
                colIdx = i;
                colIdx -= 3;
            }
        }
        else if (names.size() == 7 && i == 3) {
            rowIdx = 1;
            colIdx -= 3;
        }
        if(names.size() == 7 && i < 3) {
            ui->gridLayoutButton->addWidget(rec,rowIdx,colIdx,1,2,Qt::AlignCenter);
        }
        else {
            ui->gridLayoutButton->addWidget(rec,rowIdx,colIdx,1,1);
        }
        listRecordButton.push_back(rec);
    }
}

ControlL::~ControlL()
{
    delete ui;
}

void ControlL::activeAllButton(bool active)
{
    for(int i = 0; i < listRecordButton.size(); ++i)
    {
        listRecordButton.at(i)->setActive(active);
    }
}

bool ControlL::eventFilter(QObject *obj, QEvent *event)
{
    if (qobject_cast<QLabel*>(obj)==ui->labelView && event->type() == QEvent::MouseButtonRelease)
    {
        emit this->viewChartRelease();
        emit this->viewCameraRelease();
        return true;
    }
    else if (qobject_cast<QLabel*>(obj)==ui->lblLidarlive && event->type() == QEvent::MouseButtonRelease)
    {
        emit this->viewLidarLiveStreamRelease();
        return true;
    }
    return false;
}

void ControlL::slotRecordClicked(bool status)
{
    return;
    //check status all button
    bool isAllChecked = true;

    if(listRecordButton.size() == 3)
    {
        for(int idx = 0; idx < listRecordButton.size(); idx++)
        {
            RecordButton *btn = listRecordButton.at(idx);
            if(btn != nullptr){
                if(idx == 0)
                    isAllChecked = btn->getStatusButton();
                else
                    isAllChecked |= btn->getStatusButton();
            }
        }
    }

    if((listRecordButton.size() == 1 && (status ^ m_isAllChecked)) || (listRecordButton.size() == 3 && (isAllChecked ^ m_isAllChecked))){
        m_isAllChecked = status;
        emit signalChangeStatus(status);
    }
}
#if 0
void ControlL::setupButton()
{
    if(listRecordButton.size() == 1){
        for(int i = 0; i < listRecordButton.size(); ++i)
        {
            listRecordButton.at(i)->setCenterPoint(this->rect().center());
        }
    }
    if(listRecordButton.size() == 3){
        for(int i = 0; i < listRecordButton.size(); ++i)
        {
            QPoint r = this->rect().center();
            if(i == 0){
                r.setX(r.x() - 50);
                r.setY(r.y() - 50);
            }else if (i == 1) {
                r.setX(r.x() + 50);
                r.setY(r.y() - 50);
            }else if (i == 2) {
                r.setY(r.y() + 50);
            }
            listRecordButton.at(i)->setCenterPoint(r);
        }
    }
}

void ControlL::resizeEvent(QResizeEvent *event)
{
    return;
    if(listRecordButton.size() == 1){
        for(int i = 0; i < listRecordButton.size(); ++i)
        {
            listRecordButton.at(i)->setCenterPoint(this->rect().center());
        }
    }
    if(listRecordButton.size() == 3){
        for(int i = 0; i < listRecordButton.size(); ++i)
        {
            QPoint r = this->rect().center();
            if(i == 0){
                r.setX(r.x() - 50);
                r.setY(r.y() - 50);
            }
            else if (i == 1) {
                r.setY(r.y() + 50);
            }
            else if (i == 2) {
                r.setX(r.x() + 50);
                r.setY(r.y() - 50);
            }

            listRecordButton.at(i)->setCenterPoint(r);
            listRecordButton.at(i)->show();
        }
    }
}
#endif
bool ControlL::getStatus()
{
    return m_isAllChecked;
}

void ControlL::setStatus(bool sta)
{
//    m_isAllChecked = sta;
    for(int idx = 0; idx < listRecordButton.size(); idx++)
    {
        RecordButton *btn = listRecordButton.at(idx);
        if(btn != nullptr){
            btn->setActive(sta);
        }
    }
}

void ControlL::setStatus(uint id, bool sta)
{
    for(int idx = 0; idx < listRecordButton.size(); idx++)
    {
//        qDebug() << listRecordButton.at(idx)->getID() << id;
        if(listRecordButton.at(idx)->getID() == id)
        {
            RecordButton *btn = listRecordButton.at(idx);
            if(btn != nullptr){
                btn->setChecked(sta);
                this->updateStyle(sta);
            }
            break;
        }
    }
}

bool ControlL::isRecording()
{
    bool ret = false;
    Q_FOREACH(RecordButton* btn, listRecordButton) {
        if (btn->getStatusButton()) {
            ret = true;
        }
    }
    return ret;
}

void ControlL::AddNameLidarView(QStringList names)
{
    for(int i = 0; i < names.size(); i++)
    {
        switch (names.size()) {
        case GPS:
            ui->lblLidarlive->setHidden(true);
            break;
        case LIDAR:
            ui->lblLidarlive->setHidden(false);
            break;
        case CANBUS:
            ui->lblLidarlive->setHidden(true);
            break;
        case CAMERA:
            ui->lblLidarlive->setHidden(true);
            break;
        default:
            break;
        }
    }
}
