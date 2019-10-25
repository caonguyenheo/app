#include "navigationpanel.h"
#include "ui_navigationpanel.h"
#include <QFontDatabase>
#include <QGraphicsOpacityEffect>

NavigationPanel::NavigationPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NavigationPanel)
{
    ui->setupUi(this);

    // Set mask to button
    QPainterPath path;
    path.addRoundedRect(ui->pb_load_to->rect(), 5, 5);
    ui->pb_load_to->setMask(QRegion(path.toFillPolygon().toPolygon()));

    QPainterPath path1;
    path1.addRoundedRect(ui->pb_browse->rect(), 5, 5);
    ui->pb_browse->setMask(QRegion(path1.toFillPolygon().toPolygon()));

    QPainterPath path2;
    path2.addRoundedRect(ui->pb_export->rect(), 5, 5);
    ui->pb_export->setMask(QRegion(path2.toFillPolygon().toPolygon()));

    //Apply Style
    QFontDatabase::addApplicationFont(":/fonts/Roboto/Roboto-Medium.ttf");
    this->setStyleSheet("font:14px 'Roboto';");
//    setStyleNavCheckbox(ui->cb_all);
//    setStyleNavCheckbox(ui->cb_camera);
//    setStyleNavCheckbox(ui->cb_svm);
//    setStyleNavCheckbox(ui->cb_gps);
//    setStyleNavCheckbox(ui->cb_radar);
//    setStyleNavCheckbox(ui->cb_lidar);
//    setStyleNavCheckbox(ui->cb_can);
    setStyleNavRadioButton(ui->rd_all);
    setStyleNavRadioButton(ui->rd_camera);
    setStyleNavRadioButton(ui->rd_svm);
    setStyleNavRadioButton(ui->rd_gps);
    setStyleNavRadioButton(ui->rd_radar);
    setStyleNavRadioButton(ui->rd_lidar);
    setStyleNavRadioButton(ui->rd_can);
    setStyleNavRadioButton(ui->rd_calib);

    enableExportButton(false);
    ui->pb_export->setStyleSheet("color: #939393; background-color: #196382; border-radius: 10px;font: 12px 'Roboto';font-weight:bold;");

    connect(this, SIGNAL(signalEnableAllButton(bool)), SLOT(slotEnableAllButton(bool)));
}
void NavigationPanel::setStyleNavCheckbox(QCheckBox *chk){
    chk->setStyleSheet("QCheckBox{color: white; font: 14px 'Roboto'; font-weight:500}"
                       "QCheckBox::indicator {width: 19px; height: 19px; border-radius:5px;}"
                       "QCheckBox::indicator::checked {background-image: url(:/img/icon/icon-blue.png);}"
                       "QCheckBox::indicator::unchecked {background-image: url(:/img/icon/icon-gray.png);}"
                        );

}
void NavigationPanel::setStyleNavRadioButton(QRadioButton *rad){
    rad->setStyleSheet("QRadioButton{color: white; font: 14px 'Roboto'; font-weight:500;}"
                       "QRadioButton::indicator {width: 18px; height: 18px; border-radius:5px;}"
                       "QRadioButton::indicator::checked {background-image: url(:/img/icon/radio-check.png);}"
                       "QRadioButton::indicator::unchecked {background-image: url(:/img/icon/radio-uncheck.png);}"
                        );

}
NavigationPanel::~NavigationPanel()
{
    delete ui;
}

void NavigationPanel::loadToConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_load_to, SIGNAL(released()), receiver, member);
}

void NavigationPanel::browseConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_browse, SIGNAL(released()), receiver, member);
}

void NavigationPanel::exportConnection(const QObject *receiver, const char *member)
{
    connect(ui->pb_export, SIGNAL(released()), receiver, member);
}

void NavigationPanel::enableLoadToButton(bool status)
{
    ui->pb_load_to->setEnabled(status);

    if(!status) {
        QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->pb_load_to);
        dbEffect->setOpacity(0.5);
        ui->pb_load_to->setGraphicsEffect(dbEffect);
    }
    else {
        ui->pb_load_to->setGraphicsEffect(nullptr);
    }
}

void NavigationPanel::enableBrowseButton(bool status)
{
    ui->pb_browse->setEnabled(status);

    if(!status) {
        QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->pb_browse);
        dbEffect->setOpacity(0.5);
        ui->pb_browse->setGraphicsEffect(dbEffect);
    }
    else {
        ui->pb_browse->setGraphicsEffect(nullptr);
    }
}

void NavigationPanel::enableExportButton(bool status)
{
    ui->pb_export->setEnabled(status);
//    ui->pb_export->setStyleSheet("color: #ffffff; background-color: #0c9fdc; border-radius: 10px;font: 12px 'Roboto';font-weight:bold;");

    if(!status) {
        //QGraphicsOpacityEffect* dbEffect = new QGraphicsOpacityEffect(ui->pb_export);
        //dbEffect->setOpacity(0.5);
       // ui->pb_export->setGraphicsEffect(dbEffect);
        ui->pb_export->setStyleSheet("color: #ffffff; background-color: #0c9fdc; border-radius: 10px;font: 12px 'Roboto';font-weight:bold;");

        ui->pb_export->setStyleSheet("QPushButton{"
                                        "color: rgb(255, 255, 255);"
                                        "background-color:#0c9fdc;"
                                        "border-radius: 10px;"
                                        "font: 12px 'Roboto';"
                                        "font-weight:bold;}"
                                        );
    }
    else {
       // ui->pb_export->setGraphicsEffect(nullptr);
        ui->pb_export->setStyleSheet("QPushButton{"
                                        "color: rgb(255, 255, 255);"
                                        "background-color:#0c9fdc;"
                                        "border-radius: 10px;"
                                        "font: 12px 'Roboto';"
                                        "font-weight:bold;}"


                                        "QPushButton::hover{"
                                        "color: #fff;"
                                        "background-color: #14a6e3;"
                                        "border-radius: 10px;"
                                        "font: 12px 'Roboto';"
                                        "font-weight:bold;}"
                                        );
    }
}

void NavigationPanel::clickedCheckBox(QCheckBox *cbSender, const QObject *receiver, const char *member)
{
    connect(cbSender, SIGNAL(released()), receiver, member);
}
QList<QRadioButton*> NavigationPanel::getListObjectCalib()
{
    QList <QRadioButton*>listCl;
    listCl.append(ui->rd_calib);
    return listCl;
}
void NavigationPanel::clickedRadioButton(QRadioButton *rdSender, const QObject *receiver, const char *member)
{
    connect(rdSender, SIGNAL(released()), receiver, member);
}
QList<QRadioButton*> NavigationPanel::getListObjectRadio()
{
    QList <QRadioButton*>listRd;
    listRd.append(ui->rd_all);
    listRd.append(ui->rd_can);
    listRd.append(ui->rd_gps);
    listRd.append(ui->rd_svm);
//    listCb.append(ui->cb_calib);
    listRd.append(ui->rd_lidar);
    listRd.append(ui->rd_radar);
    listRd.append(ui->rd_camera);
    return listRd;
}

static bool tmpLoadToStt,
            tmpBrowseStt,
            tmpExportStt;
void NavigationPanel::slotEnableAllButton(bool status)
{
    if(!status) { // status = False
        tmpLoadToStt = ui->pb_load_to->isEnabled();
        tmpBrowseStt = ui->pb_browse->isEnabled();
        tmpExportStt = ui->pb_export->isEnabled();

        enableLoadToButton(status);
        enableBrowseButton(status);
        enableExportButton(status);
    }
    else { // status = True
        enableLoadToButton(tmpLoadToStt);
        enableBrowseButton(tmpBrowseStt);
        enableExportButton(tmpExportStt);
    }
}
