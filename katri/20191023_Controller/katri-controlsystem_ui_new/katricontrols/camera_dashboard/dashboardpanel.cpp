#include "dashboardpanel.h"

#include <QDir>
#include <QDateTime>
#include <QDebug>
#include "ui_dashboardpanel.h"

DashboardPanel::DashboardPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DashboardPanel)
{
    ui->setupUi(this);

    // Top right Layout
    hLayoutTop = new QHBoxLayout();
    hLayoutTop->setObjectName("hLayoutTop");
    ui->hViewLayout->addLayout(hLayoutTop);

    QString name;
    QColor color(86, 86, 86);

    // Add top view
    {
        // Add Camera View
        cameraView = new ImageView(this);
        //cameraView->getView(0);
        name = "Camera View";
        cameraView->setViewName(name);
        cameraView->setViewColor(color);
        cameraView->setCameraLayout();

        // Move to CSS code
        cameraView->setMinimumWidth(265);
        cameraView->setMinimumHeight(210);
        cameraView->setContentsMargins(0, 0, 0, 0);
        hLayoutTop->addWidget(cameraView);



        // Add SVM View
        svmView = new ImageView(this);
        svmView->setHidden(true);
        name = "SVM View";
        svmView->setViewName(name);
        svmView->setViewColor(color);
        svmView->setSVMLayout();

        // Move to CSS code
        svmView->setMinimumWidth(265);
        svmView->setMinimumHeight(210);
        //hLayoutTop->addWidget(svmView);

        // Set stretch layout
        hLayoutTop->setStretch(0, 1);
        hLayoutTop->setStretch(1, 1);
    }

    // Set stretch layout
    ui->hViewLayout->setStretch(0, 220);
    ui->hViewLayout->setStretch(1, 550);

    // Set mainscreen layout
    setMainScreenLayout();
}

DashboardPanel::~DashboardPanel() {
    delete ui;
}

void DashboardPanel::setMainScreenLayout()
{
    ui->hViewLayout->setContentsMargins(0, 0, 0, 0);
    ui->hViewLayout->setSpacing(20);

    // Add top view layout
    {
        // Camera View
        cameraView->setStyleMainScreenLayout();

        // SVM View
        svmView->setStyleMainScreenLayout();
    }
}

void DashboardPanel::setFullScreenLayout()
{
    ui->hViewLayout->setContentsMargins(56, 34, 58, 0);
    ui->hViewLayout->setSpacing(40);

    // Add top view layout
    {
        // Camera View
        cameraView->setStyleFullScreenLayout();

        // SVM View
        svmView->setStyleFullScreenLayout();
    }
}

void DashboardPanel::backButtonConnection(const QObject *receiver, const char *member)
{
    connect(ui->Backpanel->backButton(), SIGNAL(released()), receiver, member);
}

void DashboardPanel::setSpinnerDisplay(bool display)
{
    svmView->setImageSimulation(display);
    cameraView->setImageSimulation(display);
}

/**
 * @brief DashboardPanel::setDataViewHD
 * @param deviceType
 * @param deviceId
 * @param data
 * content set data image camera
 */
void DashboardPanel::setDataViewHD(int deviceId, QByteArray &data)
{
    svmView->setDataCamera(deviceId, data);
}

/**
 * @brief DashboardPanel::setDataViewFullHD
 * @param deviceType
 * @param deviceId
 * @param data
 * content set data image camera
 */
void DashboardPanel::setDataViewFullHD(int deviceId, QByteArray &data)
{
    cameraView->setDataCamera(deviceId, data);
}
