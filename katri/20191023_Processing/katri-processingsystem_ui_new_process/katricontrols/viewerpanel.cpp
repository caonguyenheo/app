#include <QDebug>
#include "viewerpanel.h"
#include "ui_viewerpanel.h"
#include <QDir>
#include <QDateTime>
#include "common.h"

#include "../src/mediator/IMediator.h"
#include "../src/mediator/IColleague.h"
#include "../src/broadcast/CVideoPlayer.h"

ViewerPanel::ViewerPanel(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::ViewerPanel) {
    ui->setupUi(this);

    // Init control panel
    control = new PlaybackControl(this);
//    control->setEnabled(false);
    ui->hControlLayout->addWidget(control);
    connect(control, SIGNAL(signalFullOrScaleScreen(int)), this, SIGNAL(signalOfPlayControl(int)));
    connect(this, SIGNAL(signalStateFullOrScale(int)), control, SLOT(slotgetIconFullOrScaleScreen(int)));

    // Left Layout
    vLayoutLeft = new QVBoxLayout();
    vLayoutLeft->setObjectName("vLayoutLeft");
    ui->hViewLayout->addLayout(vLayoutLeft);

    // Right Layout
    vLayoutRight = new QVBoxLayout();
    vLayoutRight->setObjectName("vLayoutRight");
    ui->hViewLayout->addLayout(vLayoutRight);
    // Top right Layout
    hLayoutTop = new QHBoxLayout();
    hLayoutTop->setObjectName("hLayoutTop");
    vLayoutRight->addLayout(hLayoutTop);
    // Bottom right Layout
    hLayoutBottom = new QHBoxLayout();
    hLayoutBottom->setObjectName("hLayoutBottom");
    vLayoutRight->addLayout(hLayoutBottom);

//    // Add Radar view
//    radarView = new SimulationView(this);
//    QString name("Radar View");
//    radarView->setViewName(name);

//    // Set layout and size to Radar View
//    radarView->resize(220, 432);
//    radarView->setMinimumWidth(220);
//    radarView->setMinimumHeight(432);
//    radarView->setContentsMargins(0, 0, 0, 0);
//    // radarView->setMaximumSize(220,400);

//    QColor color(86, 86, 86);
//    radarView->setViewColor(color);
//    vLayoutLeft->addWidget(radarView);
//    //vLayoutLeft->addSpacing(20);

    // Add Lidar view
    lidarGrpView = new LidarGroupView(this);
    QString name("LiDAR View");
    lidarGrpView->setViewName(name);

    // Set layout and size to Radar View
    lidarGrpView->resize(220, 432);
    lidarGrpView->setMinimumWidth(220);
    lidarGrpView->setMinimumHeight(432);
    lidarGrpView->setContentsMargins(0, 0, 0, 0);

    QColor color(86, 86, 86);
    lidarGrpView->setViewColor(color);
    vLayoutLeft->addWidget(lidarGrpView);

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
        name = "SVM View";
        svmView->setViewName(name);
        svmView->setViewColor(color);
        svmView->setSVMLayout();

        // Move to CSS code
        svmView->setMinimumWidth(265);
        svmView->setMinimumHeight(210);
        hLayoutTop->addWidget(svmView);

        // Set stretch layout
        hLayoutTop->setStretch(0, 1);
        hLayoutTop->setStretch(1, 1);
    }

    // Add bottom view
    {
        // Add GPS View
        gpsView = new MapViewer(this);
        name = "GPS View";
        QLabel* gps_label = new QLabel();
        QImage gps_image(":sample/sample/gps.png");
        gps_label->setPixmap(QPixmap::fromImage(gps_image));
        gps_label->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
        gps_label->setScaledContents(true);
        gpsView->setViewName(name);
        gpsView->setViewColor(color);
        gpsView->setMinimumWidth(170);
        gpsView->setMinimumHeight(210);
        //gpsView->setImage(gps_label);
        hLayoutBottom->addWidget(gpsView);
//        listControlScreen["gps"] = true;

//        // Add LiDAR View

//        lidarView = new SimulationLidarView(this);
//        name = "LiDAR View";
// //       QString url("qrc:/qml/qmlscatter/main.qml");
// //        lidarView->setQmlSource(url);
//        lidarView->setViewName(name);
//        lidarView->setViewColor(color);
//        lidarView->initView(3);
//        lidarView->setMinimumWidth(170);
//        lidarView->setMinimumHeight(210);
//        hLayoutBottom->addWidget(lidarView);
// //        lidarView->show();

//        listControlScreen["lidar"] = true;

        // Add LiDAR View

        radarView = new SimulationView(this);
        name = "Radar View";
        QLabel* radar_label = new QLabel();
        QImage radar_image(":sample/sample/radar.jpg");
        radar_label->setPixmap(QPixmap::fromImage(radar_image));
        radar_label->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
        radar_label->setScaledContents(true);
        radarView->setViewName(name);
        radarView->setViewColor(color);
        radarView->setMinimumWidth(170);
        radarView->setMinimumHeight(210);
        radarView->setImage(radar_label);
        hLayoutBottom->addWidget(radarView);
        //        lidarView->show();

//        listControlScreen["lidar"] = true;


        // Add CanBUS View
        canBusView = new CANBusView(this);
        name = "CAN Bus View";
        canBusView->setViewName(name);
        canBusView->setViewColor(color);
        canBusView->setMinimumWidth(170);
        canBusView->setMinimumHeight(210);
        hLayoutBottom->addWidget(canBusView);
//        listControlScreen["canbus"] = true;

        // Set stretch layout
        hLayoutBottom->setStretch(0, 1);
        hLayoutBottom->setStretch(1, 1);
        hLayoutBottom->setStretch(2, 1);
        // hLayoutBottom->addSpacing(20);
    }

    // Set stretch layout
    ui->hViewLayout->setStretch(0, 220);
    ui->hViewLayout->setStretch(1, 550);
    vLayoutRight->setStretch(0, 1);
    vLayoutRight->setStretch(1, 1);
    vLayoutLeft->setStretch(0, 1);

    // Set mainscreen layout
    setMainScreenLayout();
}

ViewerPanel::~ViewerPanel() {
    delete ui;
}

void ViewerPanel::setMainScreenLayout() {

    //radarView->setStyleSheet("border:1px solid blue;font-size:14px;");
    lidarGrpView->setStyleMainScreenLayout();
    ui->hViewLayout->setContentsMargins(0, 0, 0, 0);
    ui->hViewLayout->setSpacing(20);

    // Add top view layout
    {
        // Camera View
        cameraView->setStyleMainScreenLayout();

        // SVM View
        svmView->setStyleMainScreenLayout();
    }

    // Add bottom view
    {
//        gpsView->setStyleSheet("border:1px solid blue;font-size:14px;");
//        lidarView->setStyleSheet("border:1px solid blue;font-size:14px;");
//        canBusView->setStyleSheet("border:1px solid blue;font-size:14px;");
        // GPS View
        gpsView->setStyleMainScreenLayout();

        // LiDAR View
        radarView->setStyleMainScreenLayout();

        // CanBus View
        canBusView->setStyleMainScreenLayout();
    }
}

void ViewerPanel::setFullScreenLayout() {
    // Set layout and size to Radar View
    lidarGrpView->setStyleFullScreenLayout();
    ui->hViewLayout->setContentsMargins(56, 34, 58, 0);
    ui->hViewLayout->setSpacing(40);

    // Add top view layout
    {
        // Camera View
        cameraView->setStyleFullScreenLayout();

        // SVM View
        svmView->setStyleFullScreenLayout();
    }

    // Add bottom view
    {
        // GPS View
        gpsView->setStyleFullScreenLayout();

        // LiDAR View
        radarView->setStyleFullScreenLayout();

        // LiDAR View
        canBusView->setStyleFullScreenLayout();
    }
}

bool ViewerPanel::fakeDisplay() {
    // Fake Radar View
    radarView->appendStyleSheet("border-image: url(:/sample/sample/radar-img.jpg) 0 0 0 0 stretch stretch;");

    // Fake GPS View
    gpsView->appendStyleSheet("border-image: url(:/sample/sample/GPS-img.jpg) 0 0 0 0 stretch stretch;");

    // Fake LiDAR View
    lidarGrpView->appendStyleSheet("border-image: url(:/sample/sample/Lidar-img.jpg) 0 0 0 0 stretch stretch;");

    // Fake CAN BUS View
    canBusView->appendStyleSheet("border-image: url(:/sample/sample/CanBus-img.jpg) 0 0 0 0 stretch stretch;");

    cameraView->appendStyleSheet("border-image: url(:/sample/sample/CanBus-img.jpg) 0 0 0 0 stretch stretch;");

    svmView->appendStyleSheet("border-image: url(:/sample/sample/CanBus-img.jpg) 0 0 0 0 stretch stretch;");
    // Fake Camera View
//    for (int i = 1; i <= MAX_VIEWER_NUMBER; i++) {
//        QWidget *widget = cameraView->getView(i);
//        if (widget) {
//            widget->setStyleSheet(widget->styleSheet().append(
//                    "border-image: url(:/sample/sample/Cam" + QString::number(i) + "-img.jpg)"
//                                                                                   "0 0 0 0 stretch stretch;"));
//        }
//    }

//    // Fake SVM View
//    for (int j = 1; j <= MAX_VIEWER_NUMBER; j++) {
//        QWidget *widget = svmView->getView(j);
//        if (widget) {
//            widget->setStyleSheet(widget->styleSheet().append(
//                    "border-image: url(:/sample/sample/SVM" + QString::number(j) + "-img.jpg)"
//                                                                                   "0 0 0 0 stretch stretch;"));
//        }
//    }

    // Enable playback control
//    control->setEnabled(true);

    return true;
}

void ViewerPanel::setSpinnerDisplay(bool display) {
    canBusView->setImageSimulation(display);
    lidarGrpView->setImageSimulation(display);
    radarView->setImageSimulation(display);
    gpsView->setImageSimulation(display);
    svmView->setImageSimulation(display);
    cameraView->setImageSimulation(display);
}

//void ViewerPanel::setInterfacePage(QString name, bool data) {
//    if (name == "all") {                        //if all screen
//        for (int i = 0; i < listControlScreen.size(); i++) {
//            listControlScreen[listControlScreen.keys().at(i)] = data;
//        }
//    } else {                                    //one screen
//        for (int i = 0; i < listControlScreen.size(); i++) {
//            QString strTmp = listControlScreen.keys().at(i);
//            if (strTmp == name) {
//                listControlScreen[strTmp] = data;
//            } else {
//                listControlScreen[strTmp] = false;
//            }
//        }
//    }
//    this->showScreen(listControlScreen);
//}

void ViewerPanel::getScreen(ScreenType type) {
    bool isRadar = false, isCamera = false, isSVM = false, isGPS = false, isLiDar = false, isCanbus = false;
    switch (type)
    {
    case ScreenType::stALL:
        isCamera = true;
        isSVM = true;
        isGPS = true;
        isLiDar = true;
        isRadar = true;
        isCanbus = true;
        break;
    case ScreenType::stCAMERA:
        isCamera = true;
        break;
    case ScreenType::stSVM:
        isSVM = true;
        break;
    case ScreenType::stGPS:
        isGPS = true;
        break;
    case ScreenType::stRADAR:
        isRadar = true;
        break;
    case ScreenType::stLIDAR:
        isLiDar = true;
        break;
    case ScreenType::stCANBUS:
        isCanbus = true;
        break;
    }
//    qDebug()<<"CAM"<<isCamera;
//    qDebug()<<"SVM"<<isSVM;
//    qDebug()<<"GPS"<<isGPS;
//    qDebug()<<"Lidar"<<isLiDar;
//    qDebug()<<"RAdar"<<isRadar;
//    qDebug()<<"CANBUS"<<isCanbus;
//    qDebug()<<"----------";
    svmView->setHidden(!isSVM);
    cameraView->setHidden(!isCamera);
    gpsView->setHidden(!isGPS);
    lidarGrpView->setHidden(!isLiDar);
    radarView->setHidden(!isRadar);
    canBusView->setHidden(!isCanbus);
    if (isRadar == true && isCamera == true && isSVM == true ) {
        lidarGrpView->setMinimumHeight(432);
        ui->hViewLayout->removeItem(vLayoutLeft);
        ui->hViewLayout->removeItem(vLayoutRight);
        ui->hViewLayout->addLayout(vLayoutLeft);
        ui->hViewLayout->addLayout(vLayoutRight);
        ui->hViewLayout->setStretch(0, 220);
        ui->hViewLayout->setStretch(1, 550);

        vLayoutRight->removeItem(hLayoutTop);
        vLayoutRight->removeItem(hLayoutBottom);
        vLayoutRight->addLayout(hLayoutTop);
        vLayoutRight->addLayout(hLayoutBottom);
        lidarGrpView->setLidarVerticalLayout();
    } else if(isLiDar == true) {
        lidarGrpView->setMinimumHeight(386);
        lidarGrpView->setLidarHorizontalLayout();
        ui->hViewLayout->removeItem(vLayoutLeft);
        ui->hViewLayout->removeItem(vLayoutRight);
        ui->hViewLayout->addLayout(vLayoutLeft);
        ui->hViewLayout->addLayout(vLayoutRight);
        vLayoutRight->removeItem(hLayoutTop);
        vLayoutRight->removeItem(hLayoutBottom);
    } else if (isCamera == true || isSVM == true) {
        ui->hViewLayout->removeItem(vLayoutLeft);
        vLayoutRight->removeItem(hLayoutBottom);
        vLayoutRight->removeItem(hLayoutTop);
        vLayoutRight->addLayout(hLayoutTop);
    } else if (isGPS == true || isRadar == true || isCanbus == true) {
        ui->hViewLayout->removeItem(vLayoutLeft);
        vLayoutRight->removeItem(hLayoutTop);
        vLayoutRight->removeItem(hLayoutBottom);
        vLayoutRight->addLayout(hLayoutTop);
        vLayoutRight->addLayout(hLayoutBottom);
    }

//    set spacing for view
      setSpacing4View(isRadar, isCamera, isSVM, isGPS, isLiDar, isCanbus);
}

void ViewerPanel::setSpacing4View(bool isRadar, bool isCamera, bool isSVM, bool isGPS, bool isLiDar, bool isCanbus)
{
    if(isRadar && isCamera && isSVM && isGPS && isLiDar && isCanbus){
        cameraView->gridLayout->setContentsMargins(15, 20, 15, 11);
        cameraView->gridLayout->setVerticalSpacing(10);
        cameraView->gridLayout->setHorizontalSpacing(10);
        //svm
        svmView->hLayout->setContentsMargins(10,10,10,10);
        svmView->vLayout1->setContentsMargins(0,25,0,25);
        svmView->vLayout3->setContentsMargins(0,25,0,25);
        //ladar
//        lidarView->hLayout1->setSpacing(10);
//        lidarView->hLayout1->setContentsMargins(10,27,10,5);
//        lidarView->hLayout2->setContentsMargins(50, 5, 50, 27);
//        qDebug() << "VIEW ALL";
        setOutsideSpacing4View(3);

    }
    else if (isCamera) {
        cameraView->gridLayout->setContentsMargins(100, 20, 100, 20);
        cameraView->gridLayout->setVerticalSpacing(20);
        cameraView->gridLayout->setHorizontalSpacing(20);
//        qDebug() << "Camera";
        setOutsideSpacing4View(1);
    }
    else if (isSVM) {
        svmView->hLayout->setContentsMargins(35, 20, 35, 20);
//        svmView->hLayout->setSpacing(20);
        svmView->vLayout3->setContentsMargins(0, 56, 15, 56);
        svmView->vLayout1->setContentsMargins(15, 56, 0, 56);

        cameraView->gridLayout->setContentsMargins(100, 20, 100, 20);
        cameraView->gridLayout->setVerticalSpacing(20);
        cameraView->gridLayout->setHorizontalSpacing(20);

        setOutsideSpacing4View(1);
    }
    else if (isLiDar) {
//        lidarView->hLayout1->setSpacing(20);
//        lidarView->hLayout1->setContentsMargins(170,20,170,10);
//        lidarView->hLayout2->setContentsMargins(270, 10, 270, 20);
        qDebug() << "isLidar";
        setOutsideSpacing4View(1);
    }
    else if ((isRadar)|| (isGPS)|| (isCanbus)) {
         setOutsideSpacing4View(1);
    }

}
void ViewerPanel::setOutsideSpacing4View(int viewNum){
//    viewNum==1: choose one option, other: chk all
    if(viewNum == 1){
        ui->hViewLayout->setContentsMargins(30, 37, 30, 0);
    }
    else{
        ui->hViewLayout->setContentsMargins(0, 0, 0, 0);
    }
}
