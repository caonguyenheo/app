//#define SCHEDULER

#include "mainwindow.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDialogButtonBox>
#include <QDebug>
#include "ui_mainwindow.h"
#include <QRect>
#include <QProcess>
#include <QThread>
#include <QDirIterator>
#include <QPointer>
#include "common.h"
#include "src/common/appConfig.h"
#include "src/mediator/IMediator.h"
#include "src/broadcast/CVideoPlayer.h"
#include "src/broadcast/CLidarSimulator.h"
#include "src/broadcast/CGpsSimulator.h"
#include "src/broadcast/CCanSimulator.h"
#include "src/broadcast/CMediaControl.h"
#include "src/broadcast/SchedulerThread.h"
#include "src/common/qarchive.h"
#include <QImage>

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
    setWindowFlags( Qt::Dialog
                  | Qt::CustomizeWindowHint
                  | Qt::WindowTitleHint
                  | Qt::WindowCloseButtonHint
                  | Qt::WindowMinimizeButtonHint
                  | Qt::WindowMaximizeButtonHint
                  );
    this->setupUi();

    lblicon = new QLabel(this);
    lblmessage = new QLabel(this);

    lblicon->setStyleSheet("border : none;border-image: url(:/icon/icon/notification/Success-icon.svg)0 0 0 0 stretch stretch;");
    lblmessage->setText("Successfully load the newest data.");
    lblmessage->setStyleSheet("border:none;color : green;");

    QPoint point = this->ui->centralwidget->geometry().topLeft();
    lblicon->setGeometry(point.x()+250, point.y()+8, 18 , 18);
    lblicon->setMaximumSize(QSize(18,18));
    lblmessage->setGeometry(point.x()+270, point.y()+8, 230, 18);
    lblmessage->setMaximumSize(QSize(230,18));

    lblicon->hide();
    lblmessage->hide();

    if (!m_imgExt) {
        m_imgExt = new ImageExtraction(m_navigation);
    }

    m_mediator = new IMediator();
    registerColleagues(m_mediator);
}

MainWindow::~MainWindow()
{
    if (lblicon) {
        delete lblicon;
    }
    if (lblmessage) {
        delete lblmessage;
    }
    if (m_subwindow) {
        delete m_subwindow;
    }
    if (m_imgExt) {
        delete m_imgExt;
    }

    lblicon      = nullptr;
    lblmessage   = nullptr;
    m_subwindow  = nullptr;
    m_navigation = nullptr;
    m_viewer     = nullptr;
    m_imgExt     = nullptr;

    delete ui;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QWidget::mousePressEvent(event);
}

void MainWindow::setupUi()
{
    m_navigation = ui->navigationPanel;
    m_viewer = ui->viewerPanel;
    m_subwindow = new subwindow_export(this);
    ui->verticalLayout->addWidget(m_subwindow);
    slotBackBtnClicked();

    //layout->addWidget(m_navigation);
    m_navigation->loadToConnection(this, SLOT(slotLoadToBtnClicked()));
    m_navigation->browseConnection(this, SLOT(slotBrowseBtnClicked()));
    m_navigation->exportConnection(this, SLOT(slotExportBtnClicked()));
    m_subwindow->backConnection(this, SLOT(slotBackBtnClicked()));
    m_subwindow->cancelConnection(this,SLOT(slotBackBtnClicked()));
    QList<QRadioButton*> list = m_navigation->getListObjectRadio();
    Q_FOREACH(QRadioButton *checkBox, list) {
        m_navigation->clickedRadioButton(checkBox, this, SLOT(slotClickedOfPage()));
    }
   // connect(m_viewer, SIGNAL(signalOfPlayControl(int)), this, SLOT(slotFullScreen(int)));
    //connect(this, SIGNAL(signalScaleToViewPannel(int)), m_viewer, SIGNAL(signalStateFullOrScale(int)));
}

void MainWindow::slotFakeDisplay()
{
    lblicon->show();
    lblmessage->show();
    QTimer::singleShot(5000, this, SLOT(slothideMessageBrowse()));

    m_navigation->enableExportButton(true);
}

void MainWindow::slothideMessageBrowse()
{
    lblicon->hide();
    lblmessage->hide();
}

void MainWindow::slotLoadToBtnClicked()
{
    //Create Open File dialog
    QString loadToPath =
            QFileDialog::getExistingDirectory(this, tr("LOAD TO"),
                                              "",
                                              QFileDialog::ShowDirsOnly |
                                              QFileDialog::DontResolveSymlinks);
    if (loadToPath.compare("")==0 ||
        QDir(loadToPath).exists()==false) return;

    m_navigation->slotEnableAllButton(false);

    // Start new Thread
    QThread *m_thread = QThread::create(std::bind(&MainWindow::singleShotLoadTo, this, loadToPath));
    m_thread->start();
}

void MainWindow::singleShotLoadTo(QString loadToPath)
{
    // Load configuration
    QString loadToConfig = AppConfig::instance().getLoadToDate();
    QString nas_path = AppConfig::instance().getNasDataPath();

    // Filter NAS file based on load_to_date setting
    QStringList kittiFileFilters;
    if ( loadToConfig.compare(LOAD_TO_ALL) == 0 ) {
        kittiFileFilters << QString("*_*.tar");
    }
    else if (QDate::fromString(loadToConfig, LOAD_TO_DATE_FORMAT).isValid()) {
        kittiFileFilters << QString("*_%1.tar").arg(loadToConfig);
    }
    else {
        kittiFileFilters << QString("*_%1.tar")
                            .arg(QDate::currentDate().toString(LOAD_TO_DATE_FORMAT));
    }
    qDebug() << "kittiFileFilters:" << kittiFileFilters;

    QStringList localKittiFileList;
    QStringList kittiFileList = QDir(nas_path).entryList(kittiFileFilters, QDir::Files);
    qDebug() << "Kitti File list:" << kittiFileList;
    Q_FOREACH(QString kittiFileName, kittiFileList) {
        QString nasKittiFilePath = nas_path + "/" + kittiFileName;
        QString destKittiFilePath = loadToPath + "/" + kittiFileName;
        qDebug() << "Kitti File:" << nasKittiFilePath;

        if (QFile(nasKittiFilePath).exists()) {
            // Remove existed the same filename
            QFile destFile(destKittiFilePath);
            if(destFile.exists()) {
                destFile.remove();
            }

            // Copy data from NAS to local
            if (QFile::copy(nasKittiFilePath, destKittiFilePath)) {
                localKittiFileList.append(destKittiFilePath);
                qDebug() << "Copy success:" << nasKittiFilePath << "=>" << destKittiFilePath;
            }
            else {
                qDebug() << "Copy Error:" << nasKittiFilePath << "=>" << destKittiFilePath;
            }
        }
    }

    // Extracting data file
    Q_FOREACH(QString localFile, localKittiFileList) {
        // Create QProcess to extract tar file
        bool status;
        QProcess tarExtract;
        tarExtract.setProcessChannelMode(QProcess::MergedChannels);
        QString program = "7z.exe";
        QStringList arguments = {"x", "-y", "-o" + loadToPath, localFile};
        tarExtract.start(program, arguments);
        status = tarExtract.waitForStarted();
        if (!status)
        {
            qDebug("waitForStarted() error: %s", tarExtract.errorString().toUtf8().constData());
            continue;
        }

        status = tarExtract.waitForFinished(PROCESS_TIMEOUT);
        if (status) {
            qDebug() << "Extract successfully:" << localFile;
        }
        else {
            qDebug("waitForFinished() error: %s", tarExtract.errorString().toUtf8().constData());
            continue;
        }
        QFile(localFile).remove();
    }

    // Scan all raw data file in Kitti folder
    QStringList rawPathList;
    QStringList kittiDirFilters = {"*.kat"};
    QStringList kittiDirList = QDir(loadToPath).entryList(kittiDirFilters, QDir::Dirs);
    Q_FOREACH(QString folder, kittiDirList) {
        QString kittiFolderPath = loadToPath + "/" + folder;
        qDebug() << kittiFolderPath;

        QDirIterator it(kittiFolderPath, QStringList() << "*.raw", QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext())
            rawPathList << it.next();
    }

    // Parse raw video to png frame
    if (m_imgExt) {
        m_imgExt->setPath(rawPathList);
        emit m_imgExt->signalParseFirstVideo();
    }
    //emit m_navigation->signalEnableAllButton(true);
}

void MainWindow::slotBrowseBtnClicked()
{
    //Create Open File dialog
    QString path = QFileDialog::getExistingDirectory(this, QString::fromUtf8("Upload"));
                                              //, "/", QString::fromUtf8("KATRI File (*.kat)"));
    if (path.compare("") == 0) return;

    //Check entry point
    if(QDir(path).dirName().contains("_drive_")) {
        qDebug() << "slotBrowseBtnClicked dirname  " + QDir(path).dirName();
        m_filePath = path;
//        m_viewer->setSpinnerDisplay(true); // Move to mediator
//        if (result) {
            slotFakeDisplay();
//        }
//        m_viewer->setSpinnerDisplay(false); // Move to mediator

        m_mediator->loadData(m_filePath);

//        QString filename = "C:/Users/TRAILER/Music/2019_09_27.kat.tar";
//        QArchive::instance().setSource(filename);

//        QTime curent;
//        curent = QTime::currentTime();
//        QByteArray arr;
//        QArchive::instance().readFile("2019_09_27.kat/2019_09_27_drive_0000/image_0/video_RGBA.raw", arr);
//        qDebug() << "PhucBui1:" << curent.msecsTo(QTime::currentTime()) << "ms";


//        curent = QTime::currentTime();
//        QFile CurrentFile("C:/Users/TRAILER/Music/video_RGBA.raw");
//        if(!CurrentFile.open(QIODevice::ReadOnly)) return;
//        QByteArray DataFile = CurrentFile.readAll();
//        CurrentFile.close();
//        qDebug() << "PhucBui2:" << curent.msecsTo(QTime::currentTime()) << "ms";

    }
    else {
        // Wrong data path
        qDebug() << "slotBrowseBtnClicked wrong dirname  " + QDir(path).dirName();
        QDialog dlg;
         QVBoxLayout la(&dlg);
         QLabel *label = new QLabel();
         QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
 //        QDialogButtonBox *buttonBox = new QDialogButtonBox();
         connect(buttonBox, SIGNAL(accepted()), &dlg, SLOT(accept()));
         connect(buttonBox, SIGNAL(rejected()), &dlg, SLOT(reject()));

         dlg.setWindowTitle("KATRI Data Logger for Car");
         dlg.setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint | Qt:: WindowTitleHint);
         label->setTextFormat(Qt::RichText);
         label->setText("<img src=:/icon/icon/notification/Error-icon.svg align=top>  Failed to load data. Please try again." );
         //set Style
 //        dlg.setStyleSheet("background-color:#ffffff");
         label->setStyleSheet("color:#272727; font:14px 'Roboto'; font-weight:normal; margin-left:9px;margin-top:7px;");
         buttonBox->setStyleSheet("margin-top:22px;margin-bottom:11px;background-color:#e5f1fb; color:#272727; font:12px 'Roboto'; border:1px solid #0078d7;height:18px; min-width:73px");

         la.addWidget(label);
         la.addWidget(buttonBox);

         buttonBox->setCenterButtons(true);
 //        buttonBox->addButton("Ok", QDialogButtonBox::RejectRole);
 //        buttonBox->button(QDialogButtonBox::Ok);
         dlg.setLayout(&la);
         if(dlg.exec() == QDialog::Accepted)
         {
         }
    }
}

void MainWindow::slotExportBtnClicked()
{
    //hide icon and message browse
    slothideMessageBrowse();

    ui->centralwidget->setVisible(false);
    this->setMinimumSize(905, 585);
    m_subwindow->setVisible(true);
    m_subwindow->refreshSubWindow();
}

void MainWindow::slotBackBtnClicked()
{
    m_subwindow->setVisible(false);
    this->setMinimumSize(1110, 610);
    ui->centralwidget->setVisible(true);
}

void MainWindow::slotFullScreen(int state)
{
    if(state == 0) {
        state = 1;
        qDebug()<<"[Click] Full Screen";
        ui->horizontalLayout_2->removeWidget(ui->gridWidget_2);
        ui->gridWidget_2->setHidden(true);
        ui->gridLayout_3->setContentsMargins(0, 0, 0, 0);
        ui->gridLayout_3->setVerticalSpacing(0);
        ui->gridLayout_3->setHorizontalSpacing(0);
        ui->widget->setStyleSheet("border:1px solid #f0f0f0;background:#272727");
        emit signalScaleToViewPannel(state);
        ui->viewerPanel->setFullScreenLayout();
        this->showFullScreen();
    }
    else {
        state = 0;
        qDebug()<<"[Click] Scale Screen";
        ui->horizontalLayout_2->removeWidget(ui->gridWidget);
        ui->horizontalLayout_2->addWidget(ui->gridWidget_2);
        ui->horizontalLayout_2->addWidget(ui->gridWidget);
        ui->gridWidget_2->setHidden(false);
        ui->gridLayout_3->setContentsMargins(30,30, 30, 50);
        ui->gridLayout_3->setVerticalSpacing(10);
        ui->gridLayout_3->setHorizontalSpacing(6);
        ui->widget->setStyleSheet("border:5px solid #d2d2d6;background:#272727");
        emit signalScaleToViewPannel(state);
        ui->viewerPanel->setMainScreenLayout();
        this->showNormal();
        this->setGeometry(this->geometry().x(), this->geometry().y(), this->minimumWidth(), this->minimumHeight());
        qDebug() << this->minimumWidth() << this->minimumHeight();
    }
}
void MainWindow::slotClickedOfPage()
{
    QRadioButton *rdSender = dynamic_cast<QRadioButton*>(this->sender());
    if(rdSender) {
        if(rdSender->objectName().toLower().trimmed().contains("rd_all")){
//              m_viewer->setInterfacePage("all", rdSender->isChecked());
            m_viewer->getScreen(stALL);
        }
        if(rdSender->objectName().toLower().trimmed().contains("rd_camera")){
//            m_viewer->setInterfacePage("camera", rdSender->isChecked());
             m_viewer->getScreen(stCAMERA);
        }
        else if(rdSender->objectName().toLower().trimmed().contains("rd_svm")){
//            m_viewer->setInterfacePage("svm", rdSender->isChecked());
             m_viewer->getScreen(stSVM);
        }
        else if(rdSender->objectName().toLower().trimmed().contains("rd_gps")){
//            m_viewer->setInterfacePage("gps", rdSender->isChecked());
             m_viewer->getScreen(stGPS);
        }
        else if(rdSender->objectName().toLower().trimmed().contains("rd_lidar")){
//            m_viewer->setInterfacePage("lidar", rdSender->isChecked());
             m_viewer->getScreen(stLIDAR);
        }
        else if(rdSender->objectName().toLower().trimmed().contains("rd_can")){
//            m_viewer->setInterfacePage("canbus", rdSender->isChecked());
             m_viewer->getScreen(stCANBUS);
        }
        else if(rdSender->objectName().toLower().trimmed().contains("rd_radar")){
//            m_viewer->setInterfacePage("radar", rdSender->isChecked());
             m_viewer->getScreen(stRADAR);
        }
    }
}

QString MainWindow::getMinTimestamp()
{
    QString ret;
    if (m_viewer) {
        ret = "2019-09-12 13:00:23.900000";//m_viewer->getStartPosition();
        qDebug() << ret;
    }
    return ret;
    //return "2019-09-12 13:00:23.900000";
}

QString MainWindow::getMaxTimestamp()
{
    QString ret;
    if (m_viewer) {
        ret = "2019-09-12 13:02:09.022000";//m_viewer->getEndPosition();
        qDebug() << ret;
    }
    return ret;
    //return "2019-09-12 13:02:09.022000";
}

void MainWindow::registerColleagues(IMediator *mediator)
{
    // Create all camera colleagues
    // Camera colleague 0
    CVideoPlayer *camera0 = new CVideoPlayer(mediator, 0);
    GLWidgetImage *cameraView0 = this->m_viewer->cameraView->m_viewList[0];
    connect(camera0, SIGNAL(signalPosition(int,QString)), cameraView0, SLOT(slotReceiveTimestamp(int,QString)));
    connect(cameraView0, SIGNAL(signalFinishedRender()), camera0, SLOT(slotFinishedRender()));

    // Camera colleague 1
    CVideoPlayer *camera1 = new CVideoPlayer(mediator, 1);
    GLWidgetImage *cameraView1 = this->m_viewer->cameraView->m_viewList[1];
    connect(camera1, SIGNAL(signalPosition(int,QString)), cameraView1, SLOT(slotReceiveTimestamp(int,QString)));
    connect(cameraView1, SIGNAL(signalFinishedRender()), camera1, SLOT(slotFinishedRender()));

    // Camera colleague 2
    CVideoPlayer *camera2 = new CVideoPlayer(mediator, 2);
    GLWidgetImage *cameraView2 = this->m_viewer->cameraView->m_viewList[2];
    connect(camera2, SIGNAL(signalPosition(int,QString)), cameraView2, SLOT(slotReceiveTimestamp(int,QString)));
    connect(cameraView2, SIGNAL(signalFinishedRender()), camera2, SLOT(slotFinishedRender()));

    // Camera colleague 3
    CVideoPlayer *camera3 = new CVideoPlayer(mediator, 3);
    GLWidgetImage *cameraView3 = this->m_viewer->cameraView->m_viewList[3];
    connect(camera3, SIGNAL(signalPosition(int,QString)), cameraView3, SLOT(slotReceiveTimestamp(int,QString)));
    connect(cameraView3, SIGNAL(signalFinishedRender()), camera3, SLOT(slotFinishedRender()));

    // Create all svm colleagues
    // SVM colleague 0
    CVideoPlayer *svm0 = new CVideoPlayer(mediator, 4);
    GLWidgetImage *svmView0 = this->m_viewer->svmView->m_viewList[0];
    connect(svm0, SIGNAL(signalPosition(int,QString)), svmView0, SLOT(slotReceiveTimestamp(int,QString)));
    connect(svmView0, SIGNAL(signalFinishedRender()), svm0, SLOT(slotFinishedRender()));

    // SVM colleague 1
    CVideoPlayer *svm1 = new CVideoPlayer(mediator, 5);
    GLWidgetImage *svmView1 = this->m_viewer->svmView->m_viewList[1];
    connect(svm1, SIGNAL(signalPosition(int,QString)), svmView1, SLOT(slotReceiveTimestamp(int,QString)));
    connect(svmView1, SIGNAL(signalFinishedRender()), svm1, SLOT(slotFinishedRender()));

    // SVM colleague 2
    CVideoPlayer *svm2 = new CVideoPlayer(mediator, 6);
    GLWidgetImage *svmView2 = this->m_viewer->svmView->m_viewList[2];
    connect(svm2, SIGNAL(signalPosition(int,QString)), svmView2, SLOT(slotReceiveTimestamp(int,QString)));
    connect(svmView2, SIGNAL(signalFinishedRender()), svm2, SLOT(slotFinishedRender()));

    // SVM colleague 3
    CVideoPlayer *svm3 = new CVideoPlayer(mediator, 7);
    GLWidgetImage *svmView3 = this->m_viewer->svmView->m_viewList[3];
    connect(svm3, SIGNAL(signalPosition(int,QString)), svmView3, SLOT(slotReceiveTimestamp(int,QString)));
    connect(svmView3, SIGNAL(signalFinishedRender()), svm3, SLOT(slotFinishedRender()));

    // Create all lidar colleagues
    // Velodyne colleague 0
    CLidarSimulator *lidar0 = new CLidarSimulator(mediator, 0);
    LidarView *lidarView0 = this->m_viewer->lidarGrpView->m_viewList[0];
    connect(lidar0, SIGNAL(signalPosition(int,QString)), lidarView0, SLOT(slotReceiveTimestamp(int,QString)));
    connect(lidarView0, SIGNAL(signalFinishedRender()), lidar0, SLOT(slotFinishedRender()));

    // Velodyne colleague 1
    CLidarSimulator *lidar1 = new CLidarSimulator(mediator, 1);
    LidarView *lidarView1 = this->m_viewer->lidarGrpView->m_viewList[1];
    connect(lidar1, SIGNAL(signalPosition(int,QString)), lidarView1, SLOT(slotReceiveTimestamp(int,QString)));
    connect(lidarView1, SIGNAL(signalFinishedRender()), lidar1, SLOT(slotFinishedRender()));

    // Ouster colleague 2
    CLidarSimulator *lidar2 = new CLidarSimulator(mediator, 2);
    LidarView *lidarView2 = this->m_viewer->lidarGrpView->m_viewList[2];
    connect(lidar2, SIGNAL(signalPosition(int,QString)), lidarView2, SLOT(slotReceiveTimestamp(int,QString)));
    connect(lidarView2, SIGNAL(signalFinishedRender()), lidar2, SLOT(slotFinishedRender()));

    // Create gnss colleagues
    // GNSS colleague 0
    CGpsSimulator *gnss0 = new CGpsSimulator(mediator, 0);
    MapViewer *gnssView0 = this->m_viewer->gpsView;
    connect(gnss0, SIGNAL(signalPosition(int,QString)), gnssView0, SLOT(slotReceiveTimestamp(int,QString)));
    connect(gnssView0, SIGNAL(signalFinishedRender()), gnss0, SLOT(slotFinishedRender()));

    // Create ccan colleagues
    // CCAN colleague 0
    CCanSimulator *ccan0 = new CCanSimulator(mediator, 0);
    CANBusView *ccanView0 = this->m_viewer->canBusView;
    connect(ccan0, SIGNAL(signalPosition(int,QString)), ccanView0, SLOT(slotReceiveTimestamp(int,QString)));
    connect(ccanView0, SIGNAL(signalFinishedRender()), ccan0, SLOT(slotFinishedRender()));

    // Create media controller colleagues
    CMediaControl *mediaControl = new CMediaControl(mediator);
    m_viewer->control->seekingBarConnection(mediaControl, SLOT(slotSliderValueChanged(int)));
    m_viewer->control->controlConnection(mediaControl, SLOT(slotStateChanged(bool)));
    connect(mediaControl, SIGNAL(signalSyncSeekbarRange(int,int)), m_viewer->control, SLOT(slotSyncSeekbarRange(int,int)));
    connect(mediaControl, SIGNAL(signalPosition(int,int)), m_viewer->control, SLOT(slotReceivePosition(int,int)));
    connect(mediaControl, SIGNAL(signalEOP(bool)), m_viewer->control, SLOT(slotReceiveEOP(bool)));
}
