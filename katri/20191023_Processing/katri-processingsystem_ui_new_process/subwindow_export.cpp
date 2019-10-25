#include "subwindow_export.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDialogButtonBox>
#include <QtWidgets>
#include <QDebug>

#include "common.h"
#include "mainwindow.h"

subwindow_export::subwindow_export(QWidget *parent) :
    QWidget(parent)
{
    this->setLayout(new QHBoxLayout());
//    setWindowTitle( QString::fromUtf8("KATRI DATA LOGGER FOR CAR") );
//    setWindowFlags( Qt::Dialog
//                  | Qt::CustomizeWindowHint
//                  | Qt::WindowTitleHint
//                  | Qt::WindowCloseButtonHint
//                  | Qt::WindowMinimizeButtonHint
//                  | Qt::WindowMaximizeButtonHint
//                  );

    // Set background color
    //this->setMinimumSize(1280,720);
    this->setupUi();

    if (!ext) {
        ext = new KittiExtractor(this);
    }

    connect(this, SIGNAL(signalExceptionMessage()), SLOT(slotExceptionMessage()));
}

subwindow_export::~subwindow_export()
{
    if (ext) {
        delete ext;
    }
    ext = nullptr;
}

void subwindow_export::mousePressEvent(QMouseEvent *event)
{
    QWidget::mousePressEvent(event);
}


void subwindow_export::setupUi()
{
    this->widgetExport = new QWidget();
    this->widgetExport->setLayout(new QGridLayout());
    this->widgetExport->layout()->setMargin(0);
    this->widgetExport->setStyleSheet( "background-color:#272727;" );

    this->layout()->addWidget(this->widgetExport);
    this->layout()->setMargin(0);
    QGridLayout *gLayout = qobject_cast<QGridLayout*>(this->widgetExport->layout());

    this->backpnl = new BackPanel(this);
    this->exportView = new ViewDataExport(this);
    this->menuExport = new OptionExportData(this);

    gLayout->addWidget(backpnl,0,0,1,-1);
    gLayout->addWidget(exportView,1,0,1,1);
    gLayout->addWidget(menuExport,1,4,1,3);

    gLayout->setMargin(0);
    menuExport->openDirConnection(this, SLOT(slotOpenDirExportClicked()));
    menuExport->exportConnection(this,SLOT(slotExportClicked()));
    menuExport->cancelExportConnection(this,SLOT(slotCancelExportClicked()));
    exportView->selectConnection(this,SLOT(slotSelectViewClicked(bool)));
}

void subwindow_export::slotSelectViewClicked(bool sta)
{
    Q_UNUSED(sta);
    this->menuExport->setEnableLidar(exportView->lidar->getStatus());
    this->menuExport->setEnableImage(exportView->camera->getStatus()||exportView->svm->getStatus());
    this->menuExport->setEnableRosbag(exportView->canbus->getStatus());
}

void subwindow_export::backConnection(const QObject *receiver, const char *member)
{
    this->backpnl->backConnection(receiver, member);
}

void subwindow_export::cancelConnection(const QObject *receiver, const char *member)
{
    connect(this,SIGNAL(signalCancelConnection()),receiver,member);
}

void subwindow_export::refreshSubWindow()
{
    MainWindow* main = dynamic_cast<MainWindow*>(this->parent());

    // Get Min/Max range
    QString truncStart = main->getMinTimestamp();
    truncStart.truncate(TIMESTAMP_FORMAT.length());
    QDateTime startDT = QDateTime::fromString(truncStart, TIMESTAMP_FORMAT);
    this->exportView->setMinTimestamp(startDT);

    QString truncEnd = main->getMaxTimestamp();
    truncEnd.truncate(TIMESTAMP_FORMAT.length());
    QDateTime endDT = QDateTime::fromString(truncEnd, TIMESTAMP_FORMAT);
    this->exportView->setMaxTimestamp(endDT);
}

void subwindow_export::slotOpenDirExportClicked()
{
    //Create Open File dialog
    m_filePath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                  "",
                                                  QFileDialog::ShowDirsOnly
                                                  | QFileDialog::DontResolveSymlinks);
    menuExport->setDirectory(m_filePath);
}

void subwindow_export::slotCancelExportClicked()
{
    QDialog dialog;
    QVBoxLayout layout(&dialog);
    QLabel *label = new QLabel();
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Yes | QDialogButtonBox::No);
    connect(buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));

    QPushButton *yesBtn = buttonBox->button(QDialogButtonBox::Yes);
    yesBtn->setAutoDefault(true);
    yesBtn->setDefault(true);

    QPushButton *noBtn = buttonBox->button(QDialogButtonBox::No);
    noBtn->setAutoDefault(false);
    noBtn->setDefault(false);

    dialog.setWindowTitle("KATRI Data Logger for Car");
    dialog.setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint | Qt:: WindowTitleHint);
    label->setTextFormat(Qt::RichText);
    label->setText("<img src=:/icon/icon/notification/Question-icon.svg align=top> Do you want to discard the changes?");

    //set Style
//    dialog.setStyleSheet("background-color:#ffffff");
    label->setStyleSheet("color:#272727; font:14px 'Roboto'; font-weight:normal; margin-left:9px;margin-top:7px;");
    yesBtn->setStyleSheet("margin-top:22px;margin-bottom:11px;background-color:#e5f1fb; color:#272727; font:12px 'Roboto'; border:1px solid #0078d7;height:18px; min-width:73px");
    noBtn->setStyleSheet("margin-top:22px;margin-bottom:11px;background-color:#e1e1e1; color:#272727; font:12px 'Roboto'; border:1px solid #a7a7a7;height:18px; min-width:73px");
    layout.addWidget(label);
    layout.addWidget(buttonBox);

    buttonBox->setCenterButtons(true);
    dialog.setLayout(&layout);
    if(dialog.exec() == QDialog::Accepted)
    {
        refreshSubWindow();
        qDebug()<<"yes";
    }
}

void subwindow_export::slotExportClicked()
{
    if (menuExport->isExtraction()) {
//        QPushButton *exportBtn = dynamic_cast<QPushButton*>(this->sender());
//        exportBtn->setEnabled(false);

        // Extracting processing on sub thread
        QThread *m_thread = QThread::create(std::bind(&subwindow_export::singleShotExtraction, this));
        m_thread->start();
    }
    else {
        // Exporting processing
    }
}

void subwindow_export::singleShotExtraction()
{
    // Extracting processing
    if(menuExport->getFolderPath().compare("") != 0) {
        MainWindow* main = dynamic_cast<MainWindow*>(this->parent());
        QString KittiPath = main->getDataPath();
        KittiPath.replace('\\', '/');
        QStringList folderList = KittiPath.split('/');
        if (folderList.count() >= 2) {
            QString kittiName =
                    folderList[folderList.count() - 2] + "." +
                    QString(QDateTime::currentDateTime().toString(TIMESTAMP_FORMAT))
                        .replace(" ", "_").replace("-", "").replace(":", "")
                    + "/" + folderList[folderList.count() - 1];

            QString start = this->exportView->getStartTimestamp();
            QString end = this->exportView->getEndTimestamp();
            ext->setBasedKittiPath(KittiPath);
            ext->setExtractionKittiPath(menuExport->getFolderPath() + "/" + kittiName);
            ext->execExtracting(start, end);

            emit this->exportView->signalMessageSuccess(5000);
        }
        else {
            qDebug() << "Wrong Kitti file name" << KittiPath;
            emit this->signalExceptionMessage();
        }
    }
    else {
        emit this->signalExceptionMessage();
    }
}

void subwindow_export::slotExceptionMessage()
{
    QMessageBox *msg = new QMessageBox(this);
    msg->setWindowTitle("KATRI Data Logger for Car");
    msg->setIconPixmap(QPixmap(":/icon/icon/notification/Error-icon.svg"));
    msg->setText("Failed to export. Please try again.");
    msg->setStandardButtons(QMessageBox::Close);
//        msg.setWindowFlags( Qt::Dialog
//                          | Qt::CustomizeWindowHint
//                          | Qt::WindowTitleHint
//                          | Qt::WindowCloseButtonHint
//                          );
    msg->exec();
    msg->deleteLater();
}
