#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#define SCHEDULER

#include <QWidget>
#include <QTimer>
#include <QList>
#include "navigationpanel.h"
#include "viewerpanel.h"
#include "subwindow_export.h"
#include "src/extractor/imageextraction.h"
#include "src/mediator/IMediator.h"
#include "src/broadcast/SchedulerThread.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QWidget {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

    MainWindow &operator=(const MainWindow &);

    void setupUi();

    QString getDataPath() { return m_filePath; }

    QString getMinTimestamp();

    QString getMaxTimestamp();

    void singleShotLoadTo(QString loadToPath);

    void registerColleagues(IMediator *mediator);


public Q_SLOTS:

    void slotLoadToBtnClicked();

    void slotBrowseBtnClicked();

    void slotExportBtnClicked();

    void slotBackBtnClicked();

    void slotFakeDisplay();

    void slotFullScreen(int state);

    void slothideMessageBrowse();

    void slotClickedOfPage();

Q_SIGNALS:

    void signalScaleToViewPannel(int);

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QString m_filePath = "";
    NavigationPanel *m_navigation = nullptr;
    subwindow_export *m_subwindow = nullptr;
    ImageExtraction *m_imgExt = nullptr;

    QLabel *lblicon = nullptr;
    QLabel *lblmessage = nullptr;
    IMediator *m_mediator;

public:
    ViewerPanel *m_viewer = nullptr;

};

#endif // MAINWINDOW_H
