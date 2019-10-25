#ifndef SUBWINDOW_EXPORT_H
#define SUBWINDOW_EXPORT_H

#include <QMainWindow>
#include "navigationpanel.h"
#include "viewerpanel.h"
#include "backpanel.h"
#include "viewdataexport.h"
#include "exportdata.h"
#include "src/extractor/kittiextractor.h"

class subwindow_export : public QWidget
{
    Q_OBJECT

public:
    explicit subwindow_export(QWidget *parent = nullptr);
    ~subwindow_export();
    void setupUi();
    void backConnection(const QObject *receiver, const char *member);
    void browserConnection(const QObject *receiver, const char *member);
    void cancelConnection(const QObject *receiver, const char *member);
    void refreshSubWindow();
    void singleShotExtraction();

signals:
    void signalCancelConnection();
    void signalExceptionMessage();

public slots:
    void slotOpenDirExportClicked();
    void slotExportClicked();
    void slotCancelExportClicked();
    void slotSelectViewClicked(bool sta);
    void slotExceptionMessage();

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;

private:
    QString m_filePath = "";
    QWidget *widgetExport = nullptr;
    BackPanel *backpnl = nullptr;
    ViewDataExport *exportView = nullptr;
    OptionExportData *menuExport = nullptr;

    KittiExtractor *ext = nullptr;
};

#endif // SUBWINDOW_EXPORT_H
