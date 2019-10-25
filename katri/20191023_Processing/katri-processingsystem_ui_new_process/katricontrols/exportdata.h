#ifndef EXPORTDATA_H
#define EXPORTDATA_H

#include <QWidget>
#include "katricontrols_export.h"

namespace Ui {
class exportdata;
}

class KATRICONTROLS_EXPORT OptionExportData : public QWidget
{
    Q_OBJECT

public:
    explicit OptionExportData(QWidget *parent = nullptr);
    ~OptionExportData();
    void openDirConnection(const QObject *receiver, const char *member);
    void exportConnection(const QObject *receiver, const char *member);
    void cancelExportConnection(const QObject *receiver, const char *member);
    void setDirectory(QString dir);
    void setEnableLidar(bool status);
    void setEnableImage(bool status);
    void setEnableRosbag(bool status);
    QString getFolderPath();
    bool isExtraction();

public Q_SLOTS:
    void slotStateChanged(int status);

private:
    Ui::exportdata *ui;
};

#endif // EXPORTDATA_H
