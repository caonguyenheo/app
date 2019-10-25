#ifndef VIEWERPERFORMANCE_H
#define VIEWERPERFORMANCE_H

#include <QWidget>
#include "recordbutton.h"
#include <QGridLayout>
#include "katricontrols_export.h"

namespace Ui {
class viewerperformance;
}

class KATRICONTROLS_EXPORT viewerperformance : public QWidget
{
    Q_OBJECT

public:
    explicit viewerperformance(QWidget *parent = nullptr, int id = 0);
    ~viewerperformance() override;
    void initvalue();
    void setIP(QString name);
    void setname(QString name);
    void setDeviceId(int id);
    int getDeviceId();
    QWidget *getVideoObject();
    void addwidgetright(QWidget * widget);
    void addwidgetleft(QWidget * widget);
    void setActivate(bool status);
    bool isActivated();
    void addbuttonrecord();
    void sendCommandConnection(const QObject *receiver, const char *member);
    RecordButton* getRecordButton(){return rec;}

    void updateStyle(bool status);
    void setChecked(bool status);
Q_SIGNALS:
    void signalChangeStatus(bool recording, int device_id);

public Q_SLOTS:
    void slotRecordClicked(bool status);

private:
    Ui::viewerperformance *ui;
    QWidget *widgetVideo = nullptr;
    RecordButton *rec = nullptr;
    int device_id;

protected:
    void resizeEvent(QResizeEvent *event) override;

};

#endif // VIEWERPERFORMANCE_H
