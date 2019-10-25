#ifndef CONTROLL_H
#define CONTROLL_H

#include <QWidget>
#include "katricontrols_export.h"
#include "recordbutton.h"
#include <QGridLayout>
#include "../katrilogger/command.h"

namespace Ui {
class ControlL;
}

class KATRICONTROLS_EXPORT ControlL : public QWidget
{
    Q_OBJECT

public:
    explicit ControlL(QWidget *parent = nullptr);
    ~ControlL();

    void setName(QString name);
    void setViewColor();
    void setViewChartAction(QString name, const QObject *receiver = 0, const char *member = 0);
    void setViewCameraAction(QString name, const QObject *receiver = 0, const char *member = 0);
    void AddRecordButton(QStringList name);
    void activeAllButton(bool active);
    bool getStatus();
    void setStatus(bool status);
    bool isRecording();

    bool eventFilter(QObject *obj, QEvent *event) override;
    void setupButton();
//    void resizeEvent(QResizeEvent *event) override;
    QList<RecordButton* > getListRecordButton(){return listRecordButton;}

    void updateStyle(bool status);
    void setType(Command::Type t);
    Command::Type getType();
    void setStatus(uint id, bool sta);
Q_SIGNALS:
    void viewChartRelease();
    void viewCameraRelease();
    void signalUpdateStatus();
    void signalChangeStatus(bool);

public Q_SLOTS:
    void slotRecordClicked(bool status);

private:
    Ui::ControlL *ui;
//    QGridLayout *gLayout;
    QVBoxLayout *vLayout = new QVBoxLayout();
    QHBoxLayout *hLayout1 = new QHBoxLayout();
    QHBoxLayout *hLayout2 = new QHBoxLayout();
    bool m_isAllChecked;
    Command::Type m_Type;
    QList<RecordButton*> listRecordButton;
};

#endif // CONTROLL_H
