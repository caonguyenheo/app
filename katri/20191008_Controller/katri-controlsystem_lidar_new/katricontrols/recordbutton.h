#ifndef RECORDBUTTON_H
#define RECORDBUTTON_H

#include <QWidget>
#include <QPushButton>
#include "katricontrols_export.h"
#include <QPushButton>

namespace Ui {
class RecordButton;
}

class KATRICONTROLS_EXPORT RecordButton : public QWidget
{
    Q_OBJECT

public:
    explicit RecordButton(QWidget *parent = nullptr, uint id = 0);
    ~RecordButton() override;

    void setName(QString name);
    void setStyleSheet();
    void setLabelStyleSheet();
    void setStyleSheet_performance_window();
    void setActive(bool active);
    void setCenterPoint(QPoint pCenter);
    bool getStatusButton();
    QPushButton* getRecordButton();

    void setCallback(QObject *obj);
    uint getID();
    void setChecked(bool active);
protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::RecordButton *ui;
    QWidget *m_parent = nullptr;
    QPoint p_old;
    QPoint tl_old;
    uint btn_id;
};

#endif // RECORDBUTTON_H
