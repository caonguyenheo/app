#ifndef RECORD_H
#define RECORD_H

#include <QWidget>
#include <QCheckBox>

namespace Ui {
class record;
}

class record : public QWidget
{
    Q_OBJECT

public:
    explicit record(QWidget *parent = nullptr);
    ~record();

    void InitUi();
    QList<QCheckBox *> getListCheckbox();
    void clickcheckbox(QCheckBox *cb_box, const QObject *receiver, const char *member);
private:
    QString style;
    Ui::record *ui;
};

#endif // RECORD_H
