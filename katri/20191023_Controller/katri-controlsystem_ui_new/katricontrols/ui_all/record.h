#ifndef RECORD_H
#define RECORD_H

#include <QWidget>

namespace Ui {
class record;
}

class record : public QWidget
{
    Q_OBJECT

public:
    explicit record(QWidget *parent = nullptr);
    ~record();

private:
    QString style;
    Ui::record *ui;
};

#endif // RECORD_H
