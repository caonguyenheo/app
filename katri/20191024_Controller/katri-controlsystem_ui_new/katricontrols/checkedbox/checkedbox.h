#ifndef CHECKEDBOX_H
#define CHECKEDBOX_H

#include <QWidget>
#include <common/common.h>

class checkedbox : public QWidget
{
    Q_OBJECT
public:
    explicit checkedbox(QWidget *parent = nullptr);
    void getcheckedbox(CheckboxType type);
signals:

public slots:
};

#endif // CHECKEDBOX_H
