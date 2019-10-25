#ifndef VIEWCOMMAND_H
#define VIEWCOMMAND_H

#include <QWidget>
#include "katricontrols_export.h"

namespace Ui {
class viewcommand;
}

class KATRICONTROLS_EXPORT viewcommand : public QWidget
{
    Q_OBJECT

public:
    explicit viewcommand(QWidget *parent = nullptr);
    ~viewcommand();

private:
    Ui::viewcommand *ui;
};

#endif // VIEWCOMMAND_H
