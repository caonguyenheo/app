#ifndef EXPORTDATA_LEFT_H
#define EXPORTDATA_LEFT_H

#include <QWidget>
#include "katricontrols_export.h"

namespace Ui {
class exportdata_left;
}

class KATRICONTROLS_EXPORT exportdata_left : public QWidget
{
    Q_OBJECT

public:

    explicit exportdata_left(QWidget *parent = nullptr);
    ~exportdata_left();
     void setView_exportname(QString name);
     void setView_exportimage(QString path);

     void selectConnection(const QObject *receiver, const char *member);
     bool getStatus();
private:
    Ui::exportdata_left *ui;
};

#endif // EXPORTDATA_LEFT_H
