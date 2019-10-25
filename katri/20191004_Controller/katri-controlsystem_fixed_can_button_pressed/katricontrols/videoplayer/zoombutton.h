#ifndef ZOOMBUTTON_H
#define ZOOMBUTTON_H

#include <QWidget>
#include <QPushButton>
#include "katricontrols_export.h"

namespace Ui {
class zoombutton;
}

class KATRICONTROLS_EXPORT zoombutton : public QWidget
{
    Q_OBJECT

public:
    explicit zoombutton(QWidget *parent = nullptr);
    ~zoombutton();  
    void fullScreenConnection(const QObject *receiver, const char *member);
    QPushButton *fullscreen = nullptr;

private:
    Ui::zoombutton *ui;
};

#endif // ZOOMBUTTON_H
