#ifndef MENUBAR_H
#define MENUBAR_H

#include <QWidget>
#include "katricontrols_export.h"
#include <QPushButton>

namespace Ui {
class MenuBar;
}

class KATRICONTROLS_EXPORT MenuBar : public QWidget
{
    Q_OBJECT

public:
    explicit MenuBar(QWidget *parent = nullptr);
    ~MenuBar();
    void exitConnection(const QObject *receiver, const char *member);
    void minimumSizeConnection(const QObject *receiver, const char *member);
    void maximumSizeConnection(const QObject *receiver, const char *member);
    QPushButton *pBtnMax = nullptr;
protected:

private:
    Ui::MenuBar *ui;
};

#endif // MENUBAR_H
