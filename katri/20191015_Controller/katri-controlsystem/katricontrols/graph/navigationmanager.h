#ifndef NAVIGATIONMANAGER_H
#define NAVIGATIONMANAGER_H

#include <QWidget>


namespace Ui {
class NavigationManager;
}

class NavigationManager : public QWidget
{
    Q_OBJECT

public:
    explicit NavigationManager(QWidget *parent = nullptr);
    ~NavigationManager();

public Q_SLOTS:
    void realtimeDataSlot();

private:
    Ui::NavigationManager *ui;
    void setLayoutUI(QWidget *graph);
};

#endif // NAVIGATIONMANAGER_H
