#ifndef NAVIGATIONMANAGERSMALL_H
#define NAVIGATIONMANAGERSMALL_H

#include <QWidget>

namespace Ui {
    class NavigationManagerSmall;
}

class NavigationManagerSmall : public QWidget
{
    Q_OBJECT

public:
    explicit NavigationManagerSmall(QWidget *parent = nullptr);
    ~NavigationManagerSmall();

    void Init_cameraview();
    void initDefaultValue();
    void loadValue();
public Q_SLOTS:
    void realtimeDataSlot();

private:
    Ui::NavigationManagerSmall *ui;
};

#endif // NAVIGATIONMANAGERSMALL_H
