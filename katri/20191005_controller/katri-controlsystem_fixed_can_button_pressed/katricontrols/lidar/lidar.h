#ifndef LIDAR_H
#define LIDAR_H

#include <QWidget>

namespace Ui {
class LiDar;
}

class LiDar : public QWidget
{
    Q_OBJECT

public:
    explicit LiDar(QWidget *parent = nullptr);
    ~LiDar();

private:
    Ui::LiDar *ui;
};

#endif // LIDAR_H
