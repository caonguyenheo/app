#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow qWin;
    qWin.show();

        qWin.startNode(argc, argv);

    return a.exec();
}
