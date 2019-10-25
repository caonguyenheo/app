#include "common.h"

QString convertColorToString(const QColor &color)
{
    QString str;
    str = QString("rgb(") + QString::number(color.red())
        + QString(",")    + QString::number(color.green())
        + QString(",")    + QString::number(color.blue())
        + QString(")");
    return str;
}
