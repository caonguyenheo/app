#include "runtime.h"

Runtime::Runtime()
{

}

QString Runtime::getCurrentDate() const
{
    QSettings _config(runtime, QSettings::IniFormat);
    return _config.value("current_date", "").toString();
}

void Runtime::setCurrentDate() const
{
    QSettings _config(runtime, QSettings::IniFormat);
    _config.setValue("current_date", QDate::currentDate().toString("yyyy_MM_dd"));
}

void Runtime::setCurrentSequence(const uint &seq) const
{
    QSettings _config(runtime, QSettings::IniFormat);
    _config.setValue("current_seq", QString::number(seq).rightJustified(numberOfx, '0', true));
}

QString Runtime::getNextSequence() const
{
    QSettings _config(runtime, QSettings::IniFormat);
    return QString::number(_config.value("current_seq", "-1").toInt() + 1)
        .rightJustified(4, '0', true);
}

QString Runtime::getCurrentSequence() const
{
    QSettings _config(runtime, QSettings::IniFormat);
    return _config
        .value("current_seq", QString::fromStdString("0").rightJustified(numberOfx, '0', true))
        .toString();
}
