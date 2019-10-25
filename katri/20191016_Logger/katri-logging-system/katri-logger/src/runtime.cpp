#include "runtime.h"

Runtime::Runtime()
{

}

//QString Runtime::getCurrentDate() const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    return _config.value("current_date", "").toString();
//}

//void Runtime::setCurrentDate() const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    _config.setValue("current_date", QDate::currentDate().toString("yyyy_MM_dd"));
//}

//void Runtime::setCurrentSequence(const uint &seq) const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    _config.setValue("current_seq", formatSequence(seq));
//}

QString Runtime::formatSequence(const uint &seq) const
{
    return QString::number(seq).rightJustified(4, '0', true);
}

//void Runtime::setCurrentSequence(const QString &seq) const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    _config.setValue("current_seq", seq);
//}

//QString Runtime::getCurrentSequenceOrDefault() const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    return _config.value("current_seq", "0000").toString();
//}

//QString Runtime::getNextSequence() const
//{
//    QSettings _config(runtime, QSettings::IniFormat);
//    return QString::number(_config.value("current_seq", "-1").toInt() + 1)
//        .rightJustified(4, '0', true);
//}

//QString Runtime::getCurrentKittiPathOrDefault() const
//{
//    QString path;
//    QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
//    if (getCurrentDate() == currentDate) {
//        return path.append(currentDate)
//            .append(".kat")
//            .append("/")
//            .append(currentDate)
//            .append("_drive_")
//            .append(getCurrentSequenceOrDefault());
//    } else {
//        return path.append(currentDate)
//            .append(".kat")
//            .append("/")
//            .append(currentDate)
//            .append("_drive_0000");
//    }
//}

//QString Runtime::getNextKittiPathOrDefault() const
//{
//    QString path;
//    QString currentDate = QDate::currentDate().toString("yyyy_MM_dd");
//    if (getCurrentDate() == currentDate) {
//        QString seq = getNextSequence();
//        return path.append(currentDate)
//            .append(".kat")
//            .append("/")
//            .append(currentDate)
//            .append("_drive_")
//            .append(seq);
//    } else {
//        return path.append(currentDate)
//            .append(".kat")
//            .append("/")
//            .append(currentDate)
//            .append("_drive_0000");
//    }
//}
