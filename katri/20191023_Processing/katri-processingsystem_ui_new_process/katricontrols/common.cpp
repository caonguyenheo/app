#include "common.h"
#include <QDebug>

QString convertColorToString(const QColor &color)
{
    QString str;
    str = QString("rgb(") + QString::number(color.red())
        + QString(",")    + QString::number(color.green())
        + QString(",")    + QString::number(color.blue())
        + QString(")");
    return str;
}

void readFileBuffer(const char* path, QStringList &output)
{
    FILE *f = fopen(path, "rb");
    fseek(f, 0, SEEK_END);
    unsigned long long fsize = (unsigned long long)ftell(f);
    fseek(f, 0, SEEK_SET);  /* same as rewind(f); */

    char* data = (char*)malloc(fsize + 1);
    fread(data, 1, fsize, f);
    fclose(f);

    data[fsize] = 0;

    QString allData = QString::fromUtf8(const_cast<char*> (data));
    output.append(allData.split('\n'));
    if (output.last().compare("") == 0) {
        output.pop_back();
    }
}

void readMinMaxTimestamp(const char* path, QStringList &rangeList)
{
    FILE *f = fopen(path, "rb");
    size_t bufSize = 30;
    char* buffer = (char*)malloc(bufSize);
    fread(buffer, 1, bufSize, f);

    // get first timestamp in file
    QString firstTs = QString::fromUtf8(const_cast<char*> (buffer));
    firstTs.truncate(firstTs.lastIndexOf(".") + 3 + 1);

    long offset = 30;
    fseek(f, -offset, SEEK_END);
    fread(buffer, 1, bufSize, f);

    // get last timestamp in file
    QString lastTs = QString::fromUtf8(const_cast<char*> (buffer));
    lastTs.truncate(offset);
    int firstN = lastTs.indexOf("\n");
    int lastN  = lastTs.lastIndexOf("\n");
    lastTs = lastTs.mid((firstN+1),
                        (firstN==lastN) ? -1 : (lastN-firstN-1));
    lastTs.truncate(lastTs.lastIndexOf(".") + 3 + 1);

    fclose(f);
    free(buffer);

    rangeList.append(firstTs);
    rangeList.append(lastTs);
//    QStringList ret = {firstTs, lastTs};
//    return ret;

//    QDateTime offsetDate = QDateTime::fromString(offsetDateStr, DATE_FORMAT);
//    QDateTime firstDate  = QDateTime::fromString(firstTs, TIMESTAMP_FORMAT);
//    QDateTime lastDate   = QDateTime::fromString(lastTs, TIMESTAMP_FORMAT);
//    map.insert(offsetDate.msecsTo(firstDate))
}

QMap<int,int> *makeCommonTimeline(QString first, QString last)
{
    QMap<int,int> *map = new QMap<int,int>();
    QDateTime firstDate  = QDateTime::fromString(first, TIMESTAMP_FORMAT);
    QDateTime lastDate   = QDateTime::fromString(last, TIMESTAMP_FORMAT);

    qint64 length = firstDate.msecsTo(lastDate);

    int t = 0;
    int n = 0;
    while (t >= 0 && t <= length) {
        // Increment step {33,34,33}
        int d = n%3;
        switch (d) {
        case 1:
            d = 34;
            break;
        default: // case 0, case 2
            d = 33;
            break;
        }
        map->insert(t, d);
        t += d;
        n++;
    }

    return map;
}

bool isInRange(int less, int more, int current)
{
    return (less <= current && current <= more);
}
