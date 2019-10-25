
#ifndef QARCHIVE_H
#define QARCHIVE_H

#include <QString>
#include <QStringList>

struct QArchive_internal;
class QArchive
{
public:
    QArchive();
    void setSource(QString archive_file);
    bool listFiles(QStringList & out);
    bool readFile(QString target, QByteArray & out);
    bool readFirstFile(QByteArray & out);
    static QArchive &instance();

private:
    bool openArchive(QString archive_file);
    bool readData(QByteArray & out);
    const size_t BUFFER_SIZE = 16384;

    QArchive_internal *data = nullptr;
};

#endif
