#ifndef CANPARAMETER_H
#define CANPARAMETER_H

#include "QString"
#include "../appConfig.h"

class CanParameter
{
public:
    CanParameter();
    CanParameter(const QString &id, const QString &src, const QString &path);
    ~CanParameter();
    CanParameter & operator=(const CanParameter & rhs);

    QString getSrc() const;
    void setSrc(const QString &src);

    QString getPath() const;
    void setPath(const QString &path);

    QString getProgram() const;

    void setId(const QString &id);
    QString getId() const;

    void setFileName(const QString &);

    QString getDataPath() const;
    QString getFilePath() const;
    QString getTimestampPath() const;
    QString getCanDirectoryPath() const;

private:
    QString _src;
    QString _path;
    const QString _program = "./katricandump";
    QString _id;
    QString _fileName;
};

#endif // CANPARAMETER_H
