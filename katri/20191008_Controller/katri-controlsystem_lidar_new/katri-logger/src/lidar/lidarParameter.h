#ifndef LIDARPARAMETER_H
#define LIDARPARAMETER_H
#include "QString"
#include "../appConfig.h"

class LidarParameter
{
public:
    LidarParameter();
    LidarParameter(const QString &id, const QString &path);
    LidarParameter &operator=(const LidarParameter &rhs);
    QString getPath() const;
    void setPath(const QString &path);

    void setId(const QString &id);
    QString getId() const;

    QString getDataPath() const;
    QString getTimestampPath() const;

private:
    QString _id;
    QString _path;
};

#endif // LIDARPARAMETER_H
