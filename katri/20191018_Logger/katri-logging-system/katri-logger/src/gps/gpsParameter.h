#ifndef GPSPARAMETER_H
#define GPSPARAMETER_H
#include "QString"
#include "../appConfig.h"

class GpsParameter
{
public:
    GpsParameter();
    GpsParameter(const QString &id, const QString &path);
    GpsParameter &operator=(const GpsParameter &rhs);
    QString getPath() const;
    void setPath(const QString &path);

    void setId(const QString &id);
    QString getId() const;

    QString getDataPath() const;
    QString getDataFilePath() const;
    QString getTimestampPath() const;
    QString getDataformatPath() const;

private:
    QString _id;
    QString _path;
};

#endif // GPSPARAMETER_H
