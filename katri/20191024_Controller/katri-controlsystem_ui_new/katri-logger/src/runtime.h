#ifndef RUNTIME_H
#define RUNTIME_H

#include "QDate"
#include "QSettings"
#include "QString"

const QString runtime = "runtime.properties";

class Runtime
{
public:
    Runtime();
    //QString getCurrentDate() const;
    void setCurrentDate() const;

    //QString getNextSequence() const;
    void setCurrentSequence(const uint &) const;
    QString formatSequence(const uint &) const;
    //QString getCurrentSequenceOrDefault() const;

    //QString getCurrentKittiPathOrDefault() const;
    //QString getNextKittiPathOrDefault() const;
};

#endif // RUNTIME_H
