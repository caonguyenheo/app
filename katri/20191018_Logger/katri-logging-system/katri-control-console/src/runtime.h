#ifndef RUNTIME_H
#define RUNTIME_H

#include "QDate"
#include "QSettings"
#include "QString"

const QString runtime = "runtime.properties";
const uint numberOfx = 4;

class Runtime
{
public:
    Runtime();
    QString getCurrentDate() const;
    void setCurrentDate() const;

    void setCurrentSequence(const uint &) const;
    QString getNextSequence() const;
    QString getCurrentSequence() const;
};

#endif // RUNTIME_H
