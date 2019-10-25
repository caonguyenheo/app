#ifndef SOURCE_H
#define SOURCE_H

#include "QObject"
#include "QString"

class Source : public QObject
{
    Q_OBJECT
public:
    explicit Source(QObject *parent = nullptr);
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool isRunning() = 0;
    virtual QString getId() = 0;
    virtual QString getName() = 0;
    virtual QString getStatus() = 0;

    virtual void setPath(const QString &) = 0;
};

#endif // SOURCE_H
