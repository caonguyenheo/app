#ifndef IMEDIATOR_H_
#define IMEDIATOR_H_

#include <QString>
#include <QPointer>
#include "common.h"
#include "viewerpanel.h"
//#include "../broadcast/SchedulerThread.h"

class SchedulerThread;
class IColleague;

class IMediator : public QObject
{
Q_OBJECT
public:
    IMediator();

    virtual  ~IMediator();

    virtual void clearIColleagueList();

    void broadcast(EventType eventType, EventObject *eventObject);

    void addNewRegistered(QPointer<IColleague>, QString);

    void loadData(QString kittiPath);

    void resetSyncStatus();

    void updateSyncStatus(EventObject *eventObject);

    void setSeekPosition(EventObject *eventObject);

public Q_SLOTS:
    void receiveSignalPosition(int TimeStamp);

Q_SIGNALS:
    void signalPosition(int value);
    void signalSendTimestamp(QVector<int> *vectorTemp);

private:
    QList<QPointer<IColleague>> colleagueList;
    QMap<QString,bool> renderState;
    SchedulerThread *m_scheduler;
    QMutex m_mutex;
};


#endif //IMEDIATOR_H
