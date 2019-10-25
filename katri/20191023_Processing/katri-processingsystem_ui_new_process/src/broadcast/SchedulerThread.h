//
// Created by thanh on 9/18/2019.
//

#ifndef PROCESSOR_SCHEDULERTHREAD_H
#define PROCESSOR_SCHEDULERTHREAD_H

#include <QObject>
#include <QThread>
#include <QVector>
#include "viewerpanel.h"
#include <QWaitCondition>
#include <QMutex>
#include <QTimer>
#include "../mediator/IColleague.h"

class IMediator;

class SchedulerThread : public QThread
{
Q_OBJECT

public:
    SchedulerThread(IMediator *pMediator);
    ~SchedulerThread();
    void run();
    QStringList makeTimestamps(QString kittiPath);
    void singleShotNextPosition();
    void setSeekPosition(int pos) {m_currentPos = pos;}

Q_SIGNALS:
    void EmitBlackScreen();
    void EmitPosition(int);

public Q_SLOTS:
    void startPlaying();
    void stop();

public:
    bool m_ready;

private:
    IMediator *m_mediator;
    int m_currentPos = 0;
    QMap<int,int> *m_timeline;
    QTimer *m_timer;
    int m_end;
};


#endif //PROCESSOR_SCHEDULERTHREAD_H
