//
// Created by thanh on 9/18/2019.
//

#include "common.h"
#include "SchedulerThread.h"
#include "../mediator/IMediator.h"
#include <QDebug>
#include <QDir>
#include <QtAlgorithms>

const QString GNSS = "gnss";
const QString CCAN = "ccan";

SchedulerThread::SchedulerThread(IMediator *pMediator)
{
    m_mediator = pMediator;
//    connect(this, SIGNAL(EmitPlay()), pMediator, SLOT(receiveSignalPlay()));
//    connect(this, SIGNAL(EmitPause()), pMediator, SLOT(receiveSignalPause()));
//    connect(this, SIGNAL(EmitPosition(int)), pMediator, SLOT(receiveSignalPosition(int)));

//    connect(pMediator, SIGNAL(signalPlay()), this, SLOT(receiveSignalPlay()));
//    connect(pMediator, SIGNAL(signalPause()), this, SLOT(receiveSignalPause()));
//    connect(pMediator, SIGNAL(signalPosition(int)), this, SLOT(receiveSignalPosition(int)));
//    connect(pMediator, SIGNAL(signalSendTimestamp(QVector<int>*)), this, SLOT(getAllTimestamp(QVector<int>*)));

    m_currentPos = 0;
    m_timeline = nullptr;
    m_ready = true;
    m_end = 0;

    m_timer = new QTimer();
    connect(m_timer, &QTimer::timeout, this, &SchedulerThread::singleShotNextPosition);
}

SchedulerThread::~SchedulerThread()
{
    this->quit();
}

void SchedulerThread::run()
{
//    while (true)
//    {
//        if (playingState == true)
//        {
//        }
//    }
}

void SchedulerThread::startPlaying()
{
    if (m_timeline) {
        if (m_currentPos >= m_end) {
            m_currentPos = 0;
        }
        m_timer->start(33);
    }
}

void SchedulerThread::stop()
{
    if (m_timeline) {
        m_timer->stop();
    }
}

QStringList SchedulerThread::makeTimestamps(QString kittiPath)
{
    QStringList tsList;
    QDirIterator it(kittiPath, QStringList() << "timestamps*.txt", QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        QString fullPath = it.next();
        tsList << fullPath.replace(kittiPath, "");
    }

    QStringList rangeList;
    Q_FOREACH(QString path, tsList) {
        // Load timestamps.txt file
        QString tsPath = kittiPath + path;
        readMinMaxTimestamp(tsPath.toStdString().c_str(), rangeList);
    }
    sort(rangeList.begin(), rangeList.end());

    // Build common timeline
    if (m_timeline) {
        m_timeline->clear();
        delete m_timeline;
    }
    m_timeline = makeCommonTimeline(rangeList.first(), rangeList.last());

    if (m_timeline->keys().count() > 0) {
        m_end = m_timeline->keys().at(m_timeline->keys().count()-1);
    }
    else {
        m_end = 0;
    }

    // Return first/last to mediator
    QStringList ret = {rangeList.first(), rangeList.last()};
    return ret;
}

void SchedulerThread::singleShotNextPosition()
{
    int current = m_currentPos;
    QMap<int,int>::iterator it = m_timeline->find(m_currentPos);
    if (it.key() < m_end) {
        // Get next interval and start new timer
        it++;
        m_currentPos = it.key();
        int interval = it.value();

        m_timer->setInterval(interval);
    }
    else {
        // Stop timer
        m_timer->stop();
        m_mediator->broadcast(EventType::EOP, nullptr);
    }

    // Broadcast current position
    static int n = 0;
    int mod = n%2; n++;
    if(mod == 0) {
        if (m_ready) {
            TimelineValue *valueObj = new TimelineValue();
            valueObj->value = current;
            m_mediator->broadcast(EventType::TIMELINE_VALUE, valueObj);
        }
        else {
            qDebug() << "Skip frame" << current;
        }
    }
}
