#include "IColleague.h"
#include "IMediator.h"
#include <QString>
#include <QPointer>
#include <QDebug>
#include "iostream"
#include "../src/broadcast/SchedulerThread.h"
#include <QThread>

using namespace std;

IMediator::IMediator()
{
    m_scheduler = new SchedulerThread(this);
//    m_scheduler->start();
}

IMediator::~IMediator()
{
}

void IMediator::clearIColleagueList()
{
    colleagueList.clear();
    renderState.clear();
    if (colleagueList.size() != 0) {
        qDebug() << "ERROR: Fail to clear IColleague List";
    }
}

void IMediator::broadcast(EventType eventType, EventObject *eventObject)
{
    switch (eventType) {
    case EventType::PLAY:            // Forward PLAYING state to scheduler
        m_scheduler->m_ready = true;
        m_scheduler->startPlaying();
        break;
    case EventType::PAUSE:           // Forward PAUSED state to scheduler
        m_scheduler->stop();
        break;
    case EventType::RENDER_STATE:
        updateSyncStatus(eventObject);
        break;
    case EventType::TIMELINE_SEEK:   // Set seek position to scheduler
        setSeekPosition(eventObject);
        break;
    case EventType::TIMELINE_VALUE:
        resetSyncStatus();
    default:                         // Broadcast message to colleagues
        for (int i = 0; i < colleagueList.size(); i++)
            colleagueList.at(i)->receiveMessage(eventType, eventObject);
        break;
    }
}

void IMediator::addNewRegistered(QPointer<IColleague> registered, QString id)
{
    // Create list of resource status
    if (id.compare("") != 0) {
        renderState.insert(id, false);
    }

    colleagueList.push_back(registered);
    qDebug() << "addNewRegistered:" << colleagueList.count();
}

void IMediator::receiveSignalPosition(int TimeStamp)
{
//    EventObject updateTimeStamp;
//    updateTimeStamp.TimeStamp = TimeStamp;
//    this->broadcast(EventType::NEW_TIMESTAMP, updateTimeStamp);
}

void IMediator::loadData(QString kittiPath)
{
    // Build common timestamps
    QStringList rangeList = m_scheduler->makeTimestamps(kittiPath);

    // Broadcast start/end time
    TimelineObject *timelineObj = new TimelineObject();
    timelineObj->start    = rangeList.first();
    timelineObj->end      = rangeList.last();
    timelineObj->dataPath = kittiPath;
    broadcast(EventType::NEW_TIMELINE, timelineObj);
}

void IMediator::resetSyncStatus()
{
    m_mutex.lock();
//    qDebug() << "STATE::DONE: Reset all";
    m_scheduler->m_ready = false;
    Q_FOREACH(QString key, renderState.keys()) {
        renderState[key] = false;
    }
    m_mutex.unlock();
}

void IMediator::updateSyncStatus(EventObject *eventObject)
{
    m_mutex.lock();
    bool ret = true;
    StateEvent *event = static_cast<StateEvent*>(eventObject);
    if (event) {
//        qDebug() << "STATE::DONE:" << event->source_id;

        // Set status
        renderState[event->source_id] = event->status;

        // Check all done and reset next frame status
        Q_FOREACH(bool value, renderState.values()) {
            if (!value) {
                ret = value;
                break;
            }
        }
        if (ret) {
            m_scheduler->m_ready = true;
        }
    }
    m_mutex.unlock();
}

void IMediator::setSeekPosition(EventObject *eventObject)
{
    TimelineValue *event = static_cast<TimelineValue*>(eventObject);
    if (event) {
        m_scheduler->setSeekPosition(event->value);
        qDebug() << "Seek to" << event->value;
    }
}
