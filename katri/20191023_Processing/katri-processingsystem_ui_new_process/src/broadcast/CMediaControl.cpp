#include "CMediaControl.h"
#include <QSlider>
#include <QDebug>

const QString CONTROL_PREFIX = "control_0";

CMediaControl::CMediaControl(IMediator *mediator)
{
    p_mediator = mediator;
    p_mediator->addNewRegistered(this, "");
    localTimeline = nullptr;
    m_end = 0;
}

void CMediaControl::receiveMessage(EventType eventType, EventObject *eventObject)
{
    switch (eventType) {
    case EventType::NEW_TIMELINE:
        // build local timeline
        loadTimestamp(eventObject);
        break;
    case EventType::TIMELINE_VALUE:
        // load frame
        loadFrameData(eventObject);
        break;
    case EventType::PLAY:
        // Video playing
        break;
    case EventType::PAUSE:
        // Video paused
        break;
    case EventType::EOP:
        // End of play
        receiveEOP();
        break;
    default:
        break;
    }
}

void CMediaControl::sendMessage(EventType eventType, EventObject *eventObject)
{
    p_mediator->broadcast(eventType, eventObject);
}

QString CMediaControl::getSourcePrefix(int id)
{
    Q_UNUSED(id);
    return CONTROL_PREFIX;
}

QString CMediaControl::getBasePath(QString kittiPath)
{
    return kittiPath;
}

void CMediaControl::loadTimestamp(EventObject *eventObject)
{
    TimelineObject *timeline = static_cast<TimelineObject*>(eventObject);
    if (timeline) {
        // Build local timeline
        if (localTimeline) {
            localTimeline->clear();
            delete localTimeline;
        }
        localTimeline = buildLocalTimeline(timeline->start, timeline->end);

        QDateTime firstDate  = QDateTime::fromString(timeline->start, TIMESTAMP_FORMAT);
        QDateTime lastDate   = QDateTime::fromString(timeline->end, TIMESTAMP_FORMAT);

        if (localTimeline->keys().count() > 0) {
            m_end = localTimeline->keys().at(localTimeline->keys().count()-1);
        }
        else {
            m_end = 0;
        }
        emit signalSyncSeekbarRange(m_end, localTimeline->value(m_end));
    }
}

QMap<int,int> *CMediaControl::buildLocalTimeline(QString first, QString last)
{
    QMap<int,int> *map = new QMap<int,int>();
    QDateTime firstDate  = QDateTime::fromString(first, TIMESTAMP_FORMAT);
    QDateTime lastDate   = QDateTime::fromString(last, TIMESTAMP_FORMAT);

    qint64 length = firstDate.msecsTo(lastDate);

    int t = 0;
    int n = 0;
    while (t >= 0 && t <= length) {
        // Increment step {33,34,33}
        int d = n%3;
        switch (d) {
        case 1:
            d = 34;
            break;
        default: // case 0, case 2
            d = 33;
            break;
        }
        map->insert(t, n);
        t += d;
        n++;
    }

    return map;
}

void CMediaControl::receiveEOP()
{
    emit signalEOP(true);
}

void CMediaControl::slotStateChanged(bool state)
{
    EventObject *obj = new EventObject();
    if (state) {
        sendMessage(EventType::PLAY, obj);
    }
    else {
        sendMessage(EventType::PAUSE, obj);
    }
}

void CMediaControl::slotSliderValueChanged(int value)
{
    int key = localTimeline->key(value);
    TimelineValue *obj = new TimelineValue();
    obj->value = key;
    sendMessage(EventType::TIMELINE_SEEK, obj);
}

void CMediaControl::loadFrameData(EventObject *eventObject)
{
    if (localTimeline) {
        TimelineValue *posObj = static_cast<TimelineValue*>(eventObject);
        if (posObj) {
            int key = posObj->value;
            emit signalPosition(key, localTimeline->value(key));
        }
    }
}
