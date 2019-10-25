#include "CGpsSimulator.h"

#include <QDebug>
//#include "viewerpanel.h"
#include <cmath>

const QString GPS_PREFIX = "gnss_0";

CGpsSimulator::CGpsSimulator(IMediator *mediator, int id)
{
    p_mediator = mediator;
    m_id = id;
    mediator->addNewRegistered(this, getSourcePrefix(id));
    localTimeline = nullptr;
};

void CGpsSimulator::receiveMessage(EventType eventType, EventObject *eventObject)
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
        break;
    default:
        break;
    }
}

void CGpsSimulator::sendMessage(EventType type, EventObject *eventObject)
{
    p_mediator->broadcast(type, eventObject);
}

QString CGpsSimulator::getSourcePrefix(int id)
{
    Q_UNUSED(id);
    return GPS_PREFIX;
}

QString CGpsSimulator::getBasePath(QString kittiPath)
{
    QString prefix = getSourcePrefix(m_id);
    return kittiPath + "/" + prefix;
}

void CGpsSimulator::loadTimestamp(EventObject *eventObject)
{
    TimelineObject *timeline = static_cast<TimelineObject*>(eventObject);
    QString dataPath = getBasePath(timeline->dataPath);
    QString start = timeline->start;
    QString end = timeline->end;
    QMap<int,QString> map;
    QDateTime startDate = QDateTime::fromString(start, TIMESTAMP_FORMAT);
    QDateTime endDate   = QDateTime::fromString(end,   TIMESTAMP_FORMAT);

    QDirIterator it(dataPath,
                    QStringList() << "timestamps*.txt",
                    QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        QString tsPath = it.next();
        int imgIndex = -1;

        // Read "gnss.txt" data rows
        QString imgName = "gnss.txt";
        QString imgPath = tsPath;
        imgPath.truncate(imgPath.lastIndexOf("/"));
        imgPath += "/data/" + imgName;
        QStringList gnssData;
        readFileBuffer(imgPath.toStdString().c_str(), gnssData);

        // Read timestamps_xxxxxxxxxx.txt
        QFile f(tsPath);
        if (f.open(QIODevice::ReadOnly)) {
            QTextStream stream(&f);
            while (!stream.atEnd()) {
                imgIndex++;
                QString line = stream.readLine();
                line.truncate(line.lastIndexOf(".") + 3 + 1);

                // Get duration from begin timestamp
                QDateTime date = QDateTime::fromString(line, TIMESTAMP_FORMAT);
                int duration = int(startDate.msecsTo(date));

                QString data = gnssData.at(imgIndex);
                map.insert(duration, data);
            }
            f.close();

            // Build local timeline
            if (localTimeline) {
                localTimeline->clear();
                delete localTimeline;
            }
            localTimeline = buildLocalTimeline(map, start, end);
        }
        else {
            // Error open file
            qCritical() << "Can not access to" << tsPath;
        }
    }
}

QMap<int,QString> *CGpsSimulator::buildLocalTimeline(QMap<int,QString> &ts_map, QString first, QString last)
{
    QDateTime firstDate  = QDateTime::fromString(first, TIMESTAMP_FORMAT);
    QDateTime lastDate   = QDateTime::fromString(last, TIMESTAMP_FORMAT);
    QMap<int,QString> *timelineMap = new QMap<int,QString>();
    QMap<int, QString>::const_iterator mapEnd = ts_map.end();
    QMap<int, QString>::const_iterator mapNow = ts_map.begin();

    //timelineMap->insert(0, "");
    int length = int(firstDate.msecsTo(lastDate));

    int t = 0;
    int n = 0;
    while (t >= 0 && t <= length) {
        // second per frame step {33,34,33}
        int secPerFrame;
        int d = n%3;
        switch (d) {
        case 1:
            secPerFrame = 34;
            break;
        default: // case 0, case 2
            secPerFrame = 33;
            break;
        }

        // Find the range of current position {-1/2; +1/2}
        int lessPos = std::max(t - secPerFrame/2, 0);
        int morePos = std::min(t + (secPerFrame-secPerFrame/2), length);

        int minDeviation = -1;
        QString dataFileName = "";
        while (mapNow.key() <= morePos) {
            // Check the current position of Iterator is in {lessPos,morePos} range
            if (isInRange(lessPos, morePos, mapNow.key())) {
                int deviation = std::abs(t - mapNow.key());
                if (minDeviation == -1 || minDeviation > deviation) {
                    minDeviation = deviation;
                    dataFileName = mapNow.value();
                }
            }
            // Increase mapNow, stop at end position
            if (mapNow == mapEnd) {
                break;
            }
            else {
                mapNow++;
            }
        }

        // Add existed path to map
        if (dataFileName.compare("") != 0) {
            timelineMap->insert(t, dataFileName);
        }

        t += secPerFrame;
        n++;
    }

    return timelineMap;
}

void CGpsSimulator::loadFrameData(EventObject *eventObject)
{
    if (localTimeline) {
        TimelineValue *posObj = static_cast<TimelineValue*>(eventObject);
        if (posObj) {
            int key = posObj->value;
            QString value = localTimeline->value(key);

            if (value.compare("") != 0) {
                emit signalPosition(key, value);
                return;
            }
        }
    }

    slotFinishedRender();
}

void CGpsSimulator::slotFinishedRender()
{
    StateEvent *event  = new StateEvent();
    event->source_id = getSourcePrefix(m_id);
    event->status = true;
    sendMessage(EventType::RENDER_STATE, event);
}
