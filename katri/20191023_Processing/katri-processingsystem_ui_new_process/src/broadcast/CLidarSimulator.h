#ifndef CLIDARSIMULATOR_H
#define CLIDARSIMULATOR_H

#include "../mediator/IColleague.h"
//#include "viewerpanel.h"

class CLidarSimulator : public IColleague
{
Q_OBJECT
public:
    CLidarSimulator(IMediator *mediator, int id);

    void receiveMessage(EventType Type, EventObject *eventObject) override;

    void sendMessage(EventType Type, EventObject *eventObject) override;

    QString getSourcePrefix(int id) override;

    QString getBasePath(QString kittiPath) override;

    void loadTimestamp(EventObject *eventObject);

    QMap<int,QString> *buildLocalTimeline(QMap<int,QString> &map, QString first, QString last);

    void loadFrameData(EventObject *eventObject);

Q_SIGNALS:
    void signalPosition(int, QString);

public Q_SLOTS:
    void slotFinishedRender();

private:
    int        m_id;
    QMap<int,QString> *localTimeline;
};

#endif // CLIDARSIMULATOR_H
