#ifndef CCANSIMULATOR_H
#define CCANSIMULATOR_H

#include "../mediator/IColleague.h"
//#include "viewerpanel.h"

class CCanSimulator : public IColleague
{
    Q_OBJECT

public:
    CCanSimulator(IMediator *mediator, int id);

    virtual void receiveMessage(EventType Type, EventObject *eventObject = nullptr) override;

    virtual void sendMessage(EventType Type, EventObject *eventObject = nullptr) override;

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
    QDateTime startDate;
    QDateTime endDate;
};

#endif // CCANSIMULATOR_H
