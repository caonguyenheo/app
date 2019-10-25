#ifndef CVIDEOPLAYER_H
#define CVIDEOPLAYER_H

#include "../mediator/IColleague.h"
//#include "viewerpanel.h"

class CVideoPlayer : public IColleague
{
    Q_OBJECT

public:
    CVideoPlayer(IMediator *mediator, int id);

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

#endif //CVIDEOPLAYER_H
