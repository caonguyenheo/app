#ifndef ICOLLEAGUE_H_
#define ICOLLEAGUE_H_

#include <QString>
#include "IMediator.h"

class IColleague : public QObject
{

public:
    virtual void sendMessage(EventType Type, EventObject *eventObject) = 0;
    virtual void receiveMessage(EventType Type, EventObject *eventObject) = 0;
    virtual QString getSourcePrefix(int id) = 0;
    virtual QString getBasePath(QString kittiPath) = 0;

protected:
    IMediator *p_mediator;
    EventType type;
};

#endif //ICOLLEAGUE_H_
