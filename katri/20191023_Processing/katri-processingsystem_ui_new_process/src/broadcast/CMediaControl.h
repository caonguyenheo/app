#ifndef CMEDIACONTROL_H
#define CMEDIACONTROL_H

#include "../mediator/IColleague.h"

class CMediaControl : public IColleague
{
    Q_OBJECT

public:
    CMediaControl(IMediator *mediator);

    void receiveMessage(EventType Type, EventObject *eventObject) override;

    void sendMessage(EventType Type, EventObject *eventObject) override;

    QString getSourcePrefix(int id) override;

    QString getBasePath(QString kittiPath) override;

    void loadTimestamp(EventObject *eventObject);

    void loadFrameData(EventObject *eventObject);

    QMap<int,int> *buildLocalTimeline(QString first, QString last);

    void receiveEOP();

Q_SIGNALS:
//    void signalValueChanged(int value);
    void signalSyncSeekbarRange(int,int);
    void signalPosition(int,int);
    void signalEOP(bool);

public Q_SLOTS:
    void slotStateChanged(bool state);
    void slotSliderValueChanged(int value);

private:
    int m_curPosition = 0; //current time
    int m_length = 0; // total time
    QMap<int,int> *localTimeline;
    int m_end;
};

#endif // CMEDIACONTROL_H
