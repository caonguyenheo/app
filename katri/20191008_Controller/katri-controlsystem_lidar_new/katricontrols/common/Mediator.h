#ifndef MEDIATOR_H
#define MEDIATOR_H

#include <iostream>
#include <vector>
#include <string>
#include <QObject>
#include <QMutex>
#include "katricontrols_export.h"
#include "../katrilogger/command.h"

using namespace  std;

class Device;
class Observer;

class Mediator
{
public:
    Mediator();
    virtual ~Mediator() {}
    virtual void addDevice( Device* const c ) = 0;
    virtual void attachObserver(Observer *obs) = 0;
    virtual bool checkStatus( Device* const sender = nullptr ) = 0;
    void setupMediator();

protected:
    Mediator* mediator = nullptr;
};

class KATRICONTROLS_EXPORT Device : public QObject
{    
    Q_OBJECT

public:
    Device(Mediator* const m, const int i , Command::Type type);
    virtual ~Device() {}
    int getID();
    Command::Type getType();
    bool getStatus();

public slots:
    virtual void update( bool ) = 0;

protected:
    Mediator *mediator;
    int id;
    bool status = false;
    Command::Type deviceType;
};

class ConcreteDevice : public Device
{

public:
    ConcreteDevice(Mediator* const m, const int i ,Command::Type type);
    ~ConcreteDevice() {}
    void update( bool status );

};

class Observer : public QObject
{
    Q_OBJECT

public:
    Observer(Mediator *mediator);
    virtual void update(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS) = 0;

protected:
    Mediator *getSubject();

private:
    Mediator *subject;
};

class ConcreteObserver: public Observer {
    Q_OBJECT

Q_SIGNALS:
    void signalSendCommand(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS);

public:
    ConcreteObserver(Mediator *mediator);
    void update(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS);
};

class ConcreteMediator : public Mediator
{

public:
    ConcreteMediator();
    ~ConcreteMediator() override;
    void addDevice( Device* const c ) override;
    void attachObserver(Observer *obs) override;
    bool checkStatus( Device* const sender = nullptr ) override;
    bool checkAllDisable();
    void sendCommandToObserver(bool isNewDS, Device* const sender = nullptr);
    bool isAllDeviceOff;

private:
    std::vector<Device*> Devices;
    std::vector<Observer*> observers;
};

#endif
