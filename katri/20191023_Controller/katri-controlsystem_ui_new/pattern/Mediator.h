#ifndef MEDIATOR_H
#define MEDIATOR_H

#include <iostream>
#include <vector>
#include <string>
#include <QObject>
#include <QMutex>
#include "../katrilogger/command.h"
#include "Message.h"
#include "../katricontrols/camera/viewerperformance.h"
#include "../katricontrols/recordbutton.h"
#include "../katricontrols/controll.h"

using namespace  std;

class Device;
class Observer;

class Mediator : public QObject
{

    Q_OBJECT

Q_SIGNALS:
    void signalUpdateStatusCameraView(uint id, bool status);
    void signalUpdateStatusCameraControl(uint id, bool status);
    void signalAllButtonControlDisable();
    void signalAllButtonControlEnable();

public:
    Mediator();
    virtual ~Mediator() {}
    virtual void addDevice( Device* const c ) = 0;
    virtual void attachObserver(Observer *obs) = 0;
    virtual bool checkStatus( Device* const sender = nullptr ) = 0;

protected:
    Mediator* mediator = nullptr;
};

class Device : public QObject
{    
    Q_OBJECT

public:
    Device(Mediator* const m, const int i , Command::Type type);
    virtual ~Device() {}
    int getID();
    Command::Type getType();
    bool getStatus();

    RecordButton* getRecordButton();
public slots:
    virtual void update( bool ) = 0;

protected:
    Mediator *mediator;
    int id;
    bool status = false;
    RecordButton *recordButton = nullptr;
    Command::Type deviceType;
};
void signalUpdateStatusCameraView(uint id, bool status);
void signalUpdateStatusCameraControl(uint id, bool status);
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
    Q_OBJECT

public:
    ConcreteMediator();
    ~ConcreteMediator() override;
    void addDevice( Device* const c ) override;
    void attachObserver(Observer *obs) override;
    bool checkStatus( Device* const sender = nullptr ) override;
    bool checkAllDisable();
    void sendCommandToObserver(bool isNewDS, Device* const sender = nullptr);
    bool isAllDeviceOff;
    bool checkStatusView(Device * const sender);

//Q_SIGNALS:
//    void signalUpdateStatusCameraView(uint id, bool status);
//    void signalUpdateStatusCameraControl(uint id, bool status);

    bool checkAllEnable();
private:
    std::vector<Device*> Devices;
    std::vector<Observer*> observers;
};

#endif
