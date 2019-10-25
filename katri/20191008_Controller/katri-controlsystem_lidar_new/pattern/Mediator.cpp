#include "Mediator.h"

Mediator::Mediator()
{

}

Device::Device( Mediator* const m, const int i ,Command::Type type) :
    mediator( m ), id( i ), deviceType(type) {}

int Device::getID()
{
    return id;
}

Command::Type Device::getType()
{
    return deviceType;
}

bool Device::getStatus()
{
    return status;
}

RecordButton *Device::getRecordButton()
{
    return recordButton;
}


ConcreteDevice::ConcreteDevice(Mediator* const m, const int i , Command::Type type) :
    Device( m, i ,type ) {
    m->addDevice(this);
}

void ConcreteDevice::update(bool status)
{
    QPushButton *pButton = dynamic_cast<QPushButton*>(sender());
    if(pButton!=nullptr)
    {
        RecordButton* pRecordButton = dynamic_cast<RecordButton*>(pButton->parent());
        if(pRecordButton!=nullptr)
            recordButton = pRecordButton;
    }
    static QMutex m;
    m.lock();
    if(this->status != status) {
        this->status = status;
        mediator->checkStatus(this);
    }
    m.unlock();
}

Observer::Observer(Mediator *mediator) {
    subject = mediator;
    subject->attachObserver(this);
}

Mediator* Observer::getSubject() {
    return subject;
}

ConcreteObserver::ConcreteObserver(Mediator *mediator): Observer(mediator){}

void ConcreteObserver::update(Command::Type deviceType, int deviceId, bool recordStatus, bool isNewDS)
{
    std::cout << "ConcreteObserver::update" << std::endl;
    //std::cout << " update observer : " << "type " << deviceType << " ID " << deviceId << " status " << recordStatus<<std::endl;
    emit signalSendCommand(deviceType, deviceId, recordStatus, isNewDS);
}
ConcreteMediator::ConcreteMediator()
{
    isAllDeviceOff = true;
}

ConcreteMediator::~ConcreteMediator()
{
    for ( unsigned int i = 0; i < Devices.size(); i++ )
    {
        delete Devices[ i ];
    }
    Devices.clear();
}

void ConcreteMediator::addDevice(Device* const c)
{
    Devices.push_back( c );
}

void ConcreteMediator::attachObserver(Observer *obs)
{
    observers.push_back(obs);
}

bool ConcreteMediator::checkStatus(Device* const sender)
{
    // Q_UNUSED(sender);
    //send command to observer
    sendCommandToObserver(isAllDeviceOff, sender);
    isAllDeviceOff = checkAllDisable();
    checkAllEnable();
    checkStatusView(sender);
    return isAllDeviceOff;
}

bool ConcreteMediator::checkStatusView(Device* const sender)
{
    RecordButton *button = sender->getRecordButton();
    if(button != nullptr)
    {
        viewerperformance *vPerform = dynamic_cast<viewerperformance*>(button->parent());
        if(vPerform!=nullptr)
        {
            std::cout << "camera performance" << vPerform->getDeviceId() <<std::endl;
            emit signalUpdateStatusCameraView(vPerform->getDeviceId(),sender->getStatus());
            vPerform->updateStyle(sender->getStatus());
        }
        ControlL *control = dynamic_cast<ControlL*>(button->parent()->parent()->parent());
        if(control!=nullptr)
        {
            if(control->getType() == Command::Type::CAMERA)
            {
                std::cout << "control camera" << button->getID() << std::endl;
                emit signalUpdateStatusCameraControl(button->getID(),sender->getStatus());
            }
            control->updateStyle(sender->getStatus());
        }
    }
}

bool ConcreteMediator::checkAllDisable()
{
    bool status = false;
    for(unsigned int i = 0 ; i < Devices.size(); i++)
    {
        if(Devices.at(i)->getStatus() == true)
        {
            status = true;
            break;
        }
    }

    if (status == true)
        emit signalStatusButton(DETECT_RECORD);
    else
        emit signalStatusButton(DETECT_STOP);    

    return !status;
}

bool ConcreteMediator::checkAllEnable()
{
    bool status = true;
    for(unsigned int i = 0 ; i < Devices.size(); i++)
    {
        if(Devices.at(i)->getStatus() == false)
        {
            status = false;
            break;
        }
    }
    if(status == true){
        //std::cout << "all enable" << std::endl;
        emit signalStatusButton(RECORD_ALL);
    }

    return !status;
}

void ConcreteMediator::sendCommandToObserver(bool isNewDS, Device* const sender)
{
    // need to implement
    for (unsigned int i = 0; i < observers.size(); i++)
        observers[i]->update(sender->getType(),sender->getID(),sender->getStatus(), isNewDS);
}

