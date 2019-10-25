#include "Message.h"
#include <QDebug>

Message* Message::instance = nullptr;

shared_ptr<AbstractLogger> Message::getChainLoggers()
{
    auto popupMessage = make_shared<PopupMessage>(MessageType::POPUP);
    auto inlineMessage = make_shared<InlineMessage>(MessageType::INLINE);
    auto fileMessage = make_shared<LogMessagefile>(MessageType::LOG);
    popupMessage->setNextLogger(inlineMessage);
    inlineMessage->setNextLogger(fileMessage);
//    qDebug() << popupMessage.use_count();
//    qDebug() << inlineMessage.use_count();
    return std::move(popupMessage);
}

Message* Message::getInstance()
{
  if ( !instance )
  {
    instance = new Message();
    //default : write to file log/log.txt
    QLogger::QLoggerManager *manager = QLogger::QLoggerManager::getInstance();
    manager->addDestination("log.txt", MODULE_LOG, QLogger::LogLevel::Info);
  }
  return instance;
}

void Message::setlogMessage(const std::string &name, const std::string &modules)
{
    QLogger::QLoggerManager *manager = QLogger::QLoggerManager::getInstance();
    manager->addDestination(QString::fromStdString(name), QString::fromStdString(modules), QLogger::LogLevel::Info);
}
void Message::logMessage(const MessageType &type, const MessageStatus &status, const std::string &message, std::string modules)
{
    auto logMsg = Message::getInstance()->getChainLoggers();
    logMsg->logMessage(type,status,message,modules);
}

void Message::handlerInlineMessage(const MessageStatus &status, const std::string &message) const
{
    emit signalInlineMessage(status,message);
}

void Message::handlerPopUpMessage(const MessageStatus &status, const std::string &message) const
{
    emit signalPopUpMessage(status,message);
}


AbstractLogger::AbstractLogger(const MessageType &level) : currentLevel(level)
{}

void AbstractLogger::setNextLogger(const std::shared_ptr<AbstractLogger> &logger)
{
    nextLogger = logger;
}

void AbstractLogger::logMessage(const MessageType &level, const MessageStatus &status, const string &message, std::string modules) const
{
    if(currentLevel == level)
    {
        write(status,message, modules);
    }
    if(nextLogger != nullptr)
    {
        nextLogger->logMessage(level, status, message, modules);
    }
}

InlineMessage::InlineMessage(const MessageType &level) : AbstractLogger(level)
{
    connect(this,&InlineMessage::signalMessage,Message::getInstance(),&Message::handlerInlineMessage);
}

void InlineMessage::write(const MessageStatus &status, const std::string &message, std::string modules) const
{
    //implement inline
    static QMutex mutex;
    mutex.lock();
    QLogger::QLog_Info(QString::fromStdString(modules),QString::fromStdString(message));
    emit signalMessage(status,message);
    mutex.unlock();
}

PopupMessage::PopupMessage(const MessageType &level) : AbstractLogger(level)
{
    connect(this,&PopupMessage::signalMessage,Message::getInstance(),&Message::handlerPopUpMessage);
}

void PopupMessage::write(const MessageStatus &status, const std::string &message, std::string modules) const
{
    //implement pop-up
    static QMutex mutex;
    mutex.lock();
    QLogger::QLog_Info(QString::fromStdString(modules),QString::fromStdString(message));
    emit signalMessage(status,message);
    mutex.unlock();
}

LogMessagefile::LogMessagefile(const MessageType &level) : AbstractLogger(level)
{
    connect(this,&LogMessagefile::signalMessage,Message::getInstance(),&Message::handlerPopUpMessage);
}

void LogMessagefile::write(const MessageStatus &status, const std::string &message, std::string modules ) const
{
    //implement Write File
    static QMutex mutex;
    mutex.lock();
    QLogger::QLog_Info(QString::fromStdString(modules),QString::fromStdString(message));
    mutex.unlock();
}

