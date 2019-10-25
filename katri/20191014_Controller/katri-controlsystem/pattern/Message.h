#ifndef MESSAGE_H
#define MESSAGE_H

//Chain of Responsibility pattern

#include <iostream>
#include <memory>
#include <cstdint>
#include <../logger/QLogger.h>
#include "../define.h"
#include <QMutex>

using namespace std;

enum class MessageType : std::uint8_t { POPUP, INLINE, LOG};
enum class MessageStatus : std::uint8_t { SUCCESS, FAILED, WARNING };

class AbstractLogger : public QObject
{
    Q_OBJECT

protected:
    MessageType currentLevel;
    std::shared_ptr<AbstractLogger> nextLogger;

public:
    AbstractLogger(const MessageType &level);
    virtual ~AbstractLogger() {}

    void setNextLogger(const std::shared_ptr<AbstractLogger> &logger);
    void logMessage(const MessageType &level, const MessageStatus &status, const std::string &message, std::string modules) const;

Q_SIGNALS:
    void signalMessage(const MessageStatus &status, const std::string &message) const;

protected:
    virtual void write(const MessageStatus &status, const std::string &message, std::string modules = MODULE_LOG) const = 0;
};

class InlineMessage : public AbstractLogger
{
public:
    InlineMessage(const MessageType &level);

protected:
    void write(const MessageStatus &status, const std::string &message, std::string modules = MODULE_LOG) const override;
};

class PopupMessage : public AbstractLogger
{
public:
    PopupMessage(const MessageType &level);

protected:
    void write(const MessageStatus &status,const std::string &message, std::string modules = MODULE_LOG) const override;
};

class LogMessagefile : public AbstractLogger
{
public:
    LogMessagefile(const MessageType &level);

protected:
    void write(const MessageStatus &status, const std::string &message, std::string modules = MODULE_LOG) const override;
};

class Message : public QObject
{
    Q_OBJECT

public:
    static Message *getInstance();
    static void logMessage(const MessageType &type, const MessageStatus &status, const std::string &message, std::string modules);
    shared_ptr<AbstractLogger> getChainLoggers();

     static void setlogMessage(const std::string &name, const std::string &modules);

Q_SIGNALS:
    void signalInlineMessage(const MessageStatus &status, const std::string &message) const;
    void signalPopUpMessage(const MessageStatus &status, const std::string &message) const;

public slots:
    void handlerInlineMessage(const MessageStatus &status, const std::string &message) const;
    void handlerPopUpMessage(const MessageStatus &status, const std::string &message) const;

private:
  Message() {}
  static Message *instance;
};

#endif // MESSAGE_H
