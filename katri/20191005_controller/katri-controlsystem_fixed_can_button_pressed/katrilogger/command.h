#ifndef COMMAND_H
#define COMMAND_H

#include "appConfig.h"
#include "cstdint"

class Command
{
public:
    enum class Action : uint8_t {
        START_NEW_KITTI = 0x00,
        START = 0x01,
        STOP = 0x02,
        COPY = 0x03,
        PING = 0x04,
        ACK = 0x05,
        RELAY_OK_STATE = 0x06,
        RELAY_FAIL_STATE = 0x07
    };

    enum class Type : uint8_t {
        CAN = 0x01,
        CAMERA = 0x02,
        LIDAR = 0x03,
        RADAR = 0x04,
        MOBILEYE = 0x05,
        GNSS = 0x06,
        LOGGER = 0x07,
        CONTROLLER = 0x08,
        ALL = 0x09
    };

public:
    Command();
    Command(const Command &cmd);
    Command(Action action, Type type, std::string id, uint kitti_no);
    Action getAction() const;
    Type getType() const;
    std::string getId() const;
    uint getKittiNo() const;
    bool isValid() const;
    bool isConnectionCommand() const;
    bool isPingFromLoggerNode() const;
    bool isAckFromLoggerNodeWithLoggerNodeName() const;
    bool isStartCommand() const;
    bool isStartNewKittiCommand() const;

public:
    static std::string getActionName(Action _action);
    static std::string getTypeName(Type _type);

    bool isOkRelayResponse() const;
    bool isFailRelayResponse() const;
private:
    Action _action;
    Type _type;
    std::string _id;
    uint _kitti_no;
};

#endif // COMMAND_H
