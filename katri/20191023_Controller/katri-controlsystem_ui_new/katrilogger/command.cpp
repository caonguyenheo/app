#include "command.h"
#include "appConfig.h"
Command::Command()
{
    // dummy data, just ignore it
    _action = Action::START;
    // dummy data, just ignore it
    _type = Type::LOGGER;
    // dummy data, just ignore it
    _id = "";
    _kitti_no = 0;
}
Command::Command(const Command &cmd)
{
    _action = cmd._action;
    _type = cmd._type;
    _id = cmd._id;
    _kitti_no = cmd._kitti_no;
}
Command::Command(Action action, Type type, std::string id, uint kitti_no)
{
    this->_action = action;
    this->_type = type;
    this->_id = id;
    this->_kitti_no = kitti_no;
}

Command::Type Command::getType() const
{
    return _type;
}

Command::Action Command::getAction() const
{
    return _action;
}

std::string Command::getId() const
{
    return _id;
}

uint Command::getKittiNo() const
{
    return _kitti_no;
}

bool Command::isValid() const
{
    if (_action != Action::START_NEW_KITTI && _action != Action::COPY && _action != Action::STOP
        && _action != Action::START && _action != Action::PING && _action != Action::ACK
        && _action != Action::RELAY_OK_STATE && _action != Action::RELAY_FAIL_STATE)
        return false;

    if (_type != Type::CAN && _type != Type::GNSS && _type != Type::LIDAR && _type != Type::RADAR
        && _type != Type::CAMERA && _type != Type::MOBILEYE && _type != Type::LOGGER
        && _type != Type::CONTROLLER && _type != Type::ALL)
        return false;

    return true;
}

bool Command::isConnectionCommand() const
{
    if (_action == Action::ACK || _action == Action::PING)
        return true;
    return false;
}

/**
 * Used for Controller only
 * @brief Command::isPingFromLoggerNode
 * @return 
 */
bool Command::isPingFromLoggerNode() const
{
    if (_action == Action::PING && _type == Type::LOGGER
        && _id != AppConfig::instance().getNodeName().toStdString()) {
        return true;
    }
    return false;
}

bool Command::isOkRelayResponse() const
{
    if (_action == Action::RELAY_OK_STATE && _type == Type::LOGGER
            && _id != AppConfig::instance().getNodeName().toStdString()) {
        return true;
    }
    return false;
}

bool Command::isFailRelayResponse() const
{
    if (_action == Action::RELAY_FAIL_STATE && _type == Type::LOGGER
            && _id != AppConfig::instance().getNodeName().toStdString()) {
        return true;
    }
    return false;
}

/**
 * Used for Controller only
 * @brief Command::isAckFromLoggerNodeWithLoggerNodeName
 * @return 
 */
bool Command::isAckFromLoggerNodeWithLoggerNodeName() const
{
    if (_action == Action::ACK && _type == Type::CONTROLLER
        && _id != AppConfig::instance().getNodeName().toStdString()) {
        return true;
    }
    return false;
}

bool Command::isStartCommand() const
{
    if (_action == Action::START) {
        return true;
    }

    return false;
}

bool Command::isStartNewKittiCommand() const
{
    if (_action == Action::START_NEW_KITTI) {
        return true;
    }

    return false;
}

std::string Command::getActionName(Action action)
{
    if (action == Command::Action::START_NEW_KITTI)
        return "START_NEW_KITTI";
    if (action == Command::Action::COPY)
        return "RELAY";
    if (action == Command::Action::STOP)
        return "STOP";
    if (action == Command::Action::START)
        return "START";
    if (action == Command::Action::PING)
        return "PING";
    if (action == Command::Action::ACK)
        return "ACK";
    if (action == Command::Action::RELAY_OK_STATE)
        return "RELAY_OK_STATE";
    if (action == Command::Action::RELAY_FAIL_STATE)
        return "RELAY_FAIL_STATE";
    return "INVALID";
}

std::string Command::getTypeName(Type type)
{
    if (type == Command::Type::CAN)
        return "CAN";
    if (type == Command::Type::GNSS)
        return "GNSS";
    if (type == Command::Type::LIDAR)
        return "LIDAR";
    if (type == Command::Type::RADAR)
        return "RADAR";
    if (type == Command::Type::CAMERA)
        return "CAMERA";
    if (type == Command::Type::MOBILEYE)
        return "MOBILEYE";
    if (type == Command::Type::LOGGER)
        return "LOGGER";
    if (type == Command::Type::CONTROLLER)
        return "CONTROLLER";
    if (type == Command::Type::ALL)
        return "ALL";
    return "INVALID";
}
