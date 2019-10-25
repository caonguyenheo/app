#include "nodeWorker.h"

NodeWorker::NodeWorker(int argc, char **argv, QObject *parent) : QThread (parent)
{
    _argc = argc;
    _argv = argv;

    _logger = LoggerManager::getAppLogger();
}

NodeWorker::~NodeWorker(){
    delete _argv;
}

void NodeWorker::run(){
    try {
        rclcpp::init(_argc, _argv);
        _node = rclcpp::Node::make_shared(AppConfig::instance().getNodeName().toStdString());

        _subscription = _node->create_subscription<katri_msgs::msg::Command>(
            AppConfig::instance().getTopicName().toStdString(),
            std::bind(&NodeWorker::callback, this, std::placeholders::_1));

        _publisher = _node->create_publisher<katri_msgs::msg::Command>(
            AppConfig::instance().getTopicName().toStdString());

        _imagePublisher = _node->create_publisher<katri_msgs::msg::Image>(
            AppConfig::instance().getNodeName().toStdString());

//        _one_off_timer = _node->create_wall_timer(2s, [this]() {
//            _retry -= 1;

//            if (_retry < 0) {
//                this->_one_off_timer->cancel();
//            } else {
//                this->ping();
//            }
//        });

        _one_off_timer = _node->create_wall_timer(10s, [this]() {
            this->ping();
        });

        rclcpp::spin(_node);
        rclcpp::shutdown();
    } catch (std::exception &e) {
        if (_logger)
            _logger->debug("Got exception when starting a Ros2 Node: {}", e.what());
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when starting a Ros2 Node");
    }
}

void NodeWorker::shutDown()
{
    rclcpp::shutdown();
}

void NodeWorker::sendImage(const QString &cameraId, uchar *data, size_t size)
{
    try {
        auto image = std::make_shared<katri_msgs::msg::Image>();
        image->data.resize(size);
        memcpy(&image->data[0], data, size);
        image->size = static_cast<unsigned int>(size);
        image->id = cameraId.toStdString();
        _imagePublisher->publish(image);

    } catch (std::exception &e) {
        if (_logger)
            _logger->debug("Got exception when sending image: {}", e.what());
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when sending image");
    }
}

void NodeWorker::relayState(bool isSuccessful) {

    if(isSuccessful) {

        katri_msgs::msg::Command message = katri_msgs::msg::Command();
        // Send logger node name back to controller to identify which logger is available
        message.id = AppConfig::instance().getNodeName().toStdString();
        message.action = static_cast<uint8_t>(Command::Action::RELAY_OK_STATE);
        message.type = static_cast<uint8_t>(Command::Type::LOGGER);
        _publisher->publish(message);

        if (_logger)
            _logger->info("Sent RELAY_STATE command Done: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_OK_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          message.id);
        qDebug()<<"relayState successfuly";
    }
    else {

        katri_msgs::msg::Command message = katri_msgs::msg::Command();
        message.id = AppConfig::instance().getNodeName().toStdString();
        message.action = static_cast<uint8_t>(Command::Action::RELAY_FAIL_STATE);
        message.type = static_cast<uint8_t>(Command::Type::LOGGER);
        _publisher->publish(message);

        if (_logger)
            _logger->info("Sent RELAY_STATE command Fail: {} {} {}",
                          Command::getActionName(Command::Action::RELAY_FAIL_STATE),
                          Command::getTypeName(Command::Type::LOGGER),
                          message.id);
        qDebug()<<"relayState Fail";
    }
}

void NodeWorker::ack()
{
    katri_msgs::msg::Command message = katri_msgs::msg::Command();
    // Send logger node name back to controller to identify which logger is available
    message.id = AppConfig::instance().getNodeName().toStdString();
    message.action = static_cast<uint8_t>(Command::Action::ACK);
    message.type = static_cast<uint8_t>(Command::Type::CONTROLLER);
    _publisher->publish(message);

    if (_logger)
        _logger->info("Sent ACK command: {} {} {}",
                      Command::getActionName(Command::Action::ACK),
                      Command::getTypeName(Command::Type::CONTROLLER),
                      message.id);
}

void NodeWorker::ping()
{
    katri_msgs::msg::Command message = katri_msgs::msg::Command();
    message.id = AppConfig::instance().getNodeName().toStdString();
    message.action = static_cast<uint8_t>(Command::Action::PING);
    message.type = static_cast<uint8_t>(Command::Type::LOGGER);

    try {
        _publisher->publish(message);
    } catch (...) {
        if (_logger)
            _logger->debug("Got error when Ping message");
    }
    if (_logger)
        _logger->info("Sent PING command: {} {} {}",
                      Command::getActionName(Command::Action::PING),
                      Command::getTypeName(Command::Type::LOGGER),
                      message.id);
}

void NodeWorker::callback(const katri_msgs::msg::Command::SharedPtr msg)
{
    Command command(static_cast<Command::Action>(msg->action),
                    static_cast<Command::Type>(msg->type),
                    msg->id,
                    msg->kitti_no);

    if (_logger) {
        _logger->debug("Received command: {}, {}, {}",
                       Command::getActionName(command.getAction()),
                       Command::getTypeName(command.getType()),
                       command.getId());
    }

    if (command.isValid()) {
        if (command.isConnectionCommand()) {
            if (command.isAckFromControllerNode()) {
                // Controller stop sending ACK message -> redundant
                // Stop retry
                /*_retry = 0;
                this->_one_off_timer->cancel();

                // Logger is connected to Controller, log message out
                if (_logger)
                    _logger->info("Katri Logger Node: {} is connected to Katri Controller Node: {}",
                                  command.getId(),
                                  AppConfig::instance().getControllerNodeName().toStdString());*/
            } else if (command.isPingFromControllerNode()) {
                // Send ACK message to Controller Node
                //ack();

                //
                QFile file("ping.txt");
                file.open(QIODevice::ReadWrite);
                QByteArray str;
                str = file.readAll();
                if (str.length() <= 0) {
                    file.write("PING");
                }
                file.close();

            } else {
                _logger->debug("Ignore the command");
            }
        } else {
            emit messageReceived(command);
        }
    } else {
        if (_logger) {
            _logger->debug("Ignore an invalid command.");
        }
    }
}
