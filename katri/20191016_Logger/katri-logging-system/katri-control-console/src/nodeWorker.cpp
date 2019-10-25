#include "nodeWorker.h"

NodeWorker::NodeWorker(int argc, char **argv, QObject *parent)
    : QThread(parent)
{
    _argc = argc;
    _argv = argv;
}

NodeWorker::~NodeWorker()
{
    delete _argv;
}

void NodeWorker::run()
{
    try {
        rclcpp::init(_argc, _argv);
        _node = rclcpp::Node::make_shared(AppConfig::instance().getNodeName().toStdString());
        qDebug() << "Node is created: " << AppConfig::instance().getNodeName();

        _publisher = _node->create_publisher<katri_msgs::msg::Command>(
            AppConfig::instance().getTopicName().toStdString());
        qDebug() << "Publisher is created: " << AppConfig::instance().getTopicName();

        _subscriptionPC2 = _node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
                qDebug() << QString::fromStdString(msg->header.frame_id);
            });

        _subscription = _node->create_subscription<katri_msgs::msg::Command>(
            AppConfig::instance().getTopicName().toStdString(),
            std::bind(&NodeWorker::callback, this, _1));

        for (int i = 0; i < AppConfig::instance().getLoggerNodes().size(); ++i) {
            auto receiver = [this](katri_msgs::msg::Image::UniquePtr msg) {
                //                qDebug() << "Camera ID: " << QString::fromStdString(msg->id) << ", Type: "
                //                         << QString::fromStdString(Command::getTypeName(Command::Type::CAMERA))
                //                         << ", Size: " << msg->size;

                emit signalReceiveVideoFrame(static_cast<uint8_t>(Command::Type::CAMERA),
                                             QString::fromStdString(msg->id).toInt(),
                                             const_cast<unsigned char *>(msg->data.data()),
                                             msg->size);
            };

            _cameraSubcribers
                .insert(AppConfig::instance().getLoggerNodes().value(i),
                        _node->create_subscription<katri_msgs::msg::Image>(
                            AppConfig::instance().getLoggerNodes().value(i).toStdString(),
                            receiver));
        }

        _one_off_timer = _node->create_wall_timer(2s, [this]() {
            _retry -= 1;

            if (_retry < 0) {
                this->_one_off_timer->cancel();
            } else {
                this->ping();
            }
        });

        rclcpp::spin(_node);
        rclcpp::shutdown();

    } catch (std::exception &e) {
        qDebug() << "Got exception when starting a Node: " << e.what();
    } catch (...) {
        qDebug() << "Got error when starting a Node";
    }
}

void NodeWorker::publish(Command &cmd)
{
    try {
        katri_msgs::msg::Command message = katri_msgs::msg::Command();
        message.action = static_cast<uint8_t>(cmd.getAction());
        message.type = static_cast<uint8_t>(cmd.getType());
        message.id = cmd.getId();
        message.kitti_no = cmd.getKittiNo();

        this->_publisher->publish(message);
    } catch (std::exception &e) {
        qDebug() << "Got exception when publishing a message: " << e.what();
    } catch (...) {
        qDebug() << "Got error when publishing a message";
    }
}

void NodeWorker::callback(const katri_msgs::msg::Command::SharedPtr msg)
{
    Command command(static_cast<Command::Action>(msg->action),
                    static_cast<Command::Type>(msg->type),
                    msg->id,
                    msg->kitti_no);

    qDebug() << "Received command: "
             << QString::fromStdString(Command::getActionName(command.getAction())) << ", "
             << QString::fromStdString(Command::getTypeName(command.getType())) << ", "
             << QString::fromStdString(command.getId()) << ", "
             << QString::number(command.getKittiNo());

    if (command.isValid()) {
        if (command.isAckFromLoggerNodeWithLoggerNodeName()) {
            // Stop retry
            _retry = 0;
            this->_one_off_timer->cancel();

            qDebug() << "Katri Logger Node: " << QString::fromStdString(command.getId())
                     << " is connected to Katri Controller Node: "
                     << AppConfig::instance().getNodeName();

        } else if (command.isPingFromLoggerNode()) {
            // Send ACK message to Logger Node
            ack(command.getId());
        } else {
            qDebug() << "Ignore a command";
        }
    } else {
        qDebug() << "An invalid command, just ignore it.";
    }
}

void NodeWorker::ack(std::string loggerNodeName)
{
    katri_msgs::msg::Command message = katri_msgs::msg::Command();
    message.id = loggerNodeName;
    message.action = static_cast<uint8_t>(Command::Action::ACK);
    message.type = static_cast<uint8_t>(Command::Type::LOGGER);
    _publisher->publish(message);

    qDebug() << "Publish ACK message";
}

void NodeWorker::ping()
{
    katri_msgs::msg::Command message = katri_msgs::msg::Command();
    message.id = AppConfig::instance().getNodeName().toStdString();
    message.action = static_cast<uint8_t>(Command::Action::PING);
    message.type = static_cast<uint8_t>(Command::Type::CONTROLLER);
    _publisher->publish(message);

    qDebug() << "Publish PING message";
}
