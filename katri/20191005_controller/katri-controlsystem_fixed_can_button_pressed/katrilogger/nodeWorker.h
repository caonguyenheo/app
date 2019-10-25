#ifndef NODEWORKER_H
#define NODEWORKER_H
#include "QDebug"
#include "QHash"
#include "QString"
#include "QThread"
#include "command.h"
#include "katri_msgs/msg/command.hpp"
#include "katri_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class NodeWorker : public QThread
{
    Q_OBJECT
public:
    NodeWorker(int, char **, QObject *parent = nullptr);
    ~NodeWorker() override;
    void run() override;
    void publish(Command &);
    void callback(const katri_msgs::msg::Command::SharedPtr msg);
    void ack(std::string loggerNodeName);
    void ping();

    //void relayState(Command &);

Q_SIGNALS:
    void signalReceiveVideoFrame(int deviceType, int deviceId, QByteArray data);
    void signalLostNetWorkConnection(int deviceType, int deviceId, QByteArray data);
    void signalReceiveRelayState(QString loggerNodeName, bool status);
    //void signalLoggerNameAck(QString loggerNodeName);
    void signalPingLoggerNodeId(QString nodeNameId);

private:
    int _argc;
    char **_argv = nullptr;
    QHash<QString, rclcpp::Subscription<katri_msgs::msg::Image>::SharedPtr> _cameraSubcribers;
    rclcpp::Subscription<katri_msgs::msg::Command>::SharedPtr _subscription;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<katri_msgs::msg::Command>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _one_off_timer;
    volatile int _retry = 10;
};

#endif // NODEWORKER_H
