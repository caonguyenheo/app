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
#include "sensor_msgs/msg/point_cloud2.hpp"

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

Q_SIGNALS:
    void signalReceiveVideoFrame(int deviceType, int deviceId, const uchar *data, uint len);

private:
    int _argc;
    char **_argv = nullptr;
    QHash<QString, rclcpp::Subscription<katri_msgs::msg::Image>::SharedPtr> _cameraSubcribers;
    rclcpp::Subscription<katri_msgs::msg::Command>::SharedPtr _subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriptionPC2;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<katri_msgs::msg::Command>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _one_off_timer;
    volatile int _retry = 10;
};

#endif // NODEWORKER_H
