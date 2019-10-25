#ifndef NODEWORKER_H
#define NODEWORKER_H

#include "QHash"
#include "QThread"
#include "appConfig.h"
#include "command.h"
#include "config.h"
#include "katri_msgs/msg/command.hpp"
#include "katri_msgs/msg/image.hpp"
#include "loggerManager.h"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;

class NodeWorker : public QThread
{
    Q_OBJECT
public:
    NodeWorker(int , char **, QObject *parent = nullptr);
    ~NodeWorker() override;
    void run() override;
    void sendImage(const QString &cameraId, uchar *data, size_t size);
    static void shutDown();

    void relayState(bool isSuccessful);
private:
    int _argc;
    char **_argv = nullptr;
    rclcpp::Subscription<katri_msgs::msg::Command>::SharedPtr _subscription;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<katri_msgs::msg::Command>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _one_off_timer;
    rclcpp::Publisher<katri_msgs::msg::Image>::SharedPtr _imagePublisher;

    volatile int _retry = 10;
    std::shared_ptr<spdlog::logger> _logger = nullptr;

private:
    void callback(const katri_msgs::msg::Command::SharedPtr msg);
    void ack();
    void ping();
signals:
    void messageReceived(const Command command);

};

#endif // NODEWORKER_H
