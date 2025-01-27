#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Encoding thread, encoding a video out of a ROSBag
class PublishVideoThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PublishVideoThread(const Utils::UI::PublishVideoParameters& parameters,
                       QObject*                                 parent = nullptr);

    void
    run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;

    const Utils::UI::PublishVideoParameters& m_parameters;

    static constexpr int PROGRESS = 0;
};
