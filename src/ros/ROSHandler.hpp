#pragma once

#include <QObject>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief The interface connecting ROS to the UI. Inherits from QObject so that signals can be emitted.
 */
class ROSHandler : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    ROSHandler();
};
