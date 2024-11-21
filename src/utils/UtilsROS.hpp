#pragma once

#include <QString>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>

template<typename T, typename U>
concept WriteMessageParameter = (std::same_as<T, std_msgs::msg::String> && std::same_as<U, std::string>) ||
                                (std::same_as<T, std_msgs::msg::Int32> && std::same_as<U, int>);

// ROS related util functions
namespace Utils::ROS
{
template<typename T, typename U>
requires WriteMessageParameter<T, U>
void
writeMessage(T                    message,
             const U              messageData,
             rosbag2_cpp::Writer& writer,
             const QString&       topicName,
             const rclcpp::Time&  timeStamp)
{
    message.data = messageData;
    writer.write(message, topicName.toStdString(), timeStamp);
}


// Returns if a directory contains a ROSBag
[[nodiscard]] bool
doesDirectoryContainBagFile(const QString& bagDirectory);

// Returns if a ROSBag contains a certain topic
[[nodiscard]] bool
doesBagContainTopicName(const QString& bagDirectory,
                        const QString& topicName);

// Returns the message count for a ROSBag topic
[[nodiscard]] int
getTopicMessageCount(const QString& bagDirectory,
                     const QString& topicName);

[[nodiscard]] int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName);

// Returns the metadata for a ROSBag
[[nodiscard]] rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory);

// Returns a ROSBag topic's type
[[nodiscard]] QString
getTopicType(const QString& bagDirectory,
             const QString& topicName);

// Returns all video bag topics stored in a ROSBag
[[nodiscard]] QVector<QString>
getBagVideoTopics(const QString& bagDirectory);

// Returns if a topic name follows the ROS2 naming convention
[[nodiscard]] bool
doesTopicNameFollowROS2Convention(const QString& topicName);
}
