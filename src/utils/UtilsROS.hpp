#pragma once

#include <QString>

#include <string>
#include <vector>

// ROS related util functions
namespace UtilsROS
{
// Returns if a ROSBag contains a certain topic
[[nodiscard]] bool
doesBagContainTopicName(const std::string& bagDirectory,
                        const std::string& topicName);

// Returns the message count for a ROSBag topic
[[nodiscard]] int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName);

// Returns a ROSBag topic's type
[[nodiscard]] std::string
getTopicType(const std::string& bagDirectory,
             const std::string& topicName);

// Returns all video bag topics stored in a ROSBag
[[nodiscard]] std::vector<std::string>
getBagVideoTopics(const std::string& bagDirectory);

// Returns if a topic name follows the ROS2 naming convention
[[nodiscard]] bool
doesTopicNameFollowROS2Convention(const QString& topicName);
}
