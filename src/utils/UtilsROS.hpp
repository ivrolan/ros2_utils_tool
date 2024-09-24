#pragma once

#include <QString>

#include "rosbag2_storage/bag_metadata.hpp"

#include <string>
#include <vector>

// ROS related util functions
namespace Utils::ROS
{
// Returns if a directory contains a ROSBag
[[nodiscard]] bool
doesDirectoryContainBagFile(const std::string& bagDirectory);

// Returns if a ROSBag contains a certain topic
[[nodiscard]] bool
doesBagContainTopicName(const std::string& bagDirectory,
                        const std::string& topicName);

// Returns the message count for a ROSBag topic
[[nodiscard]] int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName);

// Returns the metadata for a ROSBag
[[nodiscard]] rosbag2_storage::BagMetadata
getBagMetadata(const std::string& bagDirectory);

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
