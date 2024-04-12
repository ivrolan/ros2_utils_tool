#include "UtilsROS.hpp"

#include <QRegularExpression>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"

namespace UtilsROS
{
bool
doesBagContainTopicName(const std::string& bagDirectory,
                        const std::string& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory);

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    for (const auto& topicAndType : topicsAndTypes) {
        if (topicAndType.name == topicName) {
            return true;
        }
    }
    return false;
}


int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory);

    const auto& metaData = reader.get_metadata();
    for (const auto& topic : metaData.topics_with_message_count) {
        if (topic.topic_metadata.name == topicName) {
            return topic.message_count;
        }
    }
    return 0;
}


std::string
getTopicType(const std::string& bagDirectory,
             const std::string& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory);

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    for (const auto& topicAndType : topicsAndTypes) {
        if (topicAndType.name == topicName) {
            return topicAndType.type;
        }
    }
    return "";
}


std::vector<std::string>
getBagVideoTopics(const std::string& bagDirectory)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory);
    std::vector<std::string> videoTopics;

    const auto topicsAndTypes = reader.get_all_topics_and_types();
    for (const auto& topicAndType : topicsAndTypes) {
        if (topicAndType.type == "sensor_msgs/msg/Image") {
            videoTopics.push_back(topicAndType.name);
        }
    }
    reader.close();
    return videoTopics;
}


bool
doesTopicNameFollowROS2Convention(const QString& topicName)
{
    auto doesFollow = true;
    doesFollow &= !topicName.endsWith('/') && !topicName.contains("//") && !topicName.contains("__");

    const auto firstCharacter = QString(topicName.front());
    QRegularExpression regularExpression("[0-9]");
    doesFollow &= !firstCharacter.contains(regularExpression);

    return doesFollow;
}
}
