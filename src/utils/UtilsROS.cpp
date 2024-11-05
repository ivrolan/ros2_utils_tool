#include "UtilsROS.hpp"

#include <QRegularExpression>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"

namespace Utils::ROS
{
bool
doesDirectoryContainBagFile(const QString& bagDirectory)
{
    rosbag2_cpp::Reader reader;
    try {
        reader.open(bagDirectory.toStdString());
    } catch (...) {
        return false;
    }

    reader.close();
    return true;
}


bool
doesBagContainTopicName(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topicsAndTypes.begin(), topicsAndTypes.end(), [&] (const auto& topic) {
        return topic.name == stdStringTopicName;
    });
    return iter != topicsAndTypes.end();
}


int
getTopicMessageCount(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topics = reader.get_metadata().topics_with_message_count;
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topics.begin(), topics.end(), [&] (const auto& topic) {
        return topic.topic_metadata.name == stdStringTopicName;
    });
    return iter != topics.end() ? iter->message_count : 0;
}


int
getTopicMessageCount(const std::string& bagDirectory, const std::string& topicName)
{
    return getTopicMessageCount(QString::fromStdString(bagDirectory), QString::fromStdString(topicName));
}


rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    return reader.get_metadata();
}


QString
getTopicType(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topicsAndTypes.begin(), topicsAndTypes.end(), [&] (const auto& topic) {
        return topic.name == stdStringTopicName;
    });
    return iter != topicsAndTypes.end() ? QString::fromStdString(iter->type) : "";
}


QVector<QString>
getBagVideoTopics(const QString& bagDirectory)
{
    QVector<QString> videoTopics;
    if (const auto doesDirContainBag = doesDirectoryContainBagFile(bagDirectory); !doesDirContainBag) {
        return videoTopics;
    }

    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    for (const auto topicsAndTypes = reader.get_all_topics_and_types(); const auto& topicAndType : topicsAndTypes) {
        if (topicAndType.type == "sensor_msgs/msg/Image") {
            videoTopics.push_back(QString::fromStdString(topicAndType.name));
        }
    }
    reader.close();
    return videoTopics;
}


bool
doesTopicNameFollowROS2Convention(const QString& topicName)
{
    // Only may contain A-z, a-z, 0-9, _ and /
    QRegularExpression regularExpression("[^A-Za-z0-9/_{}~]");
    if (topicName.contains(regularExpression)) {
        return false;
    }
    // Must not end with /, must not contain __ and //
    if (topicName.endsWith('/') || topicName.contains("//") || topicName.contains("__")) {
        return false;
    }
    // First character must not contain a number
    const auto firstCharacter = QString(topicName.front());
    regularExpression.setPattern("[0-9]");
    if (firstCharacter.contains(regularExpression)) {
        return false;
    }
    // If a ~ is contained, the next character must be a /
    for (auto i = 0; i < topicName.length() - 1; i++) {
        if (topicName.at(i) == '~' && topicName.at(i + 1) != '/') {
            return false;
        }
    }
    // Must have balanced curly braces
    return topicName.count(QLatin1Char('{')) == topicName.count(QLatin1Char('}'));
}
}
