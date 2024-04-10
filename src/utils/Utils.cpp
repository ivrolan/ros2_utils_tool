#include "Utils.hpp"

#include <QDebug>
#include <QRegularExpression>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <cmath>

namespace Utils
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


std::string
drawProgressString(int progress)
{
    const int numberOfHashtags = ((float) progress / 100.0f) * 50;
    const auto numberOfDashes = 50 - numberOfHashtags;

    const auto progressString = std::string(numberOfHashtags, '#') + std::string(numberOfDashes, '-');
    return progressString;
}


void
setWidgetHeaderFont(QWidget* widget)
{
    auto font = widget->font();
    font.setPointSize(16);
    widget->setFont(font);
}


QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit> lineEdit, QPointer<QToolButton> toolButton)
{
    // Do not let the user add anything, stick to specific dialogs with file directories
    lineEdit->setReadOnly(true);
    toolButton->setText("...");

    auto* const layout = new QHBoxLayout;
    layout->addWidget(lineEdit);
    layout->addWidget(toolButton);

    return layout;
}


bool
isDarkMode()
{
    const QWidget widget;
    const auto color = widget.palette().color(QPalette::Window);
    const auto luminance = sqrt(0.299 * std::pow(color.redF(), 2) +
                                0.587 * std::pow(color.greenF(), 2) +
                                0.114 * std::pow(color.blueF(), 2));
    return luminance < 0.2;
}
}
