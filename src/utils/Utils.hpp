#pragma once

#include <QWidget>

#include <string>

// Various util functions
namespace Utils
{
[[nodiscard]] bool
doesBagContainTopicName(const std::string& bagDirectory,
                        const std::string& topicName);

[[nodiscard]] int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName);

[[nodiscard]] std::string
getTopicType(const std::string& bagDirectory,
             const std::string& topicName);

[[nodiscard]] std::vector<std::string>
getBagVideoTopics(const std::string& bagDirectory);

[[nodiscard]] std::string
drawProgressString(int progress);

void
setWidgetHeaderFont(QWidget* widget);

[[nodiscard]] bool
isDarkMode();
}
