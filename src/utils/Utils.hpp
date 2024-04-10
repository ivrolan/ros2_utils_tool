#pragma once

#include <QWidget>

#include <string>

// Various util functions
namespace Utils
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

// Draws a small progress string in the following format:
// ############################--------------------
// 50 charactes, # shows the progress
[[nodiscard]] std::string
drawProgressString(int progress);

// Create a larger font for a certain widget
void
setWidgetHeaderFont(QWidget* widget);

// Checks if the application is using a dark mode
[[nodiscard]] bool
isDarkMode();
}
