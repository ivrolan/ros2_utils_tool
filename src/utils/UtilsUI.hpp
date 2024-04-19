#pragma once

#include <QHBoxLayout>
#include <QLineEdit>
#include <QPointer>
#include <QToolButton>
#include <QWidget>

#include <string>

// Util functions for user interface related things
namespace Utils::UI
{
// Create a larger font for a certain widget
void
setWidgetHeaderFont(QWidget* widget);

// Creates a layout of a lineedit along with a tool button
[[nodiscard]] QHBoxLayout*
createLineEditButtonLayout(QPointer<QLineEdit>   lineEdit,
                           QPointer<QToolButton> toolButton);

// Checks if the application is using a dark mode
[[nodiscard]] bool
isDarkMode();
}
