#include "UtilsUI.hpp"

#include "UtilsROS.hpp"

#include <QMessageBox>

#include <cmath>

namespace Utils::UI
{
void
setWidgetFontSize(QWidget* widget, bool isButton)
{
    auto font = widget->font();
    font.setPointSize(isButton ? FONT_SIZE_BUTTON : FONT_SIZE_HEADER);
    widget->setFont(font);
}


bool
fillComboBoxWithTopics(QPointer<QComboBox> comboBox, const QString& bagDirectory)
{
    const auto videoTopics = Utils::ROS::getBagVideoTopics(bagDirectory);
    if (videoTopics.empty()) {
        return false;
    }

    comboBox->clear();
    for (const auto& videoTopic : videoTopics) {
        comboBox->addItem(videoTopic);
    }

    return true;
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


QMessageBox*
createInvalidROSNameMessageBox()
{
    const auto headerText = "Renamed topic name(s) invalid!";
    const auto mainText = "The renamed topic name(s) do not follow the ROS2 naming convention! More information can be found here:<br>"
                          "<a href='https://design.ros2.org/articles/topic_and_service_names.html'>https://design.ros2.org/articles/topic_and_service_names.html</a><br>"
                          "Do you still want to continue?";
    auto *const msgBox = new QMessageBox(QMessageBox::Warning, headerText, mainText, QMessageBox::Yes | QMessageBox::No);
    return msgBox;
}


void
createCriticalMessageBox(const QString& headerText, const QString& mainText)
{
    auto *const msgBox = new QMessageBox(QMessageBox::Critical, headerText, mainText, QMessageBox::Ok);
    msgBox->exec();
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
