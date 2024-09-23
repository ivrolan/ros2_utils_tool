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
    const auto videoTopics = Utils::ROS::getBagVideoTopics(bagDirectory.toStdString());
    if (videoTopics.empty()) {
        return false;
    }

    comboBox->clear();
    for (const auto& videoTopic : videoTopics) {
        comboBox->addItem(QString::fromStdString(videoTopic));
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
