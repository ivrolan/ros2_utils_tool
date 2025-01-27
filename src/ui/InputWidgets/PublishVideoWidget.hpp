#pragma once

#include "BasicInputWidget.hpp"
#include "PublishVideoSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLineEdit;

/**
 * @brief The widget used to configure publishing a video as ROS message
 */
class PublishVideoWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    PublishVideoWidget(Utils::UI::PublishVideoParameters& parameters,
                       bool                               checkROS2NameConform,
                       QWidget*                           parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_topicNameLineEdit;

    Utils::UI::PublishVideoParameters& m_parameters;

    PublishVideoSettings m_settings;

    const bool m_checkROS2NameConform;
};
