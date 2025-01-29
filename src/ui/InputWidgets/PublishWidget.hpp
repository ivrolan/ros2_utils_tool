#pragma once

#include "BasicInputWidget.hpp"
#include "PublishSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLineEdit;

// Widget used to configure publishing a video OR image sequence as ROS messages
class PublishWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    PublishWidget(Utils::UI::PublishParameters& parameters,
                  bool                          checkROS2NameConform,
                  bool                          publishVideo,
                  QWidget*                      parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_topicNameLineEdit;

    Utils::UI::PublishParameters& m_parameters;

    PublishSettings m_settings;

    const bool m_checkROS2NameConform;
    const bool m_publishVideo;
};
