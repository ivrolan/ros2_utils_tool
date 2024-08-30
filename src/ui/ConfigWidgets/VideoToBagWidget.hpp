#pragma once

#include "BasicConfigWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QLineEdit;

/**
 * @brief The widget used to write a video file into a ROSBag
 */
class VideoToBagWidget : public BasicConfigWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Utils::UI::VideoParameters& videoParameters,
                     QWidget*                    parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    bagLocationButtonPressed();

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_videoNameLineEdit;
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QLineEdit> m_topicNameLineEdit;
    QPointer<QCheckBox> m_useHardwareAccCheckBox;

    Utils::UI::VideoParameters& m_videoParameters;
};
