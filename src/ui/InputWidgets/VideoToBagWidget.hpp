#pragma once

#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QLineEdit;

/**
 * @brief The widget used to write a video file into a ROSBag
 */
class VideoToBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Utils::UI::BagParameters& bagParameters,
                     QWidget*                  parent = 0);

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

    Utils::UI::BagParameters& m_bagParameters;
};
