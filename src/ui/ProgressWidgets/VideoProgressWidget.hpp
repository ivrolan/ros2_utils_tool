#pragma once

#include "BasicProgressWidget.hpp"
#include "UtilsUI.hpp"

/**
 * @brief Progress widget variant used to display video encoding or writing a video to a bag
 */
class VideoProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    VideoProgressWidget(const Utils::UI::VideoParameters& videoParameters,
                        QWidget*                          parent = 0);
};
