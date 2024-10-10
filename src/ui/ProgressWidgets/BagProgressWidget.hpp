#pragma once

#include "BasicProgressWidget.hpp"
#include "UtilsUI.hpp"

/**
 * @brief Progress widget variant used to display writing a video to a ROSBag
 */
class BagProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    BagProgressWidget(const Utils::UI::BagParameters& bagParameters,
                      QWidget*                        parent = 0);
};
