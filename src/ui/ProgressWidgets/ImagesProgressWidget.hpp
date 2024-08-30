#pragma once

#include "BasicProgressWidget.hpp"
#include "UtilsUI.hpp"

/**
 * @brief Progress widget variant used to display writing images out of a ROSBag
 */
class ImagesProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    ImagesProgressWidget(const Utils::UI::ImageParameters& imageParameters,
                         QWidget*                          parent = 0);
};
