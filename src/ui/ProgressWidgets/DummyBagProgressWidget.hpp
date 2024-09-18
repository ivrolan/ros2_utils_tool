#pragma once

#include "BasicProgressWidget.hpp"
#include "UtilsUI.hpp"

/**
 * @brief Progress widget variant used to display the writing of a dummy ros bag
 */
class DummyBagProgressWidget : public BasicProgressWidget
{
    Q_OBJECT

public:
    DummyBagProgressWidget(const Utils::UI::DummyBagParameters& dummyBagParameters,
                           QWidget*                             parent = 0);
};
