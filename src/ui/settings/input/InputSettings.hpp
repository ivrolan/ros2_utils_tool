#pragma once

#include "BasicSettings.hpp"
#include "UtilsUI.hpp"

// Store parameters used by all input widgets
class InputSettings : public BasicSettings {
public:
    InputSettings(Utils::UI::InputParameters& parameters,
                  const QString&              groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::InputParameters& m_parameters;
};
